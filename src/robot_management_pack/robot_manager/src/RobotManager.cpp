#include "RobotManager.h"
#include "RobotManagerDefs.h"
#include "./../../battery_mock/include/BatteryMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../temperature_mock/include/TemperatureMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../gps_mock/include/GpsMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../v2x_mock/include/V2xMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../estop_mock/include/EstopMockStrings.h" //TODO: Properly export the header form ROS package

#include <gazebo_msgs/GetModelState.h>
#include <gem_pure_pursuit_sim/WaypointRequestSrv.h>

#include <fstream>

namespace RobotManager {

RobotManager::RobotManager(int spinThreadCount):
    m_rosSpinner(spinThreadCount),
    m_vehicleData(),
    m_robotStateMachine()
{
    while(ros::Time::now().toSec() == 0.0) // wait for ROS time
    {
        ros::Duration(0.1).sleep();
    }

    setInitialValues();

    m_batterySub = m_nodeHandle.subscribe(BATTERYMOCK_TOPIC, 1, &RobotManager::handleBatteryData, this);
    m_temperatureSub = m_nodeHandle.subscribe(TEMPMOCK_TOPIC, 1, &RobotManager::handleTemperatureData, this);
    m_gpsSub = m_nodeHandle.subscribe(GPSMOCK_TOPIC, 1, &RobotManager::handleGpsData, this);
    m_v2xSub = m_nodeHandle.subscribe(V2X_TOPIC, 1, &RobotManager::handleV2xData, this);
    m_estopSub = m_nodeHandle.subscribe(ESTOP_TOPIC, 1, &RobotManager::handleEstopData, this);

    m_rosSpinner.start();
}

void RobotManager::handleBatteryData(const battery_mock::BatteryMsg::ConstPtr& batteryMsg)
{
    // ROS_INFO("received battery SoC: %f", double(batteryMsg->stateOfCharge));

    const auto receiveTime = ros::Time::now().toSec(); // capture receive time before critical section - possible delay at mutex

    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.batteryData.batteryMsg = *batteryMsg;
    m_vehicleData.batteryData.receiveTimestamp = receiveTime;

    //notify state machine
    processStateMachine();
}

void RobotManager::handleTemperatureData(const temperature_mock::TemperatureMsg::ConstPtr& temperatureMsg)
{
    // ROS_INFO("received temp : %f", double(temperatureMsg->temperature));
    const auto receiveTime = ros::Time::now().toSec(); // capture receive time before critical section - possible delay at mutex

    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.temperatureData.temperatureMsg = *temperatureMsg;
    m_vehicleData.temperatureData.receiveTimestamp = receiveTime;

    //notify state machine
    processStateMachine();
}

void RobotManager::handleGpsData(const gps_mock::GpsMsg::ConstPtr& gpsMsg)
{
    // ROS_INFO("received GPS x=%f y=%f accuracy=%f mm", gpsMsg->x, gpsMsg->y, gpsMsg->horizontalAccuracy);
    const auto receiveTime = ros::Time::now().toSec(); // capture receive time before critical section - possible delay at mutex

    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.gpsData.gpsMsg = *gpsMsg;
    m_vehicleData.gpsData.receiveTimestamp = receiveTime;

    //notify state machine
    processStateMachine();
}

void RobotManager::handleV2xData(const v2x_mock::V2xMsg::ConstPtr& v2xMsg)
{
    // ROS_INFO("received V2x status %d", v2xMsg->signalStrength);
    const auto receiveTime = ros::Time::now().toSec(); // capture receive time before critical section - possible delay at mutex

    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.v2xData.v2xMsg = *v2xMsg;
    m_vehicleData.v2xData.receiveTimestamp = receiveTime;

    //notify state machine
    processStateMachine();
}

void RobotManager::handleEstopData(const estop_mock::EstopMsg::ConstPtr& estopMsg)
{
   // ROS_INFO("received estop status %d", estopMsg->estopTriggered);
    const auto receiveTime = ros::Time::now().toSec(); // capture receive time before critical section - possible delay at mutex

    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.estopData.estopMsg = *estopMsg;
    m_vehicleData.estopData.receiveTimestamp = receiveTime;

    //notify state machine
    processStateMachine();
}

VehicleData RobotManager::getVehicleData()
{
    Lock_t lock(m_sensorDataMutex);
    VehicleData copy = m_vehicleData;
    m_vehicleData.logs.clear();
    return copy; //return a copy
}

std::pair<bool, std::string> RobotManager::setNavigation(const std::string& navService, const std::string& waypointsFile)
{
    Lock_t lock(m_sensorDataMutex);

    m_vehicleData.activeNavService.clear();
    m_navWaypointsFile.clear();

    if(m_vehicleData.state == RobotState::ERROR)
    {
        return std::make_pair(false, "Cannot start navigation in ERROR state");
    }

    auto navRequestResult = navigationRequest(navService, waypointsFile);
    if(navRequestResult.first && waypointsFile.size() > 0)
    {
        m_navWaypointsFile = waypointsFile;
        m_vehicleData.activeNavService = navService;
    }

    return navRequestResult;
}

std::pair<bool, std::string> RobotManager::navigationRequest(const std::string& navService, const std::string& waypointsFile)
{
    auto waypointService = m_nodeHandle.serviceClient<gem_pure_pursuit_sim::WaypointRequestSrv>(navService);
    gem_pure_pursuit_sim::WaypointRequestSrv waypointSrv;
    std::stringstream returnMessage;

    if(waypointsFile.size() == 0) // deactivate navigation
    {
        waypointService.call(waypointSrv);
        if(waypointSrv.response.success)
        {
            ROS_INFO("NAV service confirmed deactivation at %f", ros::Time::now().toSec());
        }
        else
        {
            ROS_ERROR("NAV service failed to deactivate at %f", ros::Time::now().toSec());
        }
        returnMessage << waypointSrv.response.message;
        return std::make_pair(waypointSrv.response.success, returnMessage.str());
    }

    std::ifstream file(waypointsFile);

    if(!file.is_open())
    {
        return std::make_pair(false, "Failed to open file " + waypointsFile);
    }

    std::string line, xStr, yStr, yawStr;

    try
    {
        //read CSV
        while(getline(file, line))
        {
            std::stringstream lineStream(line);
            getline(lineStream, xStr, ',') && getline(lineStream, yStr, ',') && getline(lineStream, yawStr, ',');
            waypointSrv.request.xVals.push_back(std::stod(xStr));
            waypointSrv.request.yVals.push_back(std::stod(yStr));
            waypointSrv.request.yawVals.push_back(std::stod(yawStr));
        }

        waypointService.call(waypointSrv);
        returnMessage << waypointSrv.response.message;

        return std::make_pair(waypointSrv.response.success, returnMessage.str());
    }
    catch (const std::runtime_error& err)
    {
        returnMessage << "Error loading waypoints: " << err.what();
        ROS_ERROR("%s", returnMessage.str().c_str());
        return std::make_pair(false, returnMessage.str());
    }
}

void RobotManager::processStateMachine()
{
    static RobotState oldState = RobotState::IDLE;

    auto stateMachineResult = m_robotStateMachine.processStateMachine(m_vehicleData, ros::Time::now().toSec());

    if(stateMachineResult.first != oldState) // if a state change has occured
    {
        if(stateMachineResult.first == RobotState::ERROR && m_vehicleData.activeNavService.size() > 0)
        {
            //entered error state
            //deactivate navigation service - asynchoronous call to
            ROS_INFO("Robot entered ERROR state - requesting deactivation of NAV service");
            stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "Requesting deactivation of NAV service");
            const auto navResult = navigationRequest(m_vehicleData.activeNavService, "");
            if(navResult.first)
            {
                ROS_INFO("NAV service deactivation sucessful");
                stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "NAV service deactivated");
            }
            else
            {
                ROS_ERROR("Failed to deactivate NAV service");
                stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "Failed to deactivate NAV service");
            }
        }
        else if(oldState == RobotState::ERROR && stateMachineResult.first == RobotState::RUNNING) //retrun to RUNNING after error
        {
            if(m_vehicleData.activeNavService.size() > 0 && m_navWaypointsFile.size() > 0)
            {
                ROS_INFO("Robot recovered from ERROR state at %f", ros::Time::now().toSec());
                stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "Requesting re-activation of NAV service");
                const auto navResult = navigationRequest(m_vehicleData.activeNavService, m_navWaypointsFile);
                if(navResult.first)
                {
                    ROS_INFO("NAV service reactivated sucessfully");
                    stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "NAV service reactivated sucessfully");
                }
                else
                {
                    ROS_ERROR("Failed to redeactivate NAV service");
                    stateMachineResult.second.emplace_back(ros::Time::now().toSec(), "Failed to redeactivate NAV service");
                }

            }
//           else // do nothing - navigation was cancelled form GUI while we were in ERROR state

        }
    }

    m_vehicleData.logs.insert(m_vehicleData.logs.end(), stateMachineResult.second.begin(), stateMachineResult.second.end());
    m_vehicleData.state = stateMachineResult.first;
    oldState = stateMachineResult.first;
}

void RobotManager::setInitialValues()
{
    //set initial timestamps and values to avoid triggering error state before first messages arrrive

    const auto startTime = ros::Time::now();
    m_vehicleData.batteryData.receiveTimestamp = startTime.toSec();
    m_vehicleData.temperatureData.receiveTimestamp = startTime.toSec();
    m_vehicleData.gpsData.receiveTimestamp = startTime.toSec();
    m_vehicleData.v2xData.receiveTimestamp = startTime.toSec();
    m_vehicleData.estopData.receiveTimestamp = startTime.toSec();

    m_vehicleData.batteryData.batteryMsg.timestamp = startTime;
    m_vehicleData.temperatureData.temperatureMsg.timestamp = startTime;
    m_vehicleData.gpsData.gpsMsg.timestamp = startTime;
    m_vehicleData.v2xData.v2xMsg.timestamp = startTime;
    m_vehicleData.estopData.estopMsg.timestamp = startTime;

    m_vehicleData.batteryData.batteryMsg.stateOfCharge = DEFAULT_BATTERY_SENSOR_MIN_SOC + 1.0;
    m_vehicleData.temperatureData.temperatureMsg.temperature = DEFAULT_TEMPERATURE_MAX - 1.0;
    m_vehicleData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD - 1.0;
    m_vehicleData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_GOOD;
    m_vehicleData.estopData.estopMsg.estopTriggered = false;
}


} //END namespace

