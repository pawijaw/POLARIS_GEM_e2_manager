#ifndef RobotManager_H
#define RobotManager_H

#include <thread>
#include <mutex>

#include "RobotStateMachine.h"

namespace RobotManager {

class RobotManager
{
    using Mutex_t = std::mutex;
    using Lock_t = std::lock_guard<Mutex_t>;
public:
    RobotManager(int spinThreadCount);

    /**
     * @brief getVehicleData - get current vehicle data.
     * @return Copy of sensor data. Return by value by design - overhead while copying, but then coherent snapshot returned
     */
    VehicleData getVehicleData();

    /**
     * @brief setNavigation
     * @param navService
     * @param waypointsFile - sile to load waypoints from or empty string to cancel navigation
     * @return std::pair <bool,string> - was the command sucessful?
     * It can fail if the NAV service is unreachable, refuses the command or if the vehicle is in error statemessage string
     *  second pair member is the associated message string. e.g. reason for failure.
     */
    std::pair<bool, std::string> setNavigation(const std::string& navService, const std::string& waypointsFile);

private:
    void handleBatteryData(const battery_mock::BatteryMsg::ConstPtr& batteryMsg);
    void handleTemperatureData(const temperature_mock::TemperatureMsg::ConstPtr& temperatureMsg);
    void handleGpsData(const gps_mock::GpsMsg::ConstPtr& gpsMsg);
    void handleV2xData(const v2x_mock::V2xMsg::ConstPtr& v2xMsg);
    void handleEstopData(const estop_mock::EstopMsg::ConstPtr& estopMsg);

    std::pair<bool, std::string> navigationRequest(const std::string& navService, const std::string& waypointsFile);

    void loadVehicleDataFromSim();

    void processStateMachine();

    void setInitialValues();

private:
    ros::NodeHandle m_nodeHandle;
    ros::AsyncSpinner m_rosSpinner;

    ros::Subscriber m_batterySub;
    ros::Subscriber m_temperatureSub;
    ros::Subscriber m_gpsSub;
    ros::Subscriber m_v2xSub;
    ros::Subscriber m_estopSub;

    VehicleData m_vehicleData;
    mutable Mutex_t m_sensorDataMutex; //mutable to allow usage in const members

    RobotStateMachine m_robotStateMachine;

    std::string m_navWaypointsFile;
};

} //END namespace

#endif // RobotManager_H
