/**
  * @file BatteryModel.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Battery Sensor Model - simple model publishing state of charge (SoC)
*/

#include "BatteryModel.h"
#include "BatteryMockStrings.h"

#include <functional>

namespace RobotManager::SensorMock {

BatteryModel::BatteryModel(double initialSoC):
    m_nodeHandle(),
    m_batteryCmdService(),
    m_publisher(),
    m_dataProfile(initialSoC)
{
    m_batteryCmdService = m_nodeHandle.advertiseService(
                BATTERYMOCK_SERVICE,
                &BatteryModel::setBatteryProfile,
                this);

    /**
     * Buffer size of 1 is used - No point in keeping all battery data
     *
     */
    m_publisher = m_nodeHandle.advertise<battery_mock::BatteryMsg>(BATTERYMOCK_TOPIC, 1);
}

void BatteryModel::publishBatteryData(const ros::Time& currentTime)
{
    /**
     * Update SoC - simple model assumes linear loss of charge over time
     * Optional future upgrade: assume additional loss proportional to acceneration or speed
     * (would need to subscribe for the robot data ROS topic)
     *
     */

    const auto currentTimeSeconds = currentTime.toSec();

    battery_mock::BatteryMsg batteryMsg;

    batteryMsg.stateOfCharge = m_dataProfile.getValue(currentTime.toSec());
    batteryMsg.timestamp = currentTime;

    //ROS_INFO("Publishing battery SoC: %f", double(batteryMsg.stateOfCharge));

    /**
    *  Data acquisition from a real BMS (battery managment system) might fail.
    * This is to indicate if the data contained in the message is valid.
    */
    batteryMsg.dataOk = true;

    m_publisher.publish(batteryMsg);
}

bool BatteryModel::setBatteryProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response)
{
    ROS_INFO("request: size %d", int(request.times.size()));

    if(request.times.size() == 0 || request.times.size() != request.values.size())
    {
        std::stringstream errStr;
        errStr << "ERROR: invalid data size " << request.times.size();
        response.result = errStr.str();
        ROS_ERROR("%s", response.result.c_str());
    }
    else
    {
        m_dataProfile.reset();
        auto dataSize = request.times.size();
        for(typeof(dataSize) idx = 0; idx < dataSize; ++idx)
        {
            m_dataProfile.addDataPoint(request.times[idx], request.values[idx]);
        }
        m_dataProfile.setStartTime(ros::Time::now().toSec());
        response.result = "[Battery Mock] Profile accepted";
    }

    return true;
}

} //END namespace
