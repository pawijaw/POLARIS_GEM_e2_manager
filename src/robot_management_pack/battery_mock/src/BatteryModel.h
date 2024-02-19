#ifndef BATTERYMODEL_H
#define BATTERYMODEL_H
/**
  * @file BatteryModel.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Battery Sensor Model - simple model publishing state of charge (SoC)
*/

#include "ros/ros.h"
#include <battery_mock/BatteryMsg.h>
#include <sensor_mock_lib/SensorMockSrv.h>
#include "../../sensor_mock_lib/include/DataProfile.h"

namespace RobotManager::SensorMock {

class BatteryModel
{
public:
    BatteryModel(double initialSoC = 0.0);

    void publishBatteryData(const ros::Time& currentTime);

private:
    bool setBatteryProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response);

private:
    ros::NodeHandle m_nodeHandle;
    ros::ServiceServer m_batteryCmdService;
    ros::Publisher m_publisher;

    RobotManager::Utils::DataProfile m_dataProfile;

};

} //END namespace

#endif // BATTERYMODEL_H
