#ifndef TemperatureModel_H
#define TemperatureModel_H
/**
  * @file TemperatureModel.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: Temperature Sensor Model - simple model publishing temperature(C)
*/

#include "ros/ros.h"
#include <sensor_mock_lib/SensorMockSrv.h>
#include "../../sensor_mock_lib/include/DataProfile.h"

namespace RobotManager::SensorMock {

class TemperatureModel
{
public:
    TemperatureModel(double initialSoC = 0.0);

    void publishData(const ros::Time& currentTime);

private:
    bool setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response);

private:
    ros::NodeHandle m_nodeHandle;
    ros::ServiceServer m_cmdService;
    ros::Publisher m_publisher;

    RobotManager::Utils::DataProfile m_dataProfile;

};

} //END namespace

#endif // TemperatureModel_H
