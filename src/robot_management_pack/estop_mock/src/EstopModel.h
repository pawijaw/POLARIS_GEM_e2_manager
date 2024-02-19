#ifndef EstopModel_H
#define EstopModel_H
/**
  * @file EstopModel.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: Emergency Stop Sensor Model - simple model publishing E-Stop button status
*/

#include "ros/ros.h"
#include <sensor_mock_lib/SensorMockSrv.h>
#include "../../sensor_mock_lib/include/DataProfile.h"

namespace RobotManager::SensorMock {

class EstopModel
{
public:
    EstopModel(int initialValue);

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

#endif // EstopModel_H
