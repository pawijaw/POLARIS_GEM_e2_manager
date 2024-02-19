#ifndef V2xModel_H
#define V2xModel_H
/**
  * @file V2xModel.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: V2x Sensor Model - simple model publishing signal quality
*/

#include "ros/ros.h"
#include <sensor_mock_lib/SensorMockSrv.h>
#include "../../sensor_mock_lib/include/DataProfile.h"

namespace RobotManager::SensorMock {

class V2xModel
{
public:
    V2xModel(int initialStrength);

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

#endif // V2xModel_H
