#ifndef GpsModel_H
#define GpsModel_H
/**
  * @file GpsModel.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: GPS Sensor Model - simple model publishing x,y from Gazepb and models signal state
*/

#include "ros/ros.h"
#include <sensor_mock_lib/SensorMockSrv.h>
#include <gazebo_msgs/GetModelState.h>
#include "../../sensor_mock_lib/include/DataProfile.h"

namespace RobotManager::SensorMock {

class GpsModel
{
public:
    GpsModel(double initialAccuracy);

    void publishData(const ros::Time& currentTime);

private:
    bool setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response);

private:
    ros::NodeHandle m_nodeHandle;
    ros::ServiceServer m_cmdService;
    ros::Publisher m_publisher;

    ros::ServiceClient m_gazeboClient;
    gazebo_msgs::GetModelState m_gazeboReqMsg;

    RobotManager::Utils::DataProfile m_dataProfile;

};

} //END namespace

#endif // GpsModel_H
