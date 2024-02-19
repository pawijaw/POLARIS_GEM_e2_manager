/**
  * @file gps_main.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: GPS Sensor Mockup - main
*/

#include "GpsModel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_mock");
    ros::NodeHandle nodeHandle;

    double initialAccuracy;
    nodeHandle.param("init_accuracy", initialAccuracy, 100.0);
    ROS_INFO("GPS signal initial accuracy: %f mm", initialAccuracy);

    int rate;
    nodeHandle.param("publish_rate", rate, 10);
    ROS_INFO("GPS Mock publish rate: %d", rate);

    ros::Rate loop_rate(rate);
    RobotManager::SensorMock::GpsModel gpsModel(initialAccuracy);

    while (ros::ok())
    {
        ros::spinOnce();
        gpsModel.publishData(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}

