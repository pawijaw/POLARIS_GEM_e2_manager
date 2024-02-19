/**
  * @file temperature.main.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: Battery Sensor Mockup - main
*/

#include "TemperatureModel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temperature_mock");
    ros::NodeHandle nodeHandle;

    double initialTemp;
    nodeHandle.param("init_temp", initialTemp, 10.0);
    ROS_INFO("Temperature Mock initial temperature: %f", initialTemp);

    int rate;
    nodeHandle.param("publish_rate", rate, 10);
    ROS_INFO("Temperature Mock publish rate: %d", rate);

    ros::Rate loop_rate(rate);
    RobotManager::SensorMock::TemperatureModel temperatureModel(initialTemp);

    while (ros::ok())
    {
        ros::spinOnce();
        temperatureModel.publishData(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}

