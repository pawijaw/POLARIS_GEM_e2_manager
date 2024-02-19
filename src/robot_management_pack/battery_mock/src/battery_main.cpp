/**
  * @file battery.main.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Battery Sensor Mockup - main
*/

#include "BatteryModel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_mock");
    ros::NodeHandle nodeHandle;

    double initialSoc;
    nodeHandle.param("init_soc", initialSoc, 60.0);
    ROS_INFO("Battery Mock initial SoC: %f", initialSoc);

    int rate;
    nodeHandle.param("publish_rate", rate, 10);
    ROS_INFO("Battery Mock publish rate: %d", rate);

    ros::Rate loop_rate(rate);
    RobotManager::SensorMock::BatteryModel batteryModel(initialSoc);

    while (ros::ok())
    {
        ros::spinOnce();
        batteryModel.publishBatteryData(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}

