/**
  * @file tii_estop_main.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: E-Stop Sensor Mockup - main
*/

#include "EstopModel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estop_mock");
    ros::NodeHandle nodeHandle;

    int initialState;
    nodeHandle.param("init_state", initialState, 0);
    ROS_INFO("E-Stop Mock initial state: %d", initialState);

    int rate;
    nodeHandle.param("publish_rate", rate, 10);
    ROS_INFO("E-Stop Mock publish rate: %d", rate);

    ros::Rate loop_rate(rate);
    RobotManager::SensorMock::EstopModel estopModel(initialState);

    while (ros::ok())
    {
        ros::spinOnce();
        estopModel.publishData(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}

