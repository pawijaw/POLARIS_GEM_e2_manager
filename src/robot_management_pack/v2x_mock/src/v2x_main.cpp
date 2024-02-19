/**
  * @file v2x_main.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: V2x Sensor Mockup - main
*/

#include "V2xModel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "v2x_mock");
    ros::NodeHandle nodeHandle;

    int initialState;
    nodeHandle.param("init_state", initialState, 2);
    ROS_INFO("V2x Mock initial state: %d", initialState);

    int rate;
    nodeHandle.param("publish_rate", rate, 10);
    ROS_INFO("V2x Mock publish rate: %d", rate);

    ros::Rate loop_rate(rate);
    RobotManager::SensorMock::V2xModel v2xModel(initialState);

    while (ros::ok())
    {
        ros::spinOnce();
        v2xModel.publishData(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}

