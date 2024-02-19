#ifndef MockControl_H
#define MockControl_H

#include "ros/ros.h"
#include <string>

namespace RobotManager {


class MockControl
{
public:
    MockControl();

    std::string sendBatteryDemand(const std::string& dataFile);
    std::string sendTemperatureDemand(const std::string& dataFile);
    std::string sendGpsDemand(const std::string& dataFile);
    std::string sendV2xDemand(const std::string& dataFile);
    std::string sendEstopDemand(const std::string& dataFile);

private:
    std::string sendMockSensorDemandDenamd(const std::string& dataFile, const std::string& service);

private:

    ros::NodeHandle m_node;
    

};



} //END namespace

#endif // MockControl_H
