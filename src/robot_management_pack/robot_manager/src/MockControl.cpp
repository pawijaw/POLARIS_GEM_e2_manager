#include "MockControl.h"

#include <sensor_mock_lib/SensorMockSrv.h>
#include "./../../battery_mock/include/BatteryMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../temperature_mock/include/TemperatureMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../gps_mock/include/GpsMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../v2x_mock/include/V2xMockStrings.h" //TODO: Properly export the header form ROS package
#include "./../../estop_mock/include/EstopMockStrings.h" //TODO: Properly export the header form ROS package

#include <iostream>
#include <fstream>

namespace RobotManager {

MockControl::MockControl()
{
//empty
}

std::string MockControl::sendBatteryDemand(const std::string& dataFile)
{
    return sendMockSensorDemandDenamd(dataFile, BATTERYMOCK_SERVICE);
}

std::string MockControl::sendTemperatureDemand(const std::string& dataFile)
{
    return sendMockSensorDemandDenamd(dataFile, TEMPMOCK_SERVICE);
}

std::string MockControl::sendGpsDemand(const std::string& dataFile)
{
    return sendMockSensorDemandDenamd(dataFile, GPSMOCK_SERVICE);
}

std::string MockControl::sendV2xDemand(const std::string& dataFile)
{
    return sendMockSensorDemandDenamd(dataFile, V2XMOCK_SERVICE);
}

std::string MockControl::sendEstopDemand(const std::string& dataFile)
{
    return sendMockSensorDemandDenamd(dataFile, ESTOPMOCK_SERVICE);
}

std::string MockControl::sendMockSensorDemandDenamd(const std::string& dataFile, const std::string& service)
{
    std::stringstream resStr;
    std::cout << "Request to service " << service << std::endl;

    auto sensorMockClient = m_node.serviceClient<sensor_mock_lib::SensorMockSrv>(service);

    sensor_mock_lib::SensorMockSrv mockSrvRequest;

    std::ifstream dataIn;
    dataIn.open(dataFile);

    if(!dataIn.good())
    {
        return "Failed to open file";
    }

    std::string dataLine;

    while(std::getline(dataIn, dataLine))
    {
        std::istringstream iss(dataLine);

        double time, value;
        if (!(iss >> time >> value))
        {
            // error
            resStr << "Failed to read line " << dataLine << " from file " << dataFile;
            ROS_ERROR("%s", resStr.str().c_str());
            return resStr.str();
        }

        mockSrvRequest.request.times.push_back(time);
        mockSrvRequest.request.values.push_back(value);
    }
    dataIn.close();

    sensorMockClient.call(mockSrvRequest);

    return mockSrvRequest.response.result;
}

} //END namespace
