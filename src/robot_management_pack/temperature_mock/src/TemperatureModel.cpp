/**
  * @file TemperatureModel.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Battery Sensor Model - simple model publishing state of charge (SoC)
*/

#include "TemperatureModel.h"
#include "TemperatureMockStrings.h"

#include <temperature_mock/TemperatureMsg.h>

#include <functional>

namespace RobotManager::SensorMock {

TemperatureModel::TemperatureModel(double initialSoC):
    m_nodeHandle(),
    m_cmdService(),
    m_publisher(),
    m_dataProfile(initialSoC)
{
    // m_rampProfile.setProfile(initialSoC, initialSoC, 0.0, 0.0);

    m_cmdService = m_nodeHandle.advertiseService(
                TEMPMOCK_SERVICE,
                &TemperatureModel::setProfile,
                this);

    /**
     * Buffer size of 1 is used - No point in keeping all battery data
     *
     */
    m_publisher = m_nodeHandle.advertise<temperature_mock::TemperatureMsg>(TEMPMOCK_TOPIC, 1);
}

void TemperatureModel::publishData(const ros::Time& currentTime)
{
    const auto currentTimeSeconds = currentTime.toSec();

    temperature_mock::TemperatureMsg outMsg;

    outMsg.temperature = m_dataProfile.getValue(currentTime.toSec());
    outMsg.timestamp = currentTime;

   // ROS_INFO("Publishing temperature: %f C", double(outMsg.temperature));

    /**
    *  Data acquisition from a real sensor might fail. This is to indicate if the data contained in the message is valid.
    *  Useful for future modelling of advanced error states (e.g. sensor failure)
    */
    outMsg.dataOk = true;

    m_publisher.publish(outMsg);
}

bool TemperatureModel::setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response)
{
    if(request.times.size() == 0 || request.times.size() != request.values.size())
    {
        std::stringstream errStr;
        errStr << "ERROR: invalid data size " << request.times.size();
        response.result = errStr.str();
        ROS_ERROR("%s", response.result.c_str());
    }
    else
    {
        m_dataProfile.reset();
        auto dataSize = request.times.size();
        for(typeof(dataSize) idx = 0; idx < dataSize; ++idx)
        {
            m_dataProfile.addDataPoint(request.times[idx], request.values[idx]);
        }
        m_dataProfile.setStartTime(ros::Time::now().toSec());
        response.result = "[Temperature Mock] Profile accepted";
    }

    return true;
}

} //END namespace
