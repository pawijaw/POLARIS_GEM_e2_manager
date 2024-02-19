/**
  * @file V2xModel.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: V2x Sensor Model - simple model publishing signal quality
*/

#include "V2xModel.h"
#include "V2xMockStrings.h"

#include <v2x_mock/V2xMsg.h>

#include <functional>

namespace RobotManager::SensorMock {

V2xModel::V2xModel(int initialStrength):
    m_nodeHandle(),
    m_cmdService(),
    m_publisher(),
    m_dataProfile(initialStrength)
{
    m_cmdService = m_nodeHandle.advertiseService(
                V2XMOCK_SERVICE,
                &V2xModel::setProfile,
                this);

    m_publisher = m_nodeHandle.advertise<v2x_mock::V2xMsg>(V2X_TOPIC, 1);
}

void V2xModel::publishData(const ros::Time& currentTime)
{
    const auto currentTimeSeconds = currentTime.toSec();

    v2x_mock::V2xMsg outMsg;
    outMsg.timestamp = currentTime;

    outMsg.signalStrength = m_dataProfile.getValue(currentTime.toSec());

 //   ROS_INFO("Publishing V2X signal strenght: %d", outMsg.signalStrength);

    /**
    *  Data acquisition from a real sensor might fail. This is to indicate if the data contained in the message is valid.
    *  Useful for future modelling of advanced error states (e.g. sensor failure)
    */
    outMsg.dataOk = true;

    m_publisher.publish(outMsg);
}

bool V2xModel::setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response)
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
        response.result = "[V2X Mock] Profile accepted";
    }

    return true;
}

} //END namespace
