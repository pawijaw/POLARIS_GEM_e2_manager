/**
  * @file EstopModel.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: Emergency Stop Sensor Model - simple model publishing E-Stop button status
*/

#include "EstopModel.h"
#include "EstopMockStrings.h"

#include <estop_mock/EstopMsg.h>

#include <functional>

namespace RobotManager::SensorMock {

EstopModel::EstopModel(int initialValue):
    m_nodeHandle(),
    m_cmdService(),
    m_publisher(),
    m_dataProfile(initialValue)
{
    m_cmdService = m_nodeHandle.advertiseService(
                ESTOPMOCK_SERVICE,
                &EstopModel::setProfile,
                this);

    m_publisher = m_nodeHandle.advertise<estop_mock::EstopMsg>(ESTOP_TOPIC, 1);
}

void EstopModel::publishData(const ros::Time& currentTime)
{
    const auto currentTimeSeconds = currentTime.toSec();

    estop_mock::EstopMsg outMsg;

    outMsg.estopTriggered = m_dataProfile.getValue(currentTime.toSec());
    outMsg.timestamp = currentTime;

   // ROS_INFO("Publishing E-Stop status: %d", outMsg.estopTriggered);

    /**
    *  Data acquisition from a real sensor might fail. This is to indicate if the data contained in the message is valid.
    *  Useful for future modelling of advanced error states (e.g. sensor failure)
    */
    outMsg.dataOk = true;

    m_publisher.publish(outMsg);
}

bool EstopModel::setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response)
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
        response.result = "[E-Stop Mock] Profile accepted";
    }

    return true;
}

} //END namespace
