/**
  * @file GpsModel.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Battery Sensor Model - simple model publishing state of charge (SoC)
*/

#include "GpsModel.h"
#include "GpsMockStrings.h"

#include <gps_mock/GpsMsg.h>


#include <functional>

namespace RobotManager::SensorMock {

GpsModel::GpsModel(double initialAccuracy):
    m_nodeHandle(),
    m_cmdService(),
    m_publisher(),
    m_gazeboClient(m_nodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state")),
    m_dataProfile(initialAccuracy)
{ 
    m_gazeboReqMsg.request.model_name = "gem";

    m_cmdService = m_nodeHandle.advertiseService(
                GPSMOCK_SERVICE,
                &GpsModel::setProfile,
                this);

    m_publisher = m_nodeHandle.advertise<gps_mock::GpsMsg>(GPSMOCK_TOPIC, 1);
}

void GpsModel::publishData(const ros::Time& currentTime)
{
    gps_mock::GpsMsg outMsg;

    m_gazeboClient.call(m_gazeboReqMsg);

    outMsg.x = m_gazeboReqMsg.response.pose.position.x;
    outMsg.y = m_gazeboReqMsg.response.pose.position.y;

    const auto currentTimeSeconds = currentTime.toSec();

    outMsg.horizontalAccuracy = m_dataProfile.getValue(currentTime.toSec());
    outMsg.timestamp = currentTime;

  //  ROS_INFO("Publishing GPS x=%f y=%f accuracy=%f mm", outMsg.x, outMsg.y, outMsg.horizontalAccuracy);

    /**
    *  Data acquisition from a real sensor might fail. This is to indicate if the data contained in the message is valid.
    *  Useful for future modelling of advanced error states (e.g. sensor failure)
    */
    outMsg.dataOk = true;

    m_publisher.publish(outMsg);
}

bool GpsModel::setProfile(sensor_mock_lib::SensorMockSrv::Request& request, sensor_mock_lib::SensorMockSrv::Response& response)
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
        response.result = "[GPS Mock] Profile accepted";
    }

    return true;
}

} //END namespace
