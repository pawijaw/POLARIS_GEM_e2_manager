/**
  * @file RobotStateMachine.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Robot state machine
*/

#include "RobotStateMachine.h"
#include "RobotManagerDefs.h"

namespace RobotManager {

RobotStateMachine::RobotStateMachine():
    m_node(),
    m_robotState(RobotState::IDLE),
    m_sensorDataManager()
{
    //empty
}

void RobotStateMachine::changeState(RobotState newState, const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    switch (m_robotState)
    {
    case RobotState::RUNNING:
        exitRunningState(reason, currentTime, logs);
        break;
    case RobotState::IDLE:
        exitIdleState(reason, currentTime, logs);
        break;
    case RobotState::ERROR:
        exitErrorState(reason, currentTime, logs);
        break;
    default:
        break;
    }

    bool stateEnterOk = false;

    switch (newState)
    {

    case RobotState::RUNNING:
        stateEnterOk = enterRunningState(reason, currentTime, logs);
        break;

    case RobotState::IDLE:
        enterIdleState(reason, currentTime, logs);
        stateEnterOk = true; //Entering IDLE state is always sucessful
        break;

    case RobotState::ERROR:
        enterErrorState(reason, currentTime, logs);
        stateEnterOk = true; //Entering ERROR state is always sucessful
        break;

    default:
        break;
    }

    if (stateEnterOk)
    {
        m_robotState = newState;
    }
    else
    {
        m_robotState = RobotState::ERROR;
        ROS_ERROR("State change failed, going to ERROR state!");
    }
}

void RobotStateMachine::processErrorState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    const auto sensorStatus = m_sensorDataManager.getSensorStatus(vehData, currentTime, logs);

    if(sensorStatus.first)
    {
        if(vehData.activeNavService.size() > 0) // check for condition for transitioning to RUNNING state
        {
            changeState(RobotState::RUNNING, "All healthy now - returning to RUNNING state", currentTime, logs);
        }
        else
        {
            changeState(RobotState::IDLE, "All healthy now - returning to IDLE state", currentTime, logs);
        }
    }
}

void RobotStateMachine::enterErrorState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    logs.emplace_back(currentTime, "Entering ERROR state: " + reason);
}

void RobotStateMachine::exitErrorState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    //logs.emplace_back(currentTime, "Exiting ERROR state: " + reason);
}

void RobotStateMachine::processIdleState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    const auto sensorStatus = m_sensorDataManager.getSensorStatus(vehData, currentTime, logs);
    if(!sensorStatus.first) // check for condition for transitioning to ERROR state
    {
        changeState(RobotState::ERROR, sensorStatus.second, currentTime, logs);
    }
    else if(vehData.activeNavService.size() > 0) // check for condition for transitioning to RUNNING state
    {
        changeState(RobotState::RUNNING, "NAV Service activated", currentTime, logs);
    }
}

void RobotStateMachine::enterIdleState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    logs.emplace_back(currentTime, "Enter IDLE state: " + reason);
}

void RobotStateMachine::exitIdleState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{

}

void RobotStateMachine::processRunningState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    const auto sensorStatus = m_sensorDataManager.getSensorStatus(vehData, currentTime, logs);
    if(!sensorStatus.first) // check for condition for transitioning to ERROR state
    {
        changeState(RobotState::ERROR, sensorStatus.second, currentTime, logs);
    }
    else if(vehData.activeNavService.size() == 0) // check for condition for transitioning to IDLE state
    {
        changeState(RobotState::IDLE, "NAV Service deactivated", currentTime, logs);
    }
}

bool RobotStateMachine::enterRunningState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    logs.emplace_back(currentTime, "Enter RUNNING state: " + reason);
    return true;
}
void RobotStateMachine::exitRunningState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs)
{

}

std::pair<RobotState, VehicleData::LogsWithTime_t> RobotStateMachine::processStateMachine(const VehicleData& vehData, double currentTime)
{
    VehicleData::LogsWithTime_t logs;

    switch (m_robotState)
    {
    case RobotState::RUNNING:
        processRunningState(vehData, currentTime, logs);
        break;

    case RobotState::IDLE:
        processIdleState(vehData, currentTime, logs);
        break;

    case RobotState::ERROR:
    default:
        processErrorState(vehData, currentTime, logs);
        break;
    }
    return std::make_pair(m_robotState, logs);
}

} //END namespace
