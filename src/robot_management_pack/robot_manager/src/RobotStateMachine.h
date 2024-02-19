/**
  * @file RobotStateMachine.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 14/02/2024
  * @brief: Robot state machine
*/

#ifndef ROBOTSTATEMACHINE_H
#define ROBOTSTATEMACHINE_H

#include <string>
#include <list>

#include "SensorDataManager.h"

namespace RobotManager {

class RobotStateMachine
{
public:
    RobotStateMachine();

    /**
     * @brief processStateMachine - execute the state machine on the new VehicleData (containing sensor data)
     * @param vehData - status of the vehicle
     * @param currentTime - current time [s]
     * @return std::pair - RobotState and lists of log messages with times of occurence
     */
    std::pair<RobotState, VehicleData::LogsWithTime_t> processStateMachine(const VehicleData& vehData, double currentTime);

private:
    void changeState(RobotState newState, const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);

    void processErrorState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs);
    void enterErrorState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);
    void exitErrorState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);

    void processIdleState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs);
    void enterIdleState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);
    void exitIdleState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);

    void processRunningState(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs);
    bool enterRunningState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);
    void exitRunningState(const std::string& reason, double currentTime, VehicleData::LogsWithTime_t& logs);

private:
    ros::NodeHandle m_node;
    RobotState m_robotState;

    SensorDataManager m_sensorDataManager;

};

} //END namespace

#endif // ROBOTSTATEMACHINE_H
