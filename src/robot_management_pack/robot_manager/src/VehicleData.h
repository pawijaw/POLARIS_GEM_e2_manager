#ifndef VehicleData_H
#define VehicleData_H

#include <battery_mock/BatteryMsg.h>
#include <temperature_mock/TemperatureMsg.h>
#include <gps_mock/GpsMsg.h>
#include <v2x_mock/V2xMsg.h>
#include <estop_mock/EstopMsg.h>

#include <memory>

namespace RobotManager {

enum class RobotState
{
    ERROR,
    IDLE,
    RUNNING
};

struct BatteryData
{
    battery_mock::BatteryMsg batteryMsg;
    double receiveTimestamp;
};

struct TemperatureData
{
    temperature_mock::TemperatureMsg temperatureMsg;
    double receiveTimestamp;
};

struct GpsData
{
    gps_mock::GpsMsg gpsMsg;
    double receiveTimestamp;
};

struct SignalData
{
    gps_mock::GpsMsg gpsMsg;
    double receiveTimestamp;
};

struct V2xData
{
    v2x_mock::V2xMsg v2xMsg;
    double receiveTimestamp;
};

struct EstopData
{
    estop_mock::EstopMsg estopMsg;
    double receiveTimestamp;
};


struct VehicleData
{
    using Ptr_t = std::shared_ptr<VehicleData>;
    using LogWithTime_t = std::pair<double, std::string>;
    using LogsWithTime_t = std::list<LogWithTime_t>;

    BatteryData batteryData;
    TemperatureData temperatureData;
    GpsData gpsData;
    V2xData v2xData;
    EstopData estopData;

    RobotState state = RobotState::IDLE;

    std::string activeNavService;

    LogsWithTime_t logs;
};


} // END namespace

#endif // VehicleData_H
