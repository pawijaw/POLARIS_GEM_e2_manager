#ifndef SENSORDATAMANAGER_H
#define SENSORDATAMANAGER_H


#include "ros/ros.h"

#include "VehicleData.h"
#include "SensorThreshold.h"

namespace RobotManager {

class SensorDataManager
{
public:
    SensorDataManager();

    /**
     * @brief getSensorStatus - checks if all sensors are heathy and are reporting velues within the predefined ranges
     * @param vehData - current vehicle data
     * @param currentTime - current time [seconds]
     * @param logs
     * @return std::pair< bool - true if all sensors are healthy, and an associated message
     */
    std::pair<bool, std::string> getSensorStatus(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs);

private:
    inline std::pair<bool, std::string> produceError(const std::string& errorStr, double errorTime);

private:
    using BatteryThreshold_t = SensorThreshold<double>;
    using TemperatureThreshold_t = SensorThreshold<double>;
    using GpsThreshold_t = SensorThreshold<double>;
    using SignalThreshold_t = SensorThreshold<unsigned int>;

    ros::NodeHandle m_node;

    BatteryThreshold_t m_batteryThreshold;
    TemperatureThreshold_t m_temperatureThreshold;
    GpsThreshold_t m_gpsThreshold;
    SignalThreshold_t m_signalThresholdLow;
    SignalThreshold_t m_signalThresholdLoss;
};

} //END namespace

#endif // SENSORDATAMANAGER_H
