#include "SensorDataManager.h"
#include "RobotManagerDefs.h"

namespace RobotManager {

SensorDataManager::SensorDataManager():
    m_node()
{
    // Configure battery threshold
    double minBatterySoC; // SoC percentage
    m_node.param(PARAM_BATTERY_SENSOR_MIN_SOC, minBatterySoC, DEFAULT_BATTERY_SENSOR_MIN_SOC);
    m_batteryThreshold.set(minBatterySoC, BatteryThreshold_t::Comp::GREATER, 0.0);

    // Configure temperature threshold
    double maxTemp; //degrees C
    m_node.param(PARAM_TEMP_MAX, maxTemp, DEFAULT_TEMPERATURE_MAX);
    m_temperatureThreshold.set(maxTemp, TemperatureThreshold_t::Comp::SMALLER, 0.0);

    // Configure GNSS(GPS) threshold
    double maxGpsError, gpsTimeout;
    m_node.param(PARAM_GPS_ERROR_MAX, maxGpsError, DEFAULT_GPS_ACCURACY_THRESHOLD);
    m_node.param(PARAM_GPS_ACCURACY_TIMEOUT, gpsTimeout, DEFAULT_GPS_ERROR_TIMEOUT);
    m_gpsThreshold.set(maxGpsError, GpsThreshold_t::Comp::SMALLER, gpsTimeout);

    // Configure V2x signal thresholds

    double weakSignalTimeout, lossSignalTimeout;
    m_node.param(PARAM_SIGNAL_TIMEOUT_WEAK, weakSignalTimeout, DEFAULT_V2X_SIGNAL_TIMEOUT_WEAK);
    m_node.param(PARAM_SIGNAL_TIMEOUT_LOSS, lossSignalTimeout, DEFAULT_V2X_SIGNAL_TIMEOUT_LOSS);
    m_signalThresholdLow.set(1, SignalThreshold_t::Comp::GREATER, weakSignalTimeout);
    m_signalThresholdLoss.set(0, SignalThreshold_t::Comp::GREATER, lossSignalTimeout);
}

std::pair<bool, std::string> SensorDataManager::getSensorStatus(const VehicleData& vehData, double currentTime, VehicleData::LogsWithTime_t& logs)
{
    // Battery sensor
    if(vehData.batteryData.receiveTimestamp + TIMEOUT_BATTERY_SENSOR < currentTime)
    {
        return std::make_pair(false, "No data from battery sensor");
    }
    if(!m_batteryThreshold.isInRange(vehData.batteryData.batteryMsg.stateOfCharge, currentTime))
    {
        return produceError("Battery level too low", vehData.batteryData.batteryMsg.timestamp.toSec());
    }

    //Temperature sensor
    if(vehData.temperatureData.receiveTimestamp + TIMEOUT_TEMPERATURE_SENSOR < currentTime)
    {
        return std::make_pair(false, "No data from temperature sensor");
    }
    if(!m_temperatureThreshold.isInRange(vehData.temperatureData.temperatureMsg.temperature, currentTime))
    {
        return produceError("Temperature too high", vehData.temperatureData.temperatureMsg.timestamp.toSec());
    }

    //GPS sensor
    if(vehData.gpsData.receiveTimestamp + TIMEOUT_GPS_SENSOR < currentTime)
    {
        return std::make_pair(false, "No data from GPS sensor");
    }
    if(!m_gpsThreshold.isInRange(vehData.gpsData.gpsMsg.horizontalAccuracy, currentTime))
    {
        return produceError("GPS accuracy too low", vehData.gpsData.gpsMsg.timestamp.toSec());
    }

    //V2x sensor
    if(vehData.v2xData.receiveTimestamp + TIMEOUT_V2X_SENSOR < currentTime)
    {
        return std::make_pair(false, "No data from V2x sensor");
    }
    if(!m_signalThresholdLow.isInRange(vehData.v2xData.v2xMsg.signalStrength, currentTime))
    {
        return produceError("Weak V2x signal for too long", vehData.v2xData.v2xMsg.timestamp.toSec());
    }
    if(!m_signalThresholdLoss.isInRange(vehData.v2xData.v2xMsg.signalStrength, currentTime))
    {
        return produceError("Loss of V2x signal for too long", vehData.v2xData.v2xMsg.timestamp.toSec());
    }

    //E-Stop sensor
    if(vehData.estopData.receiveTimestamp + TIMEOUT_ESTOP_SENSOR < currentTime)
    {
        return std::make_pair(false, "No data from E-Stop sensor");
    }
    if(vehData.estopData.estopMsg.estopTriggered)
    {
        return produceError("E-Stop triggered", vehData.estopData.estopMsg.timestamp.toSec());
    }

    return std::make_pair(true, "Sensors healthy");
}

inline std::pair<bool, std::string> SensorDataManager::produceError(const std::string& errorStr, double errorTime)
{
    std::stringstream errStream;
    errStream << errorStr << " - Msg time: " << std::setprecision(5) << std::fixed << errorTime;
    //ROS_ERROR("%s", errStream.str().c_str());
    return std::make_pair(false, errStream.str());
}

} //END namespace
