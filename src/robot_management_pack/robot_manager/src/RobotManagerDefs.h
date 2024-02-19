#ifndef RobotManagerDefs_H
#define RobotManagerDefs_H


namespace RobotManager {

constexpr auto PARAM_BATTERY_SENSOR_MIN_SOC = "battery_minsoc";
constexpr auto PARAM_TEMP_MAX = "temp_max";
constexpr auto PARAM_GPS_ERROR_MAX = "gps_error";
constexpr auto PARAM_GPS_ACCURACY_TIMEOUT = "gps_timeout";
constexpr auto PARAM_SIGNAL_TIMEOUT_LOSS = "signal_loss_timeout";
constexpr auto PARAM_SIGNAL_TIMEOUT_WEAK = "signal_weak_timeout";

// default values (set from the assignment description
constexpr auto DEFAULT_BATTERY_SENSOR_MIN_SOC = 50.0;
constexpr auto DEFAULT_TEMPERATURE_MAX = 55.0;
constexpr auto DEFAULT_GPS_ERROR_TIMEOUT = 15.0;
constexpr auto DEFAULT_GPS_ACCURACY_THRESHOLD = 200.0;
constexpr auto DEFAULT_V2X_SIGNAL_TIMEOUT_LOSS = 10.0;
constexpr auto DEFAULT_V2X_SIGNAL_TIMEOUT_WEAK = 20.0;

constexpr auto SERVICE_NAV_PURE_PURSUIT = "pp_waypoint_request";
constexpr auto SERVICE_NAV_STANLEY = "stanley_waypoint_request";

constexpr auto TIMEOUT_BATTERY_SENSOR = 1.0;
constexpr auto TIMEOUT_TEMPERATURE_SENSOR = 1.0;
constexpr auto TIMEOUT_GPS_SENSOR = 1.0;
constexpr auto TIMEOUT_V2X_SENSOR = 1.0;
constexpr auto TIMEOUT_ESTOP_SENSOR = 1.0;

constexpr auto SIGNAL_V2X_GOOD = 2.0;
constexpr auto SIGNAL_V2X_WEAK = 1.0;
constexpr auto SIGNAL_V2X_LOSS = 0.0;

} // END namespace

#endif // RobotManagerDefs_H
