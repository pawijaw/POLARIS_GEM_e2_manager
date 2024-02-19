

#include "ros/ros.h"
#include <gtest/gtest.h>
#include "SensorThreshold.h"
#include "RobotStateMachine.h"
#include "RobotManagerDefs.h"
#include <iostream>

//----------- Tests for SensorThreshold ------------------------------

TEST(SensorThreshold, SensorThreshold_double_notime)
{
    using SensorThreshold_double = RobotManager::SensorThreshold<double>;

    double threshold = 50.0;
    SensorThreshold_double thr(threshold, SensorThreshold_double::Comp::GREATER, 0.0);

    double testVal = 0.0;
    for(double testVal = 100.0; testVal >= 0.0; testVal -= 0.1)
    {
        auto result = thr.isInRange(testVal, 0.0);

        if(testVal > threshold)
        {
            EXPECT_TRUE(result);
        }
        else
        {
            EXPECT_FALSE(result);
        }
    }
}

TEST(SensorThreshold, SensorThreshold_double_timed)
{
    using SensorThreshold_double = RobotManager::SensorThreshold<double>;

    constexpr auto TEST_THRESHOLD = 50.0;

    SensorThreshold_double thr(TEST_THRESHOLD, SensorThreshold_double::Comp::GREATER, 20.0);

    double testVal = 0.0;
    for(double testTime = 0.0; testTime <= 40.0; testTime += 1)
    {
        double testVal = testTime < 10.0 ? TEST_THRESHOLD + 1.0 : TEST_THRESHOLD -1.0;

        //0-10s is in range
        //at 10s exits range, timer starts
        //at 30s (20+10), timer expires and threshold reports breach
        auto result = thr.isInRange(testVal, testTime);

        if(testTime < 30.0)
        {
            EXPECT_TRUE(result);
        }
        else
        {
            EXPECT_FALSE(result);
        }
    }
}

TEST(SensorThreshold, SensorThreshold_uint_notimeout)
{
    using SensorThreshold_uint= RobotManager::SensorThreshold<unsigned int>;

    constexpr auto TEST_THRESHOLD_TIMEOUT = 20.0;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = TEST_THRESHOLD_TIMEOUT - 1.0; //event is 1 second shorter than the timeout

    constexpr auto TEST_THRESHOLD = 1;

    SensorThreshold_uint thr(TEST_THRESHOLD, SensorThreshold_uint::Comp::GREATER, 20.0);

    unsigned int testVal = 0;
    for(double testTime = 0.0; testTime <= 40.0; testTime += 1.0)
    {
        unsigned int testVal = (testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION) ? TEST_THRESHOLD : TEST_THRESHOLD + 1U;

        auto result = thr.isInRange(testVal, testTime);

        EXPECT_TRUE(result);
    }
}

TEST(SensorThreshold, SensorThreshold_uint_timeout)
{
    using SensorThreshold_uint= RobotManager::SensorThreshold<unsigned int>;

    constexpr auto TEST_THRESHOLD_TIMEOUT = 20.0;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = TEST_THRESHOLD_TIMEOUT + 5.0; //event is 1 second shorter than the timeout

    constexpr auto TEST_THRESHOLD = 1;

    SensorThreshold_uint thr(TEST_THRESHOLD, SensorThreshold_uint::Comp::GREATER, TEST_THRESHOLD_TIMEOUT);

    unsigned int testVal = 0;
    for(double testTime = 0.0; testTime <= 40.0; testTime += 1.0)
    {
        unsigned int testVal = (testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION) ? TEST_THRESHOLD : TEST_THRESHOLD+1U;

        auto result = thr.isInRange(testVal, testTime);

        // std::cout << "t=" << testTime << " testval=" << testVal <<" res=" << result << std::endl;

        if(testTime > TEST_EVENT_START + TEST_THRESHOLD_TIMEOUT && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            EXPECT_FALSE(result);
        }
        else
        {
            EXPECT_TRUE(result);
        }
    }
}


//----------- Inline utility fuctions used in subsequent tests ------------------------------

inline void updateVehDataTimestamps(RobotManager::VehicleData& vehData, double currentTime)
{
    vehData.batteryData.timestamp = currentTime;
    vehData.temperatureData.timestamp = currentTime;
    vehData.gpsData.timestamp = currentTime;
    vehData.v2xData.timestamp = currentTime;
    vehData.estopData.timestamp = currentTime;
}

inline void initVehData(RobotManager::VehicleData& vehData)
{
    using namespace RobotManager;

    vehData.batteryData.batteryMsg.stateOfCharge = DEFAULT_BATTERY_SENSOR_MIN_SOC + 1.0;
    vehData.temperatureData.temperatureMsg.temperature = DEFAULT_TEMPERATURE_MAX - 1.0;
    vehData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD - 1.0;
    vehData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_GOOD;
    vehData.estopData.estopMsg.estopTriggered = false;
}

//inline std::string robotStateToString(RobotManager::RobotState state)
//{
//    switch(state)
//    {
//    case RobotManager::RobotState::ERROR:
//        return "ERROR";
//    case RobotManager::RobotState::IDLE:
//        return "IDLE";
//    case RobotManager::RobotState::RUNNING:
//        return "RUNNING";
//    default:
//        break;
//    }
//    return "UNKNOWN";
//}

//----------- Sensor timeous test (complete loss of comms with the sensor) ------------------------------

/**
 * @brief Test if loss of communication with battery sensor is detected
 */
TEST(StateMachine_timeouts, BatterySensorTimeout)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);

    for(double testTime = 0.0; testTime <= 30.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            vehData.batteryData.timestamp = testTime - TIMEOUT_BATTERY_SENSOR - 0.1;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == RobotState::IDLE);
        }
    }
}

/**
 * @brief Test if loss of communication with GPS sensor is detected
 */
TEST(StateMachine_timeouts, GpsSensorTimeout)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);

    for(double testTime = 0.0; testTime <= 30.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            vehData.gpsData.timestamp = testTime - TIMEOUT_GPS_SENSOR - 0.1;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == RobotState::IDLE);
        }
    }
}

/**
 * @brief Test if loss of communication with temperature sensor is detected
 */
TEST(StateMachine_timeouts, TemperatureSensorTimeout)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);

    for(double testTime = 0.0; testTime <= 30.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            vehData.temperatureData.timestamp = testTime - TIMEOUT_TEMPERATURE_SENSOR - 0.1;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == RobotState::IDLE);
        }
    }
}

/**
 * @brief Test if loss of communication with V2X (~Internet) sensor is detected
 */
TEST(StateMachine_timeouts, V2xSensorTimeout)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);

    for(double testTime = 0.0; testTime <= 30.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            vehData.v2xData.timestamp = testTime - TIMEOUT_V2X_SENSOR - 0.1;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == RobotState::IDLE);
        }
    }
}

/**
 * @brief Test if loss of communication with E-Stop sensor is detected
 */
TEST(StateMachine_timeouts, EstopSensorTimeout)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);

    for(double testTime = 0.0; testTime <= 30.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            vehData.estopData.timestamp = testTime - TIMEOUT_ESTOP_SENSOR - 0.1;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > 10.0 && testTime < 20.0)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == RobotState::IDLE);
        }
    }
}

//----------- State machine logic tests (acting on data received from sensors) ------------------------------

/**
 * @brief Test if low battery value is detected
 */
void batteryTest(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= 120.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        //vary battery levels form 100 to 40 and back to 100
        if(testTime < 60.0)
        {
            vehData.batteryData.batteryMsg.stateOfCharge = 100.0 - testTime; // Ramp down 100% to 40%
        }
        else
        {
            vehData.batteryData.batteryMsg.stateOfCharge = testTime - 20.0; // Ramp up 40% to 100%
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(vehData.batteryData.batteryMsg.stateOfCharge <= DEFAULT_BATTERY_SENSOR_MIN_SOC)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if low battery value is detected
 * From IDLE state
 */
TEST(StateMachine_logic, BatteryLevelTestIdle)
{
    batteryTest(RobotManager::RobotState::IDLE);
}

/**
 * @brief Test if low battery value is detected
 * From IDLE state
 */
TEST(StateMachine_logic, BatteryLevelTestRunninng)
{
    batteryTest(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if low battery value is detected
 */
void temperatureTest(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= 120.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        //vary temperature from 0 to 60 and back to 0
        if(testTime < 60.0)
        {
            vehData.temperatureData.temperatureMsg.temperature = testTime; //Ramp up from 0 to 60
        }
        else
        {
            vehData.temperatureData.temperatureMsg.temperature = 120.0 - testTime; //Ramp down from 60 to 0
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(vehData.temperatureData.temperatureMsg.temperature >= DEFAULT_TEMPERATURE_MAX)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if high temerature value is detected
 * From IDLE state
 */
TEST(StateMachine_logic, TemperatureLevelTestIdle)
{
    temperatureTest(RobotManager::RobotState::IDLE);
}

/**
 * @brief Test if high temerature value is detected
 * From RUNNING state
 */
TEST(StateMachine_logic, TemperatureLevelTestRunning)
{
    temperatureTest(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if breaching the GPS accuracy threshold for short time does not cause ERROR states
 */
void gpsTestWithinTolerance(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = DEFAULT_GPS_ERROR_TIMEOUT - 1.0; //event is 1 second shorter than the timeout

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime < TEST_EVENT_START + TEST_EVENT_DURATION + 20.0; testTime += 0.1)
    {
        updateVehDataTimestamps(vehData, testTime);

        // drop accuracy below threshold for less than 15s twice
        if(TEST_EVENT_START > 10.0 && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            vehData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD + 1.0;
        }
        else
        {
            vehData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD - 1.0;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);
        EXPECT_TRUE(result.first == nonerrorState);
    }
}

/**
 * @brief Test if breaching the accuracy threshold for short time does not cause ERROR states
 * Starting from IDLE
 */
TEST(StateMachine_logic, GpsLevelTestIdleWithinTolerance)
{
    gpsTestWithinTolerance(RobotManager::RobotState::IDLE);
}


/**
 * @brief Test if breaching the accuracy threshold for short time does not cause ERROR states
 * Starting from RUNNING
 */
TEST(StateMachine_logic, GpsLevelTestRunningWithinTolerance)
{
     gpsTestWithinTolerance(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if breaching the GPS accuracy threshold triggers error
 */
void gpsTestOutsideTolerance(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = DEFAULT_GPS_ERROR_TIMEOUT + 10.0; //event lasts 10s longer than the timeout

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= TEST_EVENT_START + TEST_EVENT_DURATION + 20.0; testTime += 0.5)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            vehData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD + 1.0;
        }
        else
        {
            vehData.gpsData.gpsMsg.horizontalAccuracy = DEFAULT_GPS_ACCURACY_THRESHOLD - 1.0;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > TEST_EVENT_START + DEFAULT_GPS_ERROR_TIMEOUT && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if breaching the GPS accuracy threshold triggers error
 * Starting from IDLE
 */
TEST(StateMachine_logic, GpsLevelIdleToError)
{
    gpsTestOutsideTolerance(RobotManager::RobotState::IDLE);
}

/**
 * @brief Test if breaching the GPS accuracy threshold triggers error
 * Starting from RUNNING
 */
TEST(StateMachine_logic, GpsLevelRunningToError)
{
    gpsTestOutsideTolerance(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if weak internet signal is detected correctly
 */
void v2xTestWeak(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = DEFAULT_V2X_SIGNAL_TIMEOUT_WEAK + 10.0; //event lasts 10s longer than the timeout

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= TEST_EVENT_START + TEST_EVENT_DURATION + 10.0; testTime += 0.5)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            vehData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_WEAK;
        }
        else
        {
            vehData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_GOOD;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        //std::cout << "t=" << testTime << " val=" << vehData.v2xData.v2xMsg.signalStrength << " state=" << robotStateToString(result.first) << std::endl;

        if(testTime > TEST_EVENT_START + DEFAULT_V2X_SIGNAL_TIMEOUT_WEAK && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if weak internet signal is detected correctly
 * From IDLE state
 */
TEST(StateMachine_logic, V2xTestSignalWeakIdle)
{
    v2xTestWeak(RobotManager::RobotState::IDLE);
}

/**
 * @brief Test if weak internet signal is detected correctly
 * From RUNNING state
 */
TEST(StateMachine_logic, InternetTestSignalWeakRunning)
{
    v2xTestWeak(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if loss of internet signal is detected correctly
 */
void v2xTestLoss(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = DEFAULT_V2X_SIGNAL_TIMEOUT_LOSS + 10.0; //event lasts 10s longer than the timeout

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= TEST_EVENT_START + TEST_EVENT_DURATION + 10.0; testTime += 0.5)
    {
        updateVehDataTimestamps(vehData, testTime);

        if(testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            vehData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_LOSS;
        }
        else
        {
            vehData.v2xData.v2xMsg.signalStrength = SIGNAL_V2X_GOOD;
        }

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        //std::cout << "t=" << testTime << " val=" << vehData.v2xData.v2xMsg.signalStrength << " state=" << robotStateToString(result.first) << std::endl;

        if(testTime > TEST_EVENT_START + DEFAULT_V2X_SIGNAL_TIMEOUT_LOSS && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if loss of internet signal is detected correctly
* From IDLE state
 */
TEST(StateMachine_logic, InternetTestSignalLossIdle)
{
    v2xTestLoss(RobotManager::RobotState::IDLE);
}



/**
 * @brief Test if loss of internet signal is detected correctly
 * From RUNNING state
 */
TEST(StateMachine_logic, InternetTestSignalLossRunning)
{
    v2xTestLoss(RobotManager::RobotState::RUNNING);
}

/**
 * @brief Test if e-Stop is detected correctly
 * From IDLE state
 */
void estopTest(RobotManager::RobotState nonerrorState)
{
    using namespace RobotManager;

    constexpr auto TEST_EVENT_START = 10.0; // trigger the event 10s into the test
    constexpr auto TEST_EVENT_DURATION = 5.0;

    RobotStateMachine testStateMachine;
    VehicleData vehData;
    initVehData(vehData);
    if(nonerrorState == RobotState::RUNNING)
    {
        vehData.activeNavService = "TestNav"; // setting this will cause the state machine to go to RUNNING
    }

    for(double testTime = 0.0; testTime <= TEST_EVENT_START + TEST_EVENT_DURATION + 10.0; testTime += 0.5)
    {
        updateVehDataTimestamps(vehData, testTime);

        vehData.estopData.estopMsg.estopTriggered = testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION;

        auto result = testStateMachine.processStateMachine(vehData, testTime);

        if(testTime > TEST_EVENT_START && testTime < TEST_EVENT_START + TEST_EVENT_DURATION)
        {
            EXPECT_TRUE(result.first == RobotState::ERROR);
        }
        else
        {
            EXPECT_TRUE(result.first == nonerrorState);
        }
    }
}

/**
 * @brief Test if e-Stop is detected correctly
 * From IDLE state
 */
TEST(StateMachine_logic, EstopFromIdle)
{
   estopTest(RobotManager::RobotState::IDLE);
}

/**
 * @brief Test if e-Stop is detected correctly
 * From RUNNING state
 */
TEST(StateMachine_logic, EstopFromRunning)
{
    estopTest(RobotManager::RobotState::RUNNING);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tii_manager_unit_tests");

    ros::NodeHandle n;

    testing::InitGoogleTest();

    return RUN_ALL_TESTS();
}
// %EndTag(FULLTEXT)%
