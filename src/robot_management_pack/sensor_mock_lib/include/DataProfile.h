/**
  * @file DataProfile.h
  * @author Pawel Jaworski
  * @version 1.0
  * @date 16/02/2024
  * @brief: Data profile utility
*/

#ifndef RampProfile_H
#define RampProfile_H

#include <vector>

namespace RobotManager::Utils {

class DataProfile
{
        using Data_t = double; // TODO: Template Class?
public:
    struct DataPoint
    {
        DataPoint(double time = 0.0, Data_t value = 0): time(time), value(value){}
        double time;
        Data_t value;
    };

public:
    DataProfile(Data_t defaultValue = 0.0);

    void reset();

    void addDataPoint(double timeSeconds, Data_t value);
    void setStartTime(double currentTimeSeconds);

    Data_t getValue(double currentTimeSeconds);

private:

    std::vector<DataPoint> m_data;
    std::vector<DataPoint>::iterator m_dataIterator;

    double m_startTime;

};

} //END namespace
#endif // RampProfile_H
