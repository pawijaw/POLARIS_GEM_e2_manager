/**
  * @file DataProfile.cpp
  * @author Pawel Jaworski
  * @version 1.0
  * @date 15/02/2024
  * @brief: Data profile utility
*/


#include "DataProfile.h"

namespace RobotManager::Utils {


DataProfile::DataProfile(Data_t defaultValue):
    m_data(),
    m_startTime(0.0)
{
    if(defaultValue != 0.0)
    {
        addDataPoint(0.0, defaultValue);
    }
}

void DataProfile::addDataPoint(double timeSeconds, Data_t value)
{
    m_data.emplace_back(timeSeconds, value);
    m_dataIterator = m_data.begin();
}

void DataProfile::setStartTime(double currentTimeSeconds)
{
    m_startTime = currentTimeSeconds;
    m_dataIterator = m_data.begin();
}

void DataProfile::reset()
{
    m_data.clear();
    m_dataIterator = m_data.end();
}

DataProfile::Data_t DataProfile::getValue(double currentTimeSeconds)
{
    if(m_data.size() == 0)
    {
        return 0.0;
    }

    static double lastQueryTime = currentTimeSeconds;
    if(lastQueryTime > currentTimeSeconds)
    {
        m_dataIterator = m_data.begin();
    }
    lastQueryTime = currentTimeSeconds;

    const auto testTimeSeconds = currentTimeSeconds - m_startTime;

    while(m_dataIterator != m_data.end())
    {
        if(m_dataIterator->time >= testTimeSeconds)
        {
            return m_dataIterator->value;
        }
        ++m_dataIterator;
    }

    // iterator is at the end - return last element
    return m_data.back().value;
}

} //END namespace
