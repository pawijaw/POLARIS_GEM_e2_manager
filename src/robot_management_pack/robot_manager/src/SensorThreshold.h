#ifndef SENSORTHRESHOLD_H
#define SENSORTHRESHOLD_H

namespace RobotManager {

template <class T>
class SensorThreshold
{
public:
    enum class Comp
    {
        GREATER,
        SMALLER
    };

    SensorThreshold(T boundaryValue, Comp op, double breachTimeSec): m_boundaryValue(boundaryValue), m_op(op), m_breachTimeSec(breachTimeSec){}
    SensorThreshold(): m_boundaryValue(0), m_op(Comp::GREATER), m_breachTimeSec(0.0){}

    /**
     * @brief configures the object
     * @param boundaryValue - the boundary threshold value
     * @param op - comparison operation ot be carried out (Smaller or Greater)
     * @param breachTimeSec - how long after breaching the boundary value should the threshold conidered to be breached
     */
    void set(T boundaryValue, Comp op, double breachTimeSec)
    {
        m_boundaryValue = boundaryValue;
        m_op = op;
        m_breachTimeSec = breachTimeSec;
    }

    /**
     * @brief isInRange - check if the value is in range, or was in range within the threshold time
     * @param value - current vaue of the signal
     * @param currentTimeSec - current time [seconds]
     * @return - true if the signal is considered within the threshold
     */
    bool isInRange(T value, double currentTimeSec)
    {
        auto compare = [this](T val, T ref) -> bool
        {
            return m_op == Comp::GREATER ? val > ref : val < ref;
        };

        if(m_breachTimeSec <= 0.0) // no timeout set
        {
            return compare(value, m_boundaryValue); // no temporal aspect, just check if in range
        }
        else // timeout is set
        {
            if(!compare(value, m_boundaryValue)) //is in breach
            {
                if(conditionBreached) // was in breach before
                {
                    return currentTimeSec < m_timeWhenBreached + m_breachTimeSec;
                }
                else //was not in breach before - entering breach condition
                {
                    m_timeWhenBreached = currentTimeSec;
                    conditionBreached = true;
                    return true;
                }
            }
            else //is not in breach
            {
                conditionBreached = false;
                return true;
            }
        }
    }

private:
    T m_boundaryValue;
    Comp m_op;
    double m_breachTimeSec;

    bool conditionBreached = false;

    double m_timeWhenBreached;

};

//template <class T>
//SensorThreshold<T>::SensorThreshold(T boundaryValue, SensorThreshold::Comp op, Time_t breachTime)
//{

//}


} //END namespace

#endif // SENSORTHRESHOLD_H
