#pragma once

#include <chrono>
#include <string>
#include <ctime>

namespace WaveApp
{

using Clock = std::chrono::high_resolution_clock;
using mSecs = std::chrono::milliseconds;
using TimePoint = std::chrono::high_resolution_clock::time_point;

class timeUtil
{
  public:
    /**
     * get current time using chrono as milliseconds from epoch
     * @return string represents current time
     */
    static std::string getCurrentTimeStampInMS();

    /*
     * get minute of the year according to SaAE J2735 definition:
     *
     * The DE_MinuteOfTheYear data element expresses the number of
     * elapsed minutes of the current year in the time system being
     * used (typically UTC time). It is typically used to provide a
     * longer range time stamp indicating when a message was created.
     * Taken together with the DSecond data element, it provides a
     * range of one full year with a resolution of 1mSecond.
     *
     * @return MinuteOfTheYear ::= INTEGER (0..527040), 527040 shall be used for invalid
     */
    static uint32_t toMinuteOfTheYear(const TimePoint& tp);

    /*
     * get minute of the year according to SaAE J2735 definition:
     *
     * The DE_MinuteOfTheYear data element expresses the number of
     * elapsed minutes of the current year in the time system being
     * used (typically UTC time). It is typically used to provide a
     * longer range time stamp indicating when a message was created.
     * Taken together with the DSecond data element, it provides a
     * range of one full year with a resolution of 1mSecond.
     *
     * @return MinuteOfTheYear ::= INTEGER (0..527040), 527040 shall be used for invalid
     */
    static uint32_t toMinuteOfTheYear(std::time_t unix_timestamp);

    /*
     * Convert DE_MinuteOfTheYear (SaAE J2735) to unix timestamp
     *
     * @return unix timestamp [seconds]
     */
    static uint64_t fromMinuteOfTheYear(uint32_t minuteOfTheYear);

    /*
     * get DSecond according to SaAE J2735 definition:
     *
     * The DSRC second expressed in this data element consists of
     * integer values from zero to 60999, representing the milliseconds
     * within a minute. A leap second is represented by the value range
     * 60000 to 60999. The value of 65535 shall represent an unavailable
     * value in the range of the minute. The values from 61000 to 65534
     * are reserved.
     *
     * @return DSecond ::= INTEGER (0..65535) -- units of milliseconds
     */
    static uint32_t toDSecond(const TimePoint& tp);
};

} // namespace WaveApp
