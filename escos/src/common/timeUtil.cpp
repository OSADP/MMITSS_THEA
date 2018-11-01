#include "timeUtil.hpp"

#include <chrono>
#include <sstream>
#include <string>

namespace WaveApp
{

std::string timeUtil::getCurrentTimeStampInMS()
{
    return std::to_string(
        std::chrono::duration_cast<mSecs>(Clock::now().time_since_epoch()).count());
}

uint32_t timeUtil::toMinuteOfTheYear(const TimePoint& tp)
{
    std::time_t now = Clock::to_time_t(tp);
    return toMinuteOfTheYear(now);
}

uint32_t timeUtil::toMinuteOfTheYear(std::time_t unix_timestamp)
{
    struct tm newyear;
    if (localtime_r(&unix_timestamp, &newyear) == nullptr)
        return 527040;
    newyear.tm_hour = 0;
    newyear.tm_mon = 0;
    newyear.tm_min = 0;
    newyear.tm_sec = 0;
    newyear.tm_mday = 1;
    return difftime(unix_timestamp, mktime(&newyear)) / 60;
}

uint64_t timeUtil::fromMinuteOfTheYear(uint32_t minuteOfTheYear)
{
    struct tm newyear;
    std::time_t now = time(nullptr);

    localtime_r(&now, &newyear);

    // If the minuteOfThe year is more than 2 months in the future, it is probably in the past year
    if (int64_t(minuteOfTheYear) > ((int64_t(newyear.tm_mon + 2)) * 30 * 24 * 60))
        newyear.tm_year = newyear.tm_year - 1;
    // If the minuteOfThe year is more than 10 months in the past, it is probably in the next year
    if (int64_t(minuteOfTheYear) < ((int64_t(newyear.tm_mon - 10)) * 30 * 24 * 60))
        newyear.tm_year = newyear.tm_year + 1;

    // newyear.tm_year = 2018;
    newyear.tm_hour = 0;
    newyear.tm_mon = 0;
    newyear.tm_min = 0;
    newyear.tm_sec = 0;
    newyear.tm_mday = 1;

    return mktime(&newyear) + (minuteOfTheYear * 60);
}

uint32_t timeUtil::toDSecond(const TimePoint& tp)
{
    auto now = Clock::to_time_t(tp);
    struct tm newyear;
    if (localtime_r(&now, &newyear) == nullptr)
        return 65535;
    return newyear.tm_sec * 1000 +
           static_cast<uint32_t>(
               std::chrono::duration_cast<mSecs>(tp.time_since_epoch()).count() % 1000);
}

} // namespace WaveApp
