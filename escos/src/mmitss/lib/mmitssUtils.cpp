#include "mmitssUtils.h"

#include <time.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace WaveApp
{
namespace Mmitss
{

void xTimeStamp(char* pc_TimeStamp_)
{
    struct tm* ps_Time;
    time_t i_CurrentTime;
    char ac_TmpStr[256];

    i_CurrentTime = time(NULL);
    ps_Time = localtime(&i_CurrentTime);

    // year
    sprintf(ac_TmpStr, "%d", ps_Time->tm_year + 1900);
    strcpy(pc_TimeStamp_, ac_TmpStr);

    // month
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_mon + 1);
    strcat(pc_TimeStamp_, ac_TmpStr);

    // day
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_mday);
    strcat(pc_TimeStamp_, ac_TmpStr);

    // hour
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_hour);
    strcat(pc_TimeStamp_, ac_TmpStr);

    // min
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_min);
    strcat(pc_TimeStamp_, ac_TmpStr);

    // sec
    sprintf(ac_TmpStr, "_%d", ps_Time->tm_sec);
    strcat(pc_TimeStamp_, ac_TmpStr);
}

// Minute must be minutes since the beginning of the current year
// ignore the special situation of cross-year
int calculateETA(int beginMin, int beginSec, int endMin, int endSec)
{
    return ((endMin * 60 + endSec) - (beginMin * 60 + beginSec));
}

double distbeforeSqrt(
    double secondx, double secondy, double firstx, double firsty, double px, double py)
{
    double nx = secondx - firstx, ny = secondy - firsty;
    double pax = firstx - px, pay = firsty - py;
    double c = nx * pax + ny * pay;
    // Closest point is first
    if (c > 0.0)
        return pax * pax + pay * pay;

    double bpx = px - secondx, bpy = py - secondy;
    double d = nx * bpx + ny * bpy;
    // Closest point is second
    if (d > 0.0)
        return bpx * bpx + bpy * bpy;

    // Closest point is between first and second
    double ex = pax - nx * (c / (nx * nx + ny * ny)), ey = pay - ny * (c / (nx * nx + ny * ny));
    return ex * ex + ey * ey;
}

// string must have correct length if contains 0x00 within,
// for example string of"0x40,0x00,0x01,0x23" must have length 4 when get created
std::string forDisplayInHexFormat(const std::string& orgStr)
{
    std::ostringstream ss;
    ss << std::hex;
    ss.fill('0');
    auto buffer = reinterpret_cast<const uint8_t*>(orgStr.data());
    for (unsigned int i = 0; i < orgStr.length(); i++)
    {
        ss << std::setw(2) << std::right << unsigned(buffer[i]);
    }
    return ss.str();
}

} // namespace Mmitss
} // namespace WaveApp
