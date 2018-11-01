#pragma once

#include <string>

namespace WaveApp
{
namespace Mmitss
{

void xTimeStamp(char* pc_TimeStamp_);
int calculateETA(int beginMin, int beginSec, int endMin, int endSec);
double distbeforeSqrt(
    double secondx, double secondy, double firstx, double firsty, double px, double py);
std::string forDisplayInHexFormat(const std::string& orgStr);

} // namespace Mmitss
} // namespace WaveApp
