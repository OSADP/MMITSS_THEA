/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitssPerfData.hpp
 *
 *  Created on: May 15, 2017
 *      Author: M. Venus
 */

#pragma once

#include "Performance.h"

namespace SieMmitss
{

std::string queueLengthToJSON(const Performance& perf, int numVehicles);

std::string ttAccToJSON(const Performance& perf);

} // namespace SieMmitss
