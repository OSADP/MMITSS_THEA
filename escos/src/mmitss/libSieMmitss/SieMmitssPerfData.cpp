/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieBsmConversion.cpp
 *
 *  Created on: May 19, 2017
 *      Author: M.Venus
 */

#include "stdheader.h"

#include "Poco/Dynamic/Struct.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"

#include "SieMmitssPerfData.hpp"

namespace SieMmitss
{

static const int MMITSS_MAX_APPROACHES = 8;
static const int MMITSS_MAX_LANES = 5;
static const int MMITSS_MAX_MOVEMENTS = 12;

static const std::vector<std::string> MovementFromIndex = {
    "NB", "EB", "SB", "WB", "NBLT", "NBRT", "EBLT", "EBRT", "SBLT", "SBRT", "WBLT", "WBRT"};

std::string queueLengthToJSON(const Performance& perf, int numVehicles)
{
    Poco::DynamicStruct out;
    Poco::Dynamic::Array arr;
    for (int app = 1; app < MMITSS_MAX_APPROACHES; app += 2) // MMITSS only processes 1,3,5,7
    {
        for (int lane = 0; lane < MMITSS_MAX_LANES; lane++)
        {
            if ((perf.maximum_estimation[app][lane] > 0) || (perf.queue_counter[app][lane] > 0) ||
                (perf.vehicle_count.at(app).at(lane) > 0))
            {
                Poco::DynamicStruct q;
                q["appr"] = app;
                q["lane"] = lane + 1;
                q["queueLen"] = static_cast<int>(perf.maximum_estimation[app][lane]);
                q["queueCnt"] = perf.queue_counter[app][lane];
                q["vehCnt"] = perf.vehicle_count.at(app).at(lane);
                arr.emplace_back(q);
            }
        }
    }
    out["queue"] = arr;
    Poco::DynamicStruct numVeh;
    numVeh["numVeh"] = numVehicles;
    out["status"] = numVeh;
    return out.toString();
}

std::string ttAccToJSON(const Performance& perf)
{
    Poco::DynamicStruct out;
    Poco::Dynamic::Array arr;
    for (int mov = 0; mov < MMITSS_MAX_MOVEMENTS; mov++)
    {
        Poco::DynamicStruct tt;
        if ((perf.App_TT[mov] > 0) || (perf.App_Delay[mov] > 0) || (perf.App_numstops[mov] > 0) ||
            (perf.App_throughput[mov] > 0))
        {
            tt["move"] = MovementFromIndex[mov];
            tt["tt"] = static_cast<int>(perf.App_TT[mov]);
            tt["delay"] = static_cast<int>(perf.App_Delay[mov]);
            tt["numstops"] = static_cast<int>(perf.App_numstops[mov]);
            tt["throughput"] = static_cast<int>(perf.App_throughput[mov]);
            arr.emplace_back(tt);
        }
    }
    out["ttAcc"] = arr;
    return out.toString();
}

} // namespace SieMmitss
