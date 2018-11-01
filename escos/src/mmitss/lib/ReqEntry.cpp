//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ReqEntry.cpp
 *  Created by Mehdi Zamanipour
 *  University of Arizona
 *  ATLAS Research Center
 *  College of Engineering
 *
 *  This code was develop under the supervision of Professor Larry Head
 *  in the ATLAS Research Center.
 *
 *  Revision History:
 */
#include "ReqEntry.h"
#include "mmitssUtils.h"
namespace WaveApp
{
namespace Mmitss
{

std::string ReqEntry::getRequestJson(bool requestSplitPhase)
{
    std::ostringstream ss;
    auto phase = (requestSplitPhase) ? Split_Phase : Phase;

    ss << R"({ "veh_id":)" << Mmitss::forDisplayInHexFormat(VehID) << R"(, "req_id":)" << iRequestID
       << R"(, "veh_class":)" << static_cast<std::underlying_type<VEH_PRIORITY>::type>(VehClass)
       << R"(, "eta":)" << ETA << R"(, "req_phase":)" << phase << R"(, "min_grn":)" << MinGreen
       << R"(, "in_approach":)" << iInLane / 10 << R"(, "in_lane":)" << iInLane % 10
       << R"(, "out_approach":)" << iOutLane / 10 << R"(, "out_lane":)" << iOutLane % 10
       << R"(, "start_hour":)" << iStartHourForDisplay << R"(, "start_minute":)"
       << iStartMinuteForDisplay << R"(, "start_second":)" << iStartSecondForDisplay
       << R"(, "end_hour":)" << iEndHourForDisplay << R"(, "end_minute":)" << iEndMinuteForDisplay
       << R"(, "end_second":)" << iEndSecondForDisplay << R"(, "abstime":)" << AbsTime
       << R"(, "veh_state":)" << iVehState << R"(, "speed":)" << iSpeed << R"(, "msg_cnt":)"
       << iMsgCnt << " }";
    return ss.str();
}

} // namespace Mmitss
} // namespace WaveApp
