//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/*  ReqEntry.h
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

#pragma once
#include <sstream>
#include <iostream>
#include <fstream>
#include <time.h>
#include <type_traits>

using namespace std;

namespace WaveApp
{
namespace Mmitss
{

//*** When the EV is in the list, the lower priority will be ignored by the Solver now. BY DJ
// 2012.2.10
enum class VEH_PRIORITY
{
    EV = 1,           //*Emergency vehicle: will have two split phases requests and priority as 1
    TRANSIT,          //*Transit bus: will have one request and priority as 2
    TRUCK,            //
    PEDESTRIAN,       //
    COORDINATION = 6, //
    NORMAL = 10       // lowest priority of normal vehicle
};

class ReqEntry
{
  public:
    string VehID{""};                            // The ID of the vehicle
    VEH_PRIORITY VehClass{VEH_PRIORITY::NORMAL}; // The Class of the vehicle
    float ETA{0};                                // The Estimation Time of Arrival
    int Phase{0};                                // The phase of the Intersection
    int PhaseStatus{0}; // The latest phase status from PHASE_STA_TIME2_ASC in Mib.h
    float MinGreen{0};  // Mini Green time to clear the queue in front of vehicle, only for 0 speed:
                        // Queue clear time
    time_t AbsTime{0};
    int Split_Phase{-10}; // If it is =0, then no split phase; >0  represents the split phase; =-1,
                          // means will not call SplitPhase
    int iInLane{0}, iOutLane{0}, iStartHourForDisplay{0}, iStartMinuteForDisplay{0},
        iStartSecondForDisplay{0}, iEndHourForDisplay{0}, iEndMinuteForDisplay{0},
        iEndSecondForDisplay{0}, iRequestID{0}, iVehState{0}, iSpeed{0}, iMsgCnt{0};
    int iStartMinute{0}, iEndMinute{0}; // minutes since the beginning of the current year

  public:
    ReqEntry() = default;
    ReqEntry(const ReqEntry&) = default;
    ReqEntry& operator=(const ReqEntry&) = default;

    std::string getRequestJson(bool requestSplitPhase = false);

    ~ReqEntry() = default;
    friend ostream& operator<<(ostream& stream, ReqEntry e)
    {
        stream << "VehID: " << e.VehID
               << "\tClass: " << static_cast<std::underlying_type<VEH_PRIORITY>::type>(e.VehClass)
               << "\tETA: " << e.ETA << "\tPhase: " << e.Phase << "\tSplitPhase: " << e.Split_Phase
               << endl;
        return stream;
    }
};

} // namespace Mmitss
} // namespace WaveApp
