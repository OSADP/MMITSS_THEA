//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/* ReqEntryListHandle.cpp
 * Created by Mehdi Zamanipour
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 */

#include "ReqEntryListHandle.h"

#include <Poco/Dynamic/Var.h>
#include <Poco/Exception.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/SharedPtr.h>
#include <sys/types.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iterator>
#include <sstream>
#include <type_traits>

#include "facilityLayer.hpp"
#include "mmitssUtils.h"

namespace WaveApp
{
namespace Mmitss
{

int ReqEntryListHandle::CountVehTypesInList(VEH_PRIORITY Veh_Class)
{
    int times = 0;
    for (auto& req : m_reqListImp)
    {
        if (req.VehClass == Veh_Class)
            times++;
    }
    return times;
}

std::string ReqEntryListHandle::allVehClassAsString(int& NumberofRequests)
{
    std::ostringstream format_message;
    for (auto& req : m_reqListImp)
    {
        NumberofRequests++;
        format_message << NumberofRequests;
        format_message << " ";
        format_message << static_cast<std::underlying_type<VEH_PRIORITY>::type>(req.VehClass);
        format_message << " ";
    }

    return format_message.str();
}

std::string ReqEntryListHandle::allNonCoordinationVehClassAsString(
    int& NumberofRequests, int iNumberofTransitInList, int iNumberofTruckInList)
{
    std::ostringstream format_message;

    for (auto& req : m_reqListImp)
    {
        if (req.VehClass != VEH_PRIORITY::COORDINATION) // only consider non coordination request in
                                                        // this weighting list. THe coordination
                                                        // request weight is considered in the
                                                        // objective function
        {
            NumberofRequests++;
            if (req.VehClass == VEH_PRIORITY::TRANSIT)
                iNumberofTransitInList++;
            if (req.VehClass == VEH_PRIORITY::TRUCK)
                iNumberofTruckInList++;
            format_message << NumberofRequests;
            format_message << " ";
            format_message << static_cast<std::underlying_type<VEH_PRIORITY>::type>(req.VehClass);
            format_message << " ";
        }
    }
    return format_message.str();
}

VEH_PRIORITY ReqEntryListHandle::FindListHighestPriority()
{
    // The smaller number, the higher priority
    // TRANSIT(2) is lower than EV(1): a large number
    VEH_PRIORITY priority = VEH_PRIORITY::NORMAL;

    if (!m_reqListImp.empty())
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehClass < priority)
            {
                priority = req.VehClass;
            }
        }
    }
    return priority;
}

void ReqEntryListHandle::addPhantomReqEntry(int ReqPhase, int PhaseVCMissing)
{
    int Pos = FindRequestPhaseInList(ReqPhase);
    auto req = m_reqListImp.begin();
    if (Pos >= 0)
        req += Pos;
    ReqEntry PhantomEntry = *req; // copy this request
    PhantomEntry.Phase = PhaseVCMissing;
    m_reqListImp.emplace_back(PhantomEntry);
    ITSAPP_LOG("Add a Phantom Req Entry!");
}

// Return the first position of the request phase in the ReqTimePtList
// In case there are more vehicles request the same phases.
int ReqEntryListHandle::FindRequestPhaseInList(int phase)
{
    int posOfReq = -1;
    int i = 0;

    if (!m_reqListImp.empty())
    {
        for (auto& req : m_reqListImp)
        {
            if (req.Phase == phase)
            {
                posOfReq = i;
                break;
            }
            else
            {
                i++;
            }
        }
    }
    return posOfReq;
}

void ReqEntryListHandle::deleteRequestPhaseInList(int phase)
{
    int Pos = FindRequestPhaseInList(phase);
    if (Pos >= 0)
    {
        m_reqListImp.erase(m_reqListImp.begin() + Pos);
    }
}

std::string ReqEntryListHandle::getParamFromConfig(const RsuConfig& cfg)
{
    std::ostringstream format_message;
    const RsuConfig::RSU_Config& ConfigIS = cfg.getConfig();

    int iAlreadySetCurrent = 0;
    if (m_reqListImp.empty() == 0)
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehClass == VEH_PRIORITY::COORDINATION &&
                iAlreadySetCurrent == 0) // if it is coordination request, we should know
            {
                if (req.MinGreen > 0)
                {
                    format_message
                        << "param current:="
                        << max(ConfigIS.dCoordinationSplit[0], ConfigIS.dCoordinationSplit[1]) -
                               (double)req.MinGreen
                        << ";\n";
                }
                else if (req.ETA > 0)
                {
                    format_message << "param current:=" << ConfigIS.iCoordinationCycle - req.ETA
                                   << ";\n";
                }
                iAlreadySetCurrent = 1;
            }
        }
    }
    return format_message.str();
}

void ReqEntryListHandle::FindReqWithSamePriority(VEH_PRIORITY priority, RequestList& Req_List_New)
{

    Req_List_New.clear();

    if (m_reqListImp.empty() == 0)
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehClass == priority)
            {
                Req_List_New.emplace_back(req);
            }
        }
    }
}

std::vector<int> ReqEntryListHandle::getEVPhase()
{
    std::vector<int> EV_Phase_vc;

    for (auto& req : m_reqListImp)
    {
        if (req.VehClass == VEH_PRIORITY::EV)
            EV_Phase_vc.push_back(req.Phase);
    }
    return EV_Phase_vc;
}

// Find if the TestEntry is in the STOP list, return value is the position. //
int ReqEntryListHandle::FindInStopList(ReqEntry TestEntry)
{
    int posOfReq = -1;
    int i = 0;
    if (!m_stopList.empty())
    {
        for (auto& stop : m_stopList)
        {
            if (stop.VehID == TestEntry.VehID)
            {
                posOfReq = i;
                break;
            }
            else
            {
                i++;
            }
        }
    }
    return posOfReq;
}

// Find if the TestEntry is in the list, return value is the position.
int ReqEntryListHandle::FindInReqList(ReqEntry TestEntry)
{
    int posOfReq = -1;
    int i = 0;
    if (!m_reqListImp.empty())
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehID == TestEntry.VehID)
            {
                posOfReq = i;
                break;
            }
            else
            {
                i++;
            }
        }
    }
    return posOfReq;
}

std::vector<std::string> ReqEntryListHandle::getVehIDList()
{
    std::vector<std::string> VehID_vc;

    for (auto& req : m_reqListImp)
    {
        VehID_vc.emplace_back(req.VehID);
    }
    return VehID_vc;
}

void ReqEntryListHandle::getNonZeroWeightReq(
    double dTrnWeight, double dTrkWeight, RequestList& Req_List_New)
{

    Req_List_New.clear();
    if (!m_reqListImp.empty())
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehClass == VEH_PRIORITY::TRANSIT)
            {
                if (dTrnWeight > 0)
                    Req_List_New.emplace_back(req);
            }
            else if (req.VehClass == VEH_PRIORITY::TRUCK)
            {
                if (dTrkWeight > 0)
                    Req_List_New.emplace_back(req);
            }
            else
                Req_List_New.emplace_back(req);
        }
    }
}

bool ReqEntryListHandle::FindVehClassInList(VEH_PRIORITY VehClass)
{
    if (!m_reqListImp.empty())
    {
        for (auto& req : m_reqListImp)
        {
            if (req.VehClass == VehClass)
            {
                return true;
            }
        }
    }
    return false;
}

std::vector<std::string> ReqEntryListHandle::UpdateCurrentList(
    double curCycleTime, double coordPhaseSplit, double coordCycle)
{
    std::vector<std::string> deletedVehciles;
    auto req = m_reqListImp.begin();
    while (req != m_reqListImp.end())
    {
        if ((time(NULL) - req->AbsTime > m_reqInterval) &&
            (req->VehClass != VEH_PRIORITY::COORDINATION)) // if the time that we received the last
                                                           // SRM is (req_interval second) ago and
                                                           // the SRM has not been updated during
                                                           // this interval, this request is a
                                                           // residual request and should be
                                                           // deleted!
        {
            ITSAPP_LOG(" ************Residual request with ID %s DELETED from the requests list "
                       "************",
                Mmitss::forDisplayInHexFormat(req->VehID).c_str());
            VEH_PRIORITY HighestP = FindListHighestPriority();
            ITSAPP_LOG(" ************ Vehicle with HighestPriority is %d", HighestP);
            if ((req->VehClass == VEH_PRIORITY::EV) ||
                (HighestP != VEH_PRIORITY::EV)) // if the new request is an EV or if there is no EV
                                                // in the request list
            {
                m_reqListUpdateFlag = FLAG::DELETE;
            }
            deletedVehciles.emplace_back(req->VehID);
            req = m_reqListImp.erase(req);
        }
        else
        {
            if (req->VehClass == VEH_PRIORITY::COORDINATION)
            {
                if (coordPhaseSplit >
                    curCycleTime) // if we are in the split period of coordination phase
                {
                    req->MinGreen = float(coordPhaseSplit - curCycleTime);
                    ITSAPP_LOG(" ************ MinGreen is updated for %s",
                        Mmitss::forDisplayInHexFormat(req->VehID).c_str());
                }
                else
                {
                    req->ETA = max(coordCycle - curCycleTime, 0.0);
                    ITSAPP_LOG(" ************ ETA is updated for %s",
                        Mmitss::forDisplayInHexFormat(req->VehID).c_str());
                }
            }
            else
            {
                if (req->ETA > 0)
                    req->ETA = req->ETA - (float)ROLLING_HOR_INTERVAL;
                else
                    req->ETA = 0;
                ITSAPP_LOG(" ************ ETA is updated for %s to %f",
                    Mmitss::forDisplayInHexFormat(req->VehID).c_str(), (double)req->ETA);
            }
            req++;
        }
    }
    return deletedVehciles;
}

// IF there is EV, the change of lower priority will not change the status
void ReqEntryListHandle::UpdateList(const std::string& vehID,
    char* RcvMsg,
    VEH_PRIORITY VehClass,
    const PhaseState& phaseStatus,
    const PhaseState& combinedPhase)
{
    m_flagForClearingInterfaceCmd = 0;
    ReqEntry NewReq; // default NewReq.Split_Phase=-10;
    NewReq.VehID = vehID;
    NewReq.VehClass = VehClass;
    char RSU_ID[16], msgType[16];
    sscanf(RcvMsg, "%s %s %d %f %d %d %f %ld %d %d %d %d %d %d %d %d %d %d %d %d %d", msgType,
        RSU_ID, &NewReq.iRequestID, &NewReq.ETA, &NewReq.Phase, &NewReq.PhaseStatus,
        &NewReq.MinGreen, &NewReq.AbsTime, &NewReq.iInLane, &NewReq.iOutLane,
        &NewReq.iStartHourForDisplay, &NewReq.iStartMinuteForDisplay,
        &NewReq.iStartSecondForDisplay, &NewReq.iEndHourForDisplay, &NewReq.iEndMinuteForDisplay,
        &NewReq.iEndSecondForDisplay, &NewReq.iVehState, &NewReq.iSpeed, &NewReq.iMsgCnt,
        &NewReq.iStartMinute, &NewReq.iEndMinute);

    VEH_PRIORITY HighestP = FindListHighestPriority();
    ITSAPP_LOG(" ************ Vehicle with HighestPriority is %d", HighestP);
    int SplitPhase = 0;
    if (NewReq.VehClass == VEH_PRIORITY::EV)
    {
        SplitPhase = FindSplitPhase(NewReq.Phase, phaseStatus, combinedPhase);
        ITSAPP_LOG("split phase is: %d", SplitPhase);
    }

    //---------------Beginning of Handling the Received Requests.----------------//
    int pos = FindInReqList(NewReq);
    ITSAPP_LOG("msg type is %s", msgType);
    if ((strcmp(msgType, "request") == 0 || strcmp(msgType, "coord_request") == 0) &&
        NewReq.Phase > 0) // the vehicle is approaching the intersection or in it is in the queue
                          // and requesting a phase OR it is a coordination request
    {
        ITSAPP_LOG("Phase is %d ", NewReq.Phase);
        if (pos < 0) // Request is NOT in the List. It includes the ReqNo<=0 case as well
        {
            NewReq.Split_Phase = SplitPhase; // UPDATE the split phase when first time occurs.

            // new request, must solve, save first time solve ETA and Abstime
            if ((NewReq.VehClass == VEH_PRIORITY::EV) ||
                (HighestP != VEH_PRIORITY::EV)) // if the new request is an EV or if there is no EV
                                                // in the request list. We do not solve the problem
                                                // when there is an EV in the list and another
                                                // vehicle with lower priority send request
            {
                m_reqListUpdateFlag = FLAG::ADD;
            }
            ITSAPP_LOG("***Add Request**** { %s }\t \tFLAG {%d}.\n", RcvMsg, m_reqListUpdateFlag);
            m_reqListImp.emplace_back(NewReq);
        }
        else // The request is already in the list.
        {
            auto req = m_reqListImp.begin() + pos;
            if (req->iRequestID == NewReq.iRequestID) // the received SRM with identical RequestID
                                                      // is already in the list, so we should not
                                                      // add the new req to the list. And request
                                                      // for solving would be sent if:
            // 1. Endtime in SRM is too different from that in list
            // 2. phase change happens and differs from that in list
            {
                uint ETADiff = abs(calculateETA(req->iEndMinute, req->iEndSecondForDisplay,
                    NewReq.iEndMinute, NewReq.iEndSecondForDisplay));
                char PHASE_STA_TIME2_ASC_SYMBOL[] = {
                    '-', 'R', 'R', 'R', 'R', 'Y', 'G', 'G', '-', '-', 'Y'}; // GetSignalColor
                if (PHASE_STA_TIME2_ASC_SYMBOL[req->PhaseStatus - 1] !=
                        PHASE_STA_TIME2_ASC_SYMBOL[NewReq.PhaseStatus - 1] ||
                    ETADiff > ETA_MIN_DIFF)
                {

                    ITSAPP_LOG("PhaseStatus change: %c -> %c ",
                        PHASE_STA_TIME2_ASC_SYMBOL[req->PhaseStatus - 1],
                        PHASE_STA_TIME2_ASC_SYMBOL[NewReq.PhaseStatus - 1]);
                    ITSAPP_LOG("EndTime difference: %d, from %d:%d to %d:%d", ETADiff,
                        req->iEndMinuteForDisplay, req->iEndSecondForDisplay,
                        NewReq.iEndMinuteForDisplay, NewReq.iEndSecondForDisplay);
                    m_reqListUpdateFlag = FLAG::UPDATE; // ask for solving
                }
                else // just update information, keep old End time
                {
                    NewReq.iEndMinuteForDisplay = req->iEndMinuteForDisplay;
                    NewReq.iEndSecondForDisplay = req->iEndSecondForDisplay;
                    ITSAPP_LOG("Keep old EndTime");
                }
                m_reqListImp[pos] = NewReq; // Update the existed entry.
                req = m_reqListImp.begin() + pos;
                ITSAPP_LOG(" ************ Information is updated for %s to %ld",
                    Mmitss::forDisplayInHexFormat(req->VehID).c_str(), req->AbsTime);
                ITSAPP_LOG("%s %d %f %d %d %f %ld %d %d %d %d %d %d %d %d %d %d %d %d",
                    Mmitss::forDisplayInHexFormat(req->VehID).c_str(), req->VehClass,
                    (double)req->ETA, req->Phase, req->PhaseStatus, (double)req->MinGreen,
                    req->AbsTime, req->iInLane, req->iOutLane, req->iStartHourForDisplay,
                    req->iStartMinuteForDisplay, req->iStartSecondForDisplay,
                    req->iEndHourForDisplay, req->iEndMinuteForDisplay, req->iEndSecondForDisplay,
                    req->iRequestID, req->iVehState, req->iSpeed, req->iMsgCnt);
            }
            else
            {
                if (NewReq.VehClass == VEH_PRIORITY::EV) // Modified by YF: Solve the problem that
                                                         // if duing the first request the split
                                                         // phase is -1, it will never change!!!
                {
                    NewReq.Split_Phase = SplitPhase;
                }
                else
                {
                    NewReq.Split_Phase = req->Split_Phase;
                }
                if ((NewReq.VehClass == VEH_PRIORITY::EV) ||
                    (HighestP != VEH_PRIORITY::EV)) // if the new request is an EV or if there is no
                                                    // EV in the request list
                {
                    m_reqListUpdateFlag = FLAG::UPDATE;
                    ITSAPP_LOG("***Update Request**** { %s }\t \tFLAG {%d}.\n", RcvMsg,
                        m_reqListUpdateFlag);
                }

                m_reqListImp[pos] = NewReq; // Update the existed entry.
                req = m_reqListImp.begin() + pos;
                ITSAPP_LOG(" ************ AbsTime is updated for %s to %ld",
                    Mmitss::forDisplayInHexFormat(req->VehID).c_str(), req->AbsTime);
                ITSAPP_LOG("%s %d %f %d %f %ld %d %d %d %d %d %d %d %d %d %d %d %d",
                    Mmitss::forDisplayInHexFormat(req->VehID).c_str(), req->VehClass,
                    (double)req->ETA, req->Phase, (double)req->MinGreen, req->AbsTime, req->iInLane,
                    req->iOutLane, req->iStartHourForDisplay, req->iStartMinuteForDisplay,
                    req->iStartSecondForDisplay, req->iEndHourForDisplay, req->iEndMinuteForDisplay,
                    req->iEndSecondForDisplay, req->iRequestID, req->iVehState, req->iSpeed,
                    req->iMsgCnt);
            }
        }

        //----------------End Update the Requests list according to the received time.--------//
    }
    else if (strcmp(msgType, "request_clear") == 0)
    {
        if (pos >= 0) // this is the first time we receive the clear request
        {
            auto req = m_reqListImp.begin() + pos;
            m_reqListImp.erase(req);
            ITSAPP_LOG("***CLEAR**** %s at time (%ld).", RcvMsg, time(NULL));
            if (m_reqListImp.size() >
                0) // if there is another request in the table we should solve the problem again
                m_reqListUpdateFlag = FLAG::CANCEL;
            else
                m_flagForClearingInterfaceCmd = 1;
        }
        else
        {
            if (m_reqListImp.size() >
                0) // if there is another request in the table we should solve the problem again
                m_reqListUpdateFlag = FLAG::CANCEL;
            else
                m_flagForClearingInterfaceCmd = 1;
        }
    }
    //---------------End of Handling the Received Requests.----------------//
    if (m_reqListImp.size() == 0 && m_reqListUpdateFlag != FLAG::CANCEL)
    {
        ITSAPP_LOG("*************Empty List.");
        m_reqListUpdateFlag = FLAG::NO_ACTION;
    }
}

int ReqEntryListHandle::FindSplitPhase(
    int phase, const PhaseState& phaseStatus, const PhaseState& combinedPhase)
{
    //*** From global array CombinedPhase[] to get the combined phase :get_configfile() generates
    // CombinedPhase[]
    //*** If the phase exsits, the value is not 0; if not exists, the default value is '0'.; "-1"
    // means will not change later
    //*** The argument phase should be among {1..8}
    // int Phase_Seq[8]={1,2,3,4,5,6,7,8};
    //*** NOW also consider the case that if phase 6 is current or next, will only call 2, and 6
    // will be in the solver as well

    int combined_phase = 0;

    switch (phase)
    {
        case 1:
            combined_phase = combinedPhase[6 - 1];
            break;
        case 2: // if current phase or next phase is 6: we should not call phase 5, because cannot
            // reverse from 6 to 5;
            {
                if (phaseStatus[6 - 1] == 2 ||
                    phaseStatus[6 - 1] == 7) // do not consider not enable case: phaseStatus[6-1]==3
                    combined_phase = -1;
                else
                    combined_phase = combinedPhase[5 - 1];
                break;
            }
        case 3:
            combined_phase = combinedPhase[8 - 1];
            break;
        case 4:
        {
            if (phaseStatus[8 - 1] == 2 || combinedPhase[8 - 1] == 7)
                combined_phase = -1;
            else
                combined_phase = combinedPhase[7 - 1];
            break;
        }
        case 5:
            combined_phase = combinedPhase[2 - 1];
            break;
        case 6:
        {
            if (phaseStatus[2 - 1] == 2 || phaseStatus[2 - 1] == 7)
                combined_phase = -1;
            else
                combined_phase = combinedPhase[1 - 1];
            break;
        }
        case 7:
            combined_phase = combinedPhase[4 - 1];
            break;
        case 8:
        {
            if (phaseStatus[4 - 1] == 2 || phaseStatus[4 - 1] == 7)
                combined_phase = -1;
            else
                combined_phase = combinedPhase[3 - 1];
            break;
        }
        default:
            ITSAPP_WRN("*** Wrong Phase Information***");
            break;
    }

    return combined_phase;
}

void ReqEntryListHandle::UpdateListForPed(const PhaseState& pedPhaseStatus)
{
    for (int i = 0; i < 8; i++)
    {
        int iTempPosition = FindRequestPhaseInList(i + 1);
        if (pedPhaseStatus[i] > 0)
        {
            ReqEntry NewReq;
            NewReq.VehID =
                to_string(i + 1); // for ped, we consider the vehID! the same as ped reqeusted phase
            NewReq.VehClass = VEH_PRIORITY::PEDESTRIAN;
            NewReq.AbsTime = time(NULL);
            NewReq.iVehState = pedPhaseStatus[i];
            if (iTempPosition < 0) // this ped request is not is the table
                m_reqListImp.emplace_back(NewReq);
            else // if the ped call is in the request list but the ped status has been changed, we
                 // should update the list
            {
                auto req = m_reqListImp.begin();
                if (iTempPosition >= 0)
                    req += iTempPosition;
                if (NewReq.iVehState != req->iVehState)
                {
                    req->iVehState = NewReq.iVehState;
                    req->AbsTime = time(NULL);
                }
            }
        }
        else
        {
            if (iTempPosition >= 0)
            {
                auto req = m_reqListImp.begin() + iTempPosition;
                m_reqListImp.erase(req);
            }
        }
    }
}

std::string ReqEntryListHandle::reqListAsString() const
{
    std::ostringstream ss_msg;
    ss_msg << "current SignalRequests:" << std::endl
           << "(vehId  VehClass,    ETA    , minGrn, inLane, "
              "outLane, StartTime, EndTime)";

    for (const auto& request : m_reqListImp)
    {
        ss_msg << std::endl
               << "(" << Mmitss::forDisplayInHexFormat(request.VehID) << "     "
               << static_cast<std::underlying_type<VEH_PRIORITY>::type>(request.VehClass)
               << "          " << request.ETA << "        " << request.MinGreen << "      "
               << request.iInLane << "     " << request.iOutLane << "     "
               << request.iStartHourForDisplay << ":" << request.iStartMinuteForDisplay << ":"
               << request.iStartSecondForDisplay << "   " << request.iEndHourForDisplay << ":"
               << request.iEndMinuteForDisplay << ":" << request.iEndSecondForDisplay << ")";
    }
    return ss_msg.str();
}

std::string ReqEntryListHandle::getRequestListJson(std::vector<std::string> grantedVehicleList)
{
    std::ostringstream ss;

    if (!m_reqListImp.empty())
    {
        auto numEV = CountVehTypesInList(VEH_PRIORITY::EV);
        if (numEV == 1) // ONLY have one EV will possibly call split phase
        {
            for (auto& req : m_reqListImp)
            {
                if ((req.iInLane == 0) && (req.iOutLane == 0) && (req.iStartHourForDisplay == 0) &&
                    (req.iStartMinuteForDisplay == 0) && (req.iStartSecondForDisplay == 0) &&
                    (req.VehClass != VEH_PRIORITY::PEDESTRIAN) &&
                    (req.VehClass != VEH_PRIORITY::COORDINATION))
                {
                    ITSAPP_LOG(
                        "******************  A residual request detected ******************.");
                }
                else
                {
                    ss << "," << req.getRequestJson(false);

                    if (!(req.Split_Phase <= 0 || req.VehClass == VEH_PRIORITY::TRANSIT ||
                            req.VehClass == VEH_PRIORITY::TRUCK))
                    {
                        ss << "," << req.getRequestJson(true);
                    }
                }
            }
        }
        else // if(times!=1): will only call phases requested without split_phase
        {
            for (auto& req : m_reqListImp)
            {
                // skip un-granted requests
                if (std::find(grantedVehicleList.begin(), grantedVehicleList.end(), req.VehID) ==
                    grantedVehicleList.end())
                {
                    ITSAPP_LOG("skipping not granted: %s",
                        Mmitss::forDisplayInHexFormat(req.VehID).c_str());
                    continue;
                }

                if ((req.iInLane == 0) && (req.iOutLane == 0) && (req.iStartHourForDisplay == 0) &&
                    (req.iStartMinuteForDisplay == 0) && (req.iStartSecondForDisplay == 0) &&
                    (req.VehClass != VEH_PRIORITY::PEDESTRIAN) &&
                    (req.VehClass != VEH_PRIORITY::COORDINATION))
                {
                    ITSAPP_LOG(
                        "******************  A residual request detected ******************.");
                }
                else
                {
                    ss << "," << req.getRequestJson(false);
                }
            }
        }
    }
    ITSAPP_LOG("sending: %s", ss.str().c_str());
    return ss.str();
}

void ReqEntryListHandle::buildRequestListFromJson(std::string str)
{
    m_reqListImp.clear();
    ITSAPP_LOG("received: %s", str.c_str());
    if (str.empty())
        return;
    std::string buf = str;
    buf[0] = '[';
    buf.push_back(']');

    Poco::Dynamic::Var parsedResult = Poco::JSON::Parser().parse(buf);
    Poco::JSON::Array::Ptr json = parsedResult.extract<Poco::JSON::Array::Ptr>();

    for (auto& msg : *json)
    {
        Poco::JSON::Object::Ptr object2 = msg.extract<Poco::JSON::Object::Ptr>();
        try
        {
            Poco::Dynamic::Var test = object2->get("veh_id");
            ReqEntry req;

            req.VehID = object2->get("veh_id").toString();
            req.iRequestID = std::stoi(object2->get("req_id").toString(), nullptr, 10);
            req.VehClass = static_cast<VEH_PRIORITY>(
                std::stoi(object2->get("veh_class").toString(), nullptr, 10));
            req.ETA = std::stof(object2->get("eta").toString());
            req.Phase = std::stoi(object2->get("req_phase").toString(), nullptr, 10);
            req.MinGreen = std::stof(object2->get("min_grn").toString());
            req.iInLane = std::stoi(object2->get("in_approach").toString(), nullptr, 10) * 10 +
                          std::stoi(object2->get("in_lane").toString(), nullptr, 10);
            req.iOutLane = std::stoi(object2->get("out_approach").toString(), nullptr, 10) * 10 +
                           std::stoi(object2->get("out_lane").toString(), nullptr, 10);
            req.iStartHourForDisplay =
                std::stoi(object2->get("start_hour").toString(), nullptr, 10);
            req.iStartMinuteForDisplay =
                std::stoi(object2->get("start_minute").toString(), nullptr, 10);
            req.iStartSecondForDisplay =
                std::stoi(object2->get("start_second").toString(), nullptr, 10);
            req.iEndHourForDisplay = std::stoi(object2->get("end_hour").toString(), nullptr, 10);
            req.iEndMinuteForDisplay =
                std::stoi(object2->get("end_minute").toString(), nullptr, 10);
            req.iEndSecondForDisplay =
                std::stoi(object2->get("end_second").toString(), nullptr, 10);
            req.AbsTime = std::stoi(object2->get("abstime").toString(), nullptr, 10);
            req.iVehState = std::stoi(object2->get("veh_state").toString(), nullptr, 10);
            req.iSpeed = std::stoi(object2->get("speed").toString(), nullptr, 10);
            req.iMsgCnt = std::stoi(object2->get("msg_cnt").toString(), nullptr, 10);

            m_reqListImp.emplace_back(req);
        }
        catch (Poco::InvalidAccessException&)
        {
            ITSAPP_WRN("*** Error parsing request from Json ***");
        }
    }
}

} // namespace Mmitss
} // namespace WaveApp
