//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/* ReqEntryListHandle.h
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

#pragma once

#include <string>
#include <vector>

#include "PhaseState.h"
#include "Config.h"
#include "ReqEntry.h"

namespace WaveApp
{
namespace Mmitss
{

// rolling horizon is used for doing countdown on ETA of the received SRM
const double ROLLING_HOR_INTERVAL = 499999.0 / 999999.0;
const double ETA_MIN_DIFF = 2; // in seconds

using RequestList = std::vector<ReqEntry>;

/**
 *  ReqEntryListHandle class
 */
class ReqEntryListHandle
{
  public:
    ReqEntryListHandle(){};
    virtual ~ReqEntryListHandle() = default;

    void setRSUID(std::string& id) { m_rsuId = id; }
    std::string getRSUID() const { return m_rsuId; }

    void clearReqListUpdateFlag() { m_reqListUpdateFlag = FLAG::NO_ACTION; }
    void setCoordRequestFlag() { m_reqListUpdateFlag = FLAG::COORD_REQUEST; }
    bool needToTakeAction() const { return m_reqListUpdateFlag != FLAG::NO_ACTION; }
    bool needToAddOrUpdate() const
    {
        return (m_reqListUpdateFlag == FLAG::ADD || m_reqListUpdateFlag == FLAG::UPDATE);
    }
    bool needToDelete() const { return (m_reqListUpdateFlag == FLAG::DELETE); }
    bool needToCancel() const { return (m_reqListUpdateFlag == FLAG::CANCEL); }
    bool needUpdateDB() const
    {
        return (m_reqListUpdateFlag == FLAG::NO_ACTION || m_reqListUpdateFlag == FLAG::UPDATE);
    }
    bool isCoordRequestFlag() const { return m_reqListUpdateFlag == FLAG::COORD_REQUEST; }

    void setInterfaceClearFlag(int val) { m_flagForClearingInterfaceCmd = val; }
    int getInterfaceClearFlag() const { return m_flagForClearingInterfaceCmd; }

    int size() const { return m_reqListImp.size(); }
    bool empty() const { return m_reqListImp.empty(); }
    void clear() { return m_reqListImp.clear(); }

    int CountVehTypesInList(VEH_PRIORITY Veh_Class);
    std::string allVehClassAsString(int& NumberofRequests);
    std::string allNonCoordinationVehClassAsString(
        int& NumberofRequests, int iNumberofTransitInList, int iNumberofTruckInList);
    VEH_PRIORITY FindListHighestPriority();
    void addPhantomReqEntry(int ReqPhase, int PhaseVCMissing);
    void deleteRequestPhaseInList(int phase);
    std::string getParamFromConfig(const RsuConfig& cfg);

    bool hasVehClassInListCombined(int VehClass);

    int FindInStopList(ReqEntry TestEntry);
    void UpdateList(const std::string& vehID,
        char* RcvMsg,
        VEH_PRIORITY VehClass,
        const PhaseState& phaseStatus,
        const PhaseState& combinedPhase);

    void printRequestListToFile();
    void writeCombinedRequestListToDb();
    void PrintList2FileCombined(const std::string& Filename);

    bool readCombinedRequestListFromDb();
    void ReqListFromFile_EV(const std::string& filename);

    bool FindVehClassInList(VEH_PRIORITY VehClass);
    std::vector<int> getEVPhase();
    std::vector<std::string> getVehIDList();
    int FindRequestPhaseInList(int phase);
    void FindReqWithSamePriority(VEH_PRIORITY priority, RequestList& Req_List_New);
    void getNonZeroWeightReq(double dTrnWeight, double dTrkWeight, RequestList& Req_List_New);

    int FindSplitPhase(int phase, const PhaseState& phaseStatus, const PhaseState& combinedPhase);
    std::vector<std::string> UpdateCurrentList(
        double curCycleTime, double coordPhaseSplit, double coordCycle);
    void UpdateListForPed(const PhaseState& pedPhaseStatus);
    std::string reqListAsString() const;
    std::string getRequestListJson(std::vector<std::string> grantedVehicleList);
    void buildRequestListFromJson(std::string json);

    RequestList& getReqList()
    {
        return m_reqListImp;
    } // for anybody who wants to operate on m_reqListImp directly

  private:
    int FindInReqList(ReqEntry TestEntry);

    RequestList m_reqListImp;

    RequestList m_stopList;

    std::string m_rsuId{};

    enum class FLAG // flag definition for m_reqListUpdateFlag
    {
        NO_ACTION,
        ADD,    // ADD a new request
        UPDATE, // UPDATE request (changing the speed, joining the queue, leaving the queue)
        DELETE, // DELETE an obsolete request
        CANCEL, // CANCEL a request due to leaving intersection
        COORD_REQUEST
    };
    // The Flag to identify the ReqList update
    FLAG m_reqListUpdateFlag{FLAG::NO_ACTION};
    // a flag to clear all commands in the interface when the request passed
    int m_flagForClearingInterfaceCmd{0};
    // if a request is not updated for req_interval second in request list, it should be deleted
    const int m_reqInterval{15};
};

} // namespace Mmitss
} // namespace WaveApp
