//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/* MMITSS_MRP_PriorityRequestServer_field.cpp
 * Created by Mehdi Zamanipour
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 *  1. Revised by Mehdi Zamanipour on 8/4/14 to add the posibility to receive SRM,decode it and
 *  populate Jun's buffer  (RcvMsg). Request_conbimnd txt file is updated. New informaiton such as
 *  InLane,OutLane,BeginServiceTime, EndServiceTime and IsCanceld added to each line of
 * request_combined.txt
 *  2. PRS can integrate with ISIG (COP) with argument -c 2 and can work with priority+actuation by
 *  defaule argument -c
 *  3. PRS is capable to show the ped call in the table but does not change the flag for solver to
 * solve peds
 *  4. PRS is now capable of puting cordination request when argument -o 1 is inserted
 *
 */

#include "mmitssPriorityRequestServer.hpp"

#include <Poco/Dynamic/Struct.h>
#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Object.h>
#include <Poco/Net/SocketAddress.h>
#include <sigc++/functors/mem_fun.h>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <sstream>
#include <type_traits>
#include <utility>

#include "GetInfo.h"
#include "mmitssUtils.h"
#include "ReqEntry.h"

#include "asnHelper.hpp"
#include "facilityBase.hpp"
#include "facilityDatabaseAdapter.hpp"
#include "facilityLayer.hpp"
#include "mmitssPriorityRequestServerApp.hpp"
#include "siekdbus-vtable.hpp"
#include "timeUtil.hpp"
#include "SieMmitss.hpp"
#include "SieMmitssConfig.hpp"
#include "SieMmitssSrm.hpp"

#define numPhases 8
#define RED 1
#define GREEN 3
#define YELLOW 4
#define MAX_SRM_BUFLEN 4096
#define BSM_BLOB_SIZE 38

using WaveApp::AsnHelper::iter_asn;

namespace WaveApp
{
namespace Mmitss
{

static VEH_PRIORITY saeVehRoleToMmitssPriority(int role)
{
    VEH_PRIORITY prio = VEH_PRIORITY::NORMAL;
    switch (role)
    {
        case saeBasicVehicleRole__ambulance:
        case saefire:
        case saepolice:
        case saeroadSideSource:
            prio = VEH_PRIORITY::EV;
            break;
        case saeBasicVehicleRole__transit:
            prio = VEH_PRIORITY::TRANSIT;
            break;
        case saetruck:
            prio = VEH_PRIORITY::TRUCK;
            break;
        case saepedestrian:
            prio = VEH_PRIORITY::PEDESTRIAN;
            break;
        default:
            break;
    }
    return prio;
}

MmitssPriorityRequestServer::MmitssPriorityRequestServer(MmitssPriorityRequestServerApp& parent)
    : m_parent(parent)
    , m_udpReceiver(::SieMmitss::getPrioReqSrvPort(),
          MAX_SRM_BUFLEN,
          [this](const void* data, size_t len) { return this->onUdpReceive(data, len); })
    , m_reqList()
    , m_cfg()
    , m_phases(m_cfg)
    , m_mib()
{
}

bool MmitssPriorityRequestServer::onStartup()
{

    m_parent.readDefault(m_ssmContext);
    return mmitssInit();
}

void MmitssPriorityRequestServer::onShutdown()
{

    ITSAPP_TRACE("MmitssPriorityRequestServer stopped.");
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    m_udpReceiver.stop();
    clearSsm();
}

void MmitssPriorityRequestServer::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

bool MmitssPriorityRequestServer::onRxSrm(const saeSignalRequestMessage& msg)
{
    m_recvFromUdp = false;
    if (msg.requests.count == 0)
        return true;

    auto& srm = const_cast<saeSignalRequestMessage&>(msg);

    for (auto& req : iter_asn<asncsaeSignalRequestLists>(srm.requests))
    {
        SignalRequest request;
        std::string msgType = "request_clear";
        try
        {
            request.type = req.request->requestType;

            unsigned int len;
            unsigned char* p;
            dynamic_cast<asncsaeentityID*>(msg.requestor->id)->entityID.getOctetString(&len, &p);
            ITSAPP_TRACE("vehId length: %d.", len);
            for (unsigned int i = 0; i < len; i++)
                ITSAPP_TRACE("0x%02x ", p[i]);

            request.vehId = AsnHelper::octetStringToString(
                dynamic_cast<asncsaeentityID*>(msg.requestor->id)->entityID);
            if (m_enableGrantRequest) // should be display-able numbers
                request.name = AsnHelper::octetStringToString(msg.requestor->name);

            ITSAPP_TRACE("request.vehId: %s, name: %s",
                Mmitss::forDisplayInHexFormat(request.vehId).c_str(), request.name.c_str());
            request.vehRole = msg.requestor->type->role;

            if (req.request->inBoundLane->alternative == asn_saeIntersectionAccessPoint__lane)
                request.inLaneId =
                    dynamic_cast<asncsaeIntersectionAccessPoint__lane*>(req.request->inBoundLane)
                        ->lane;
            if (req.request->outBoundLane->alternative == asn_saeIntersectionAccessPoint__lane)
                request.outLaneId =
                    dynamic_cast<asncsaeIntersectionAccessPoint__lane*>(req.request->outBoundLane)
                        ->lane;
            ITSAPP_TRACE(
                "request.inLaneId: %d, request.outLaneId: %d", request.inLaneId, request.outLaneId);
            request.minute = req.minute;
            request.second = req.second;
            request.duration = req.duration;
            ITSAPP_TRACE("request.minute: %d, request.second: %d, request.duration: %d",
                request.minute, request.second, request.duration);
            request.intersecId = req.request->id->id;
            request.requestId = req.request->requestID;
            request.seqNum = msg.sequenceNumber;
            request.speed = 0;
            ITSAPP_TRACE("request.requestId: %d, request.seqNum: %d, request.speed: %d",
                request.requestId, request.seqNum, request.speed);
        }
        catch (const std::exception& e)
        {
            ITSAPP_TRACE("data missing from message: %s.", e.what());
            return false;
        }

        processRequest(request);
    }

    return true;
}

void MmitssPriorityRequestServer::resetPriorityRequestTableInDb()
{
    auto& db = m_parent.database();
    ITSAPP_VERIFY(db.clearList(MMITSS_DB_PRIORITY_REQ_LIST));
}

bool MmitssPriorityRequestServer::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    // reset priority request table data
    resetPriorityRequestTableInDb();
    clearSsm();
    initSsm();
    m_requests.clear();
    m_reqList.clear();
    ITSAPP_VERIFY(
        m_parent.database().read("its/us/mmitss/priority/ask_for_grant", m_enableGrantRequest));

    // coordination parameters
    m_currentTime = 0.0;
    m_coordSentTime = -100.0; // - cycle time !!!
    m_coordSplitTimeFlag = 1;
    m_coordMsgCount = 0;

    //  be defulat the first coordinated phase is phase 2 and the second is phase 6
    m_coordPhase[0] = 2;
    m_coordPhase[1] = 6;

    m_lastReqFileRevisedTime = 0.0;

    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });

    //--------------------------------For the wireless
    // connection--------------------------------------//
    m_testSystemAddr = SieMmitss::getTestSystemSocketAddress();
    if (!m_testSystemAddr.empty())
        ITSAPP_LOG("Test System address: %s", m_testSystemAddr.c_str());
    //------------------------------End of the wireless connection----------------------------//
    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssPriorityRequestServer::initSsm()
{
    if (m_ssm == nullptr)
    {
        m_ssm = std::make_unique<saeSignalStatusMessage>();
        // add one intersection
        auto* status = asnCppNew asncsaeSignalStatusLists();
        status->id = asnCppNew saeIntersectionReferenceID();
        status->id->id = m_map.intersection.ID;
        m_ssm->status.addElement(status);
    }
}

void MmitssPriorityRequestServer::updateSsm()
{
    m_ssm->second = timeUtil::toDSecond(std::chrono::high_resolution_clock::now());
    vector<string> VehID_vc = m_reqList.getVehIDList();
    std::unordered_map<string, SignalRequest> requests;
    for (auto& r : VehID_vc)
    {
        requests[r] = m_requests[r];
    }
    m_requests = requests;

    auto status = dynamic_cast<asncsaeSignalStatusLists*>(m_ssm->status.getFirstElement());
    status->sequenceNumber = m_msgCnt++ % 128;
    status->id = asnCppNew saeIntersectionReferenceID;
    status->id->id = m_lanePhase.iIntersectionID;
    status->sigStatus.clearAndDestroy();
    for (auto& r : m_requests)
    {
        auto& curReq = r.second;
        auto sigStatus = asnCppNew asncsaeSignalStatusPackageLists();
        sigStatus->optional.setPresence(asn_saeSignalStatusPackage__duration);
        sigStatus->duration = curReq.duration;
        sigStatus->optional.setPresence(asn_saeSignalStatusPackage__minute);
        sigStatus->minute = curReq.minute;
        sigStatus->optional.setPresence(asn_saeSignalStatusPackage__second);
        sigStatus->second = curReq.second;
        sigStatus->optional.setPresence(asn_saerequester);
        sigStatus->requester = asnCppNew saeSignalRequesterInfo();
        sigStatus->requester->id = asnCppNew asncsaeentityID();
        dynamic_cast<asncsaeentityID*>(sigStatus->requester->id)
            ->entityID.setOctetString(TEMP_ID_LEN, (unsigned char*)curReq.vehId.c_str());
        unsigned int len;
        unsigned char* p;
        dynamic_cast<asncsaeentityID*>(sigStatus->requester->id)->entityID.getOctetString(&len, &p);
        ITSAPP_TRACE("getOctetString of %d chars ", len);
        for (unsigned int i = 0; i < len; i++)
            ITSAPP_TRACE("0x%02x ", p[i]);
        sigStatus->requester->sequenceNumber = curReq.seqNum;
        sigStatus->requester->request = curReq.requestId;
        sigStatus->requester->optional.setPresence(asn_saeSignalRequesterInfo__role);
        sigStatus->requester->role = curReq.vehRole;
        sigStatus->inboundOn = asnCppNew asncsaeIntersectionAccessPoint__lane();
        dynamic_cast<asncsaeIntersectionAccessPoint__lane*>(sigStatus->inboundOn)->lane =
            curReq.inLaneId;
        sigStatus->optional.setPresence(asn_saeoutboundOn);
        sigStatus->outboundOn = asnCppNew asncsaeIntersectionAccessPoint__lane();
        dynamic_cast<asncsaeIntersectionAccessPoint__lane*>(sigStatus->outboundOn)->lane =
            curReq.outLaneId;
        ITSAPP_TRACE(
            "curReq.inLaneId: %d ,curReq.outLaneId: %d ", curReq.inLaneId, curReq.outLaneId);
        ITSAPP_TRACE(
            "curReq.inLaneId: %d ,curReq.outLaneId: %d ", curReq.inLaneId, curReq.outLaneId);
        ITSAPP_TRACE("m_grantStatusTable[%s] is %d (0:Pending, 1:Granted, 2:Rejected).",
            Mmitss::forDisplayInHexFormat(r.first).c_str(),
            static_cast<std::underlying_type<GRANT_STATE>::type>(m_grantStatusTable[r.first]));
        if (m_grantStatusTable[r.first] == GRANT_STATE::Pending)
        {
            sigStatus->status = saeprocessing;
        }
        if (m_grantStatusTable[r.first] == GRANT_STATE::Granted)
        {
            sigStatus->status = saegranted;
        }
        if (m_grantStatusTable[r.first] == GRANT_STATE::Rejected)
        {
            sigStatus->status = saerejected;
        }
        status->sigStatus.addElement(sigStatus);
    }
}

void MmitssPriorityRequestServer::clearSsm()
{
    m_ssm = nullptr;
}

void MmitssPriorityRequestServer::onUdpReceive(const void* data, size_t len)
{
    m_recvFromUdp = true;
    if ((data == nullptr) || (len == 0))
        return;

    SieMmitss::SieMmitssSrm srm(reinterpret_cast<const char*>(data));
    if (!srm.isValid(m_enableGrantRequest))
    {
        ITSAPP_WRN("Invalid SRM JSON received: %s", reinterpret_cast<const char*>(data));
        return;
    }

    SignalRequest request;
    try
    {
        request.vehId = srm.getVehicleIdString(); // display-able string
        if (request.vehId == "0") // special id for simulate a octetString id "40 0 1 2"
        {
            asnOctetString str{TEMP_ID_LEN};
            asnbyte b[4];
            // input ascii number from 0-255
            b[0] = 64;
            b[1] = 0;
            b[2] = 1;
            b[3] = 2;
            str.setOctetString(TEMP_ID_LEN, b);
            request.vehId = AsnHelper::octetStringToString(str);
            ITSAPP_TRACE("request.vehId: %s", Mmitss::forDisplayInHexFormat(request.vehId).c_str());
        }

        if (m_enableGrantRequest)                      // should be display-able numbers
            request.name = srm.getVehicleNameString(); // display-able string
        request.type = srm.getRequestType();
        request.requestId = srm.getRequestID();

        ITSAPP_TRACE("request.name: %s", request.name.c_str());

        if (request.type != SieMmitss::RequestType::CANCELLATION)
        {
            request.intersecId = srm.getIntersectionID();
            request.vehRole = srm.getVehicleRole();
            request.inLaneId = srm.getInLaneID();
            request.outLaneId = srm.getOutLaneID();
            request.minute = srm.getEndTimeMinute();
            request.second = srm.getEndTimeSecond();
            request.duration = srm.getDuration();
            request.seqNum = srm.getMsgCount();
            request.speed = srm.getSpeed();
            ITSAPP_TRACE(
                "request.inLaneId: %d, request.outLaneId: %d", request.inLaneId, request.outLaneId);
            ITSAPP_TRACE("request.seqNum: %d, request.speed: %d", request.seqNum, request.speed);
        }
        ITSAPP_TRACE("request.requestId: %d", request.requestId);
    }
    catch (const std::exception& e)
    {
        ITSAPP_TRACE("data missing from message: %s.", e.what());
        return;
    }

    processRequest(request);
}

void MmitssPriorityRequestServer::onSsmTimer(siekdbus::timerid_t id)
{
    NOTUSED(id);
    if (not m_started)
        return;
    if (m_ssm == nullptr)
        return;
    auto status = dynamic_cast<asncsaeSignalStatusLists*>(m_ssm->status.getFirstElement());
    if (status->sigStatus.count == 0)
        return;
    ITSAPP_TRACE("sending SSM in Timer, count: %d", status->sigStatus.count);

    if (!m_parent.xmitSSM(*m_ssm, m_ssmContext))
    {
        ITSAPP_WRN("Failed sending SSM");
    }
}

void MmitssPriorityRequestServer::onTimer(siekdbus::timerid_t id)
{
    NOTUSED(id);
    if (not m_started)
        return;

    // --- Reading the ped status from controller to put the ped call in the table
    m_mib.PedStatusRead();
    if (m_onCoordination == 1)
        m_currentTime = GetSeconds();

    // Read request file and put the request in data structure and return the number of current
    // received requests
    //    int totalCurrentReq = m_reqList.ReqListFromFile(SieMmitss::getRequestsFilePath());
    int totalCurrentReq = m_reqList.size();

    if (((totalCurrentReq > 0) && (m_onCoordination == 0)) ||
        (((m_currentTime - m_lastReqFileRevisedTime) > ROLLING_HOR_INTERVAL) &&
            (totalCurrentReq > 0) && (m_onCoordination == 1)))
    { // there are still some requests in the request list, their ETA should be updated

        ITSAPP_TRACE("Current time: %lf", m_currentTime);
        ITSAPP_TRACE("Current cycle time: %lf", m_currentCycleTime);
        m_lastReqFileRevisedTime = m_currentTime;
        m_reqList.clearReqListUpdateFlag();
        vector<string> deletedVehciles =
            m_reqList.UpdateCurrentList(m_currentCycleTime, m_coordPhaseSplit, m_coordCycle);
        // if one list item is to be deleted, then clear all ETA update flag
        if (m_reqList.needToDelete())
        {
            // remove deleted Vehciles from m_grantStatusTable
            for (auto& veh : deletedVehciles)
            {
                ITSAPP_TRACE("delete %s from m_grantStatusTable",
                    Mmitss::forDisplayInHexFormat(veh).c_str());
                removeFromNameTableById(veh);
                m_grantStatusTable.erase(veh);
            }
            requestForSolveAfterDeleteRequests();
        }
        writeCombinedRequestListToDb(MMITSS_DB_PRIORITY_REQ_LIST);
        updateSsm();
    }

    // ---- handeling coordination request
    // --- Setting up the coordination requests -----
    if (m_onCoordination == 1)
    {
        // Get the current phase for setting the coordination request
        m_mib.PhaseTimingStatusRead();

        if (m_currentCycleTime >= m_coordCycle) //( (PhaseStatus[1]==7) && (PhaseStatus[5]==7) &&
                                                //(fcurrentCycleTime >= dCoordinationCycle) )
        {
            m_coordSplitTimeFlag = 0;
            m_coordSentTime = m_currentTime;
            m_reqList.setCoordRequestFlag();
        }
        m_currentCycleTime = m_currentTime - m_coordSentTime;
        // at the master clock we should put the coordination request for current cycle.
        // at the end of the coordination phase split we should cancel the current cycle request
        // and put the next cycle coordination request
        if ((m_currentCycleTime >= m_coordPhaseSplit) && (m_coordSplitTimeFlag == 0))
        {
            m_reqList.setCoordRequestFlag();
            m_coordSplitTimeFlag = 1;
        }

        if (m_reqList.isCoordRequestFlag())
        {
            double minGRn;
            float eta;
            std::string msgType = "coord_request";
            if (m_currentCycleTime <= 2)
            {
                minGRn = m_coordPhaseSplit;
                eta = 0.0;
            }
            else
            {
                minGRn = 0.0;
                eta = m_coordCycle - m_coordPhaseSplit;
            }

            m_coordMsgCount = (m_coordMsgCount + 10) % 127;
            char rcvMsg[1024];

            // ///// ***** IMPORTANT **** Coordination priority type is set to be 6 , AND the
            // request  ID is 0
            sprintf(rcvMsg, "%s %s %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d",
                msgType.c_str(), m_rsuId.c_str(), 0, (double)eta, m_coordPhase[0], minGRn,
                time(NULL), 0, 0, 0, 0, 0, 0, 0, 0, 1, m_coordMsgCount);
            ITSAPP_TRACE(
                "******************  Coordination Request Is Set ****************** At simulation "
                "time %f.",
                m_currentTime);
            ITSAPP_TRACE("{%s}", rcvMsg);
            // Update the Req List data structure considering first coordination , AND the request
            // ID is 9999
            m_reqList.UpdateList("0", rcvMsg, VEH_PRIORITY::COORDINATION, m_mib.getPhaseStatus(),
                m_cfg.getCombinedPhases());
            // the id for the second ring coordination request is set to 99999
            sprintf(rcvMsg, "%s %s %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d",
                msgType.c_str(), m_rsuId.c_str(), 99999, (double)eta, m_coordPhase[1], minGRn,
                time(NULL), 0, 0, 0, 0, 0, 0, 0, 0, 2, m_coordMsgCount);
            ITSAPP_TRACE("{%s}", rcvMsg);
            // Update the Req List data structure considering first coordination
            m_reqList.UpdateList("99999", rcvMsg, VEH_PRIORITY::COORDINATION,
                m_mib.getPhaseStatus(), m_cfg.getCombinedPhases());
            updateSsm();
        }
    }

    if (((m_reqList.needToTakeAction()) && (m_reqList.size() == 0)) ||
        (m_reqList.getInterfaceClearFlag() == 1))
    { // Request list is empty and the last vehicle just passed the intersection
        m_reqList.setInterfaceClearFlag(0);
        m_reqList.clearReqListUpdateFlag();
        m_reqList.UpdateCurrentList(m_currentCycleTime, m_coordPhaseSplit, m_coordCycle);
        //        m_reqList.printRequestListToFile();
        updateSsm();
        // in case the program is being used for actuation and priority this part of code send a
        // hold command to interface for all phases
        // when ever the last request pass the intersection to delete previous command
        if (m_codeUsage == 1)
            m_mib.SendClearCommands();
    }
}

void MmitssPriorityRequestServer::processRequest(const SignalRequest& req)
{

    if (req.type != saepriorityCancellation && req.intersecId != m_lanePhase.iIntersectionID)
    {
        ITSAPP_WRN("m_lanePhase.iIntersectionID: %d is different from req.intersecId: %d",
            m_lanePhase.iIntersectionID, req.intersecId);
        return;
    }

    std::string msgType = (req.type == saepriorityCancellation) ? "request_clear" : "request";

    char rcvMsg[1024];

    auto priority = VEH_PRIORITY::NORMAL;

    if (req.type != saepriorityCancellation)
    {
        auto inApproachNum = m_map.getApproach(req.inLaneId);
        auto outApproachNum = m_map.getApproach(req.outLaneId);
        auto inLaneNum = m_map.getLaneNum(req.inLaneId);
        auto outLaneNum = m_map.getLaneNum(req.outLaneId);

        ITSAPP_TRACE("SRM request: In: %d.%d, Out:%d.%d", inApproachNum, inLaneNum, outApproachNum,
            outLaneNum);

        if ((inLaneNum <= 0) || (outLaneNum <= 0))
        {
            ITSAPP_WRN("invalid/missing inLane/outLane in SRM");
            return;
        }
        auto requestedPhase =
            m_lanePhase.iInLaneOutLanePhase[inApproachNum][inLaneNum][outApproachNum][outLaneNum];
        ITSAPP_TRACE("Phase: %d", requestedPhase);

        // MMITSS encodes lane and approach in that single int ...
        inLaneNum += inApproachNum * 10;
        outLaneNum += outApproachNum * 10;

        // the value in duration should be ms, while minGrn is in seconds
        double minGrn = static_cast<double>(req.duration) / 1000;
        auto curMinute = WaveApp::timeUtil::toMinuteOfTheYear(WaveApp::Clock::now());

        auto startHour = (curMinute / 60) % 24;
        auto startMinute = curMinute % 60;
        auto startSecond = (WaveApp::timeUtil::toDSecond(WaveApp::Clock::now()) / 1000) % 60;
        ITSAPP_TRACE(
            "startHour: %d, startMinute: %d, startSecond: %d", startHour, startMinute, startSecond);

        auto endHour = (req.minute / 60) % 24;
        auto endMinute = req.minute % 60;
        auto endSecond = (req.second / 1000) % 60;
        ITSAPP_WRN("endHour: %d, endMinute: %d, endSecond: %d", endHour, endMinute, endSecond);

        int ETA = calculateETA(curMinute, startSecond, req.minute, endSecond);
        ITSAPP_TRACE("calculateETA: %d", ETA);
        // If ETA is less than -60 the SRM should be ignored (because it’s too far in the past to be
        // still considered)
        if (ETA < -60)
        {
            ITSAPP_WRN("the SRM is too far in the past.");
            return;
        }
        // If ETA is greater than 120 then we should also ignore it as too far in the future
        else if (ETA > 120)
        {
            ITSAPP_WRN("the SRM is too far in the future.");
            return;
        }
        // If ETA is between -60 and 0 we could continue with ETA = 0 and still consider it (even
        // though it’s really kind of late)
        else if (ETA >= -60 and ETA < 0)
        {
            ETA = 0;
            endMinute = startMinute;
            endSecond = startSecond;
            curMinute = req.minute;
        }
        ITSAPP_TRACE("endHour: %d, endMinute: %d, endSecond: %d, ETA: %d", endHour, endMinute,
            endSecond, ETA);

        // NOTE: this doesn't seem to needed
        int vehicleState = 0;

        priority = saeVehRoleToMmitssPriority(req.vehRole);

        m_requests[req.vehId] = req;

        // Get the current phase status for determining the split phases
        m_mib.PhaseTimingStatusRead();

        auto phaseStatus = m_mib.getPhaseStatus()[requestedPhase - 1];
        ITSAPP_TRACE("Status of request Phase: %c", m_mib.GetSignalColor(phaseStatus));

        sprintf(rcvMsg, "%s %s %d %.2f %d %d %.2f %ld %d %d %d %d %d %d %d %d %d %d %d %d %d",
            msgType.c_str(), m_rsuId.c_str(), req.requestId, (double)ETA, requestedPhase,
            phaseStatus, minGrn, time(NULL), inLaneNum, outLaneNum, startHour, startMinute,
            startSecond, endHour, endMinute, endSecond, vehicleState, req.speed, req.seqNum,
            curMinute, req.minute);
        ITSAPP_LOG("******************  SRM  Received ****************** ");
        ITSAPP_LOG(
            "msgType, Priority,    ETA    , minGrn(duration), inLane, outLane, StartTime, EndTime");
        std::stringstream srmLog;
        srmLog << msgType << "     "
               << static_cast<std::underlying_type<VEH_PRIORITY>::type>(priority) << "          "
               << ETA << "              " << minGrn << "           " << inLaneNum / 10 << "."
               << inLaneNum % 10 << "     " << outLaneNum / 10 << "." << outLaneNum % 10 << "     "
               << startHour << ":" << startMinute << ":" << startSecond << "   " << endHour << ":"
               << endMinute << ":" << endSecond;
        ITSAPP_LOG("%s", srmLog.str().c_str());
        ITSAPP_LOG("********Current Signal status:************");
        ITSAPP_LOG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t", 1, 2, 3, 4, 5, 6, 7, 8);
        ITSAPP_LOG("%c\t%c\t%c\t%c\t%c\t%c\t%c\t%c\t\n",
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[0]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[1]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[2]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[3]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[4]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[5]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[6]),
            m_mib.GetSignalColor(m_mib.getPhaseStatus()[7]));
    }
    else
    {
        sprintf(rcvMsg, "%s %s", msgType.c_str(), m_rsuId.c_str());
        ITSAPP_LOG("******************  SRM  Received ****************** ");
        ITSAPP_LOG("Type, RsuID, ID");
        ITSAPP_LOG("%s", rcvMsg);
    }

    // Update the Req List data structure considering received message
    m_reqList.clearReqListUpdateFlag();
    m_reqList.UpdateList(
        req.vehId, rcvMsg, priority, m_mib.getPhaseStatus(), m_cfg.getCombinedPhases());
    if (m_reqList.needToAddOrUpdate())
    {
        if (m_enableGrantRequest)
        {
            ITSAPP_TRACE("m_grantStatusTable[%s] is set to pending, name is %s",
                Mmitss::forDisplayInHexFormat(req.vehId).c_str(), req.name.c_str());
            m_grantStatusTable[req.vehId] = GRANT_STATE::Pending;
            m_nameTable[req.name] = req.vehId;
            requestForSchedule(req.name);
        }
        else
        {
            ITSAPP_TRACE("m_grantStatusTable[%s] is Granted",
                Mmitss::forDisplayInHexFormat(req.vehId).c_str());
            m_grantStatusTable[req.vehId] = GRANT_STATE::Granted;
            requestForSolve();
        }
    }
    else if (m_reqList.needToCancel())
    {
        ITSAPP_TRACE(
            "delete %s from m_grantStatusTable", Mmitss::forDisplayInHexFormat(req.vehId).c_str());
        m_grantStatusTable.erase(req.vehId);
        removeFromNameTableById(req.vehId);
        requestForSolveAfterDeleteRequests();
    }
    if (m_reqList.empty()) // last request gets cancelled
    {
        m_grantStatusTable.clear();
        m_nameTable.clear();
    }

    writeCombinedRequestListToDb(MMITSS_DB_PRIORITY_REQ_LIST);
    updateSsm();
}

void MmitssPriorityRequestServer::requestForSchedule(string name)
{
    if (m_recvFromUdp) // only for testing, done in MmitssCtrl
        return;

    Poco::DynamicStruct request;
    request["command"] = "requestPriority";
    request["rsu_host"] = getRsuId();
    request["intersection_id"] = to_string(getIntersectionId());
    request["vehicle_id"] = name;

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitssctl", "/com/siemens/c2x/mmitssctl",
        "com.siemens.c2x.mmitssctl.Service", "Command", "ss", "json", request.toString().c_str());
}

void MmitssPriorityRequestServer::enableGrantRequest(bool enable)
{
    ITSAPP_TRACE("m_grantStatusTable is cleared");
    m_grantStatusTable.clear();
    m_nameTable.clear();
    ITSAPP_TRACE("m_enableGrantRequest is set to %d", enable);
    m_enableGrantRequest = enable;
}

MmitssPriorityRequestServer::VehcileIdList MmitssPriorityRequestServer::getGrantedVehciles()
{
    VehcileIdList vehList;
    ITSAPP_TRACE("m_grantStatusTable contains %d vehicles", m_grantStatusTable.size());
    for (auto& veh : m_grantStatusTable)
    {
        if (veh.second == GRANT_STATE::Granted)
        {
            ITSAPP_TRACE("m_grantStatusTable[%s] is Granted",
                Mmitss::forDisplayInHexFormat(veh.first).c_str());
            vehList.emplace_back(veh.first);
        }
    }
    return vehList;
}
void MmitssPriorityRequestServer::tick()
{
    ITSAPP_TRACE("tick start..started: %d (0: not started yet, 1: started)", m_started);
    if (m_started)
        return;
    // When loading from mmitss config files or mib is ready
    if (m_delayedInit.valid() && m_delayedInit.wait_for(1ms) == std::future_status::ready)
    {
        if (m_delayedInit.get() == true)
        {
            if (m_timerId == 0)
            {
                m_timerConn = siekdbus::timerCreate(m_timerId, m_timerInterval, true,
                    sigc::mem_fun(*this, &MmitssPriorityRequestServer::onTimer));
            }
            else
            {
                siekdbus::timerSetInterval(m_timerId, m_timerInterval);
                siekdbus::timerSetEnabled(m_timerId, true);
            }
            if (m_ssmTimerId == 0)
            {
                m_ssmTimerConn = siekdbus::timerCreate(m_ssmTimerId, m_ssmTimerInterval, true,
                    sigc::mem_fun(*this, &MmitssPriorityRequestServer::onSsmTimer));
            }
            else
            {
                siekdbus::timerSetInterval(m_ssmTimerId, m_ssmTimerInterval);
                siekdbus::timerSetEnabled(m_ssmTimerId, true);
            }
            if (m_udpReceiver.start() == false)
            {
                ITSAPP_ERROR("m_udpReceiver started failed.");
                return;
            }
            ITSAPP_TRACE("Started successfully..");
            m_started = true;
        }
    }
    ITSAPP_TRACE("tick end..started: %d (0: not started yet, 1: started)", m_started);
}

bool MmitssPriorityRequestServer::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..");
    if (!m_lanePhase.ReadLanePhaseMap(SieMmitss::getInLaneOutLanePhaseMappingFilePath()))
        return false;

    // cout<<"m_rollingHorInterval"<<m_rollingHorInterval<<endl;

    //--- deleting the residual the Results.txt and requests.txt-------//
    FILE* fp_req = fopen(SieMmitss::getRequestsFilePath().c_str(), "w");
    if (fp_req != nullptr)
    {
        fprintf(fp_req, "Num_req -1 0\n");
        fclose(fp_req);
    }
    fp_req = fopen(SieMmitss::getRequestsCombinedFilePath().c_str(), "w");
    if (fp_req != nullptr)
    {
        fprintf(fp_req, "Num_req -1 0\n");
        fclose(fp_req);
    }
    //--- end of deleting the Results.txt and requests.txt-------//

    int curPlan = m_mib.CurTimingPlanRead();
    ITSAPP_LOG("Current timing plan is: %d", curPlan);

    // Generate: configinfo_XX.txt
    // ---getting the configurations
    m_cfg.ReadInConfig(SieMmitss::getConfigInfoFilePath());
    m_mib.setControllerAddress(get_ip_address());
    // Get the current RSU ID from "rsuid.txt" into string "RSUID"
    m_rsuId = get_rsu_id();
    m_reqList.setRSUID(m_rsuId);
    // getting the priority configuration
    if (!ReadInCoordinationConfig(SieMmitss::getPriorityConfigurationFilePath()))
        return false;

    m_map.ParseIntersection(SieMmitss::getMapFilePath().c_str());
    // geting the current active phase from controller
    m_mib.PhaseTimingStatusRead(); // First found out the not used(disabled) phases if there is any.

    for (size_t i = 0; i < m_mib.getPhaseStatus().size(); i++)
    {
        if (m_mib.getPhaseStatus()[i] == 3)
            m_phaseNotEnabledList.push_back(i + 1);
    }
    if (m_phaseNotEnabledList.size() > 0)
    {
        m_phaseNotEnabledList.push_back(100); // 100 is a random number, later the
                                              // m_phaseNotEnabledList will be used to search the
                                              // unused phases
    }
    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssPriorityRequestServer::removeFromNameTableById(std::string id)
{
    auto mapping = m_nameTable.begin();
    while (mapping != m_nameTable.end())
    {
        if (mapping->second == id)
        {
            mapping = m_nameTable.erase(mapping);
            break;
        }
        mapping++;
    }
}

void MmitssPriorityRequestServer::updateGrantedStatus(std::string name, std::string granted)
{
    std::string current = timeUtil::getCurrentTimeStampInMS();
    ITSAPP_LOG("current time in ms: %s", current.c_str());
    if (not m_enableGrantRequest)
    {
        ITSAPP_LOG(" grant is not necessary");
        return;
    }
    ITSAPP_TRACE("update granted status for %s as %s", name.c_str(), granted.c_str());
    try
    {
        std::string vehID = m_nameTable.at(name);
        if (m_grantStatusTable.at(vehID) != GRANT_STATE::Granted and
            granted == "true") // first time granted
        {
            ITSAPP_TRACE("m_grantStatusTable[%s] is set to Granted",
                Mmitss::forDisplayInHexFormat(vehID).c_str());
            m_grantStatusTable[vehID] = GRANT_STATE::Granted;
            ITSAPP_LOG(" ********Need to solve.************");
            requestForSolve();
        }
        if (granted == "false")
        {
            ITSAPP_TRACE("m_grantStatusTable[%s] is set to Rejected",
                Mmitss::forDisplayInHexFormat(vehID).c_str());
            // do not delete request from here, wait for it to timeout
            m_grantStatusTable[vehID] = GRANT_STATE::Rejected;
        }
    }
    catch (exception&) // vehicleId does not appear in m_grantStatusTable
    {
        ITSAPP_TRACE("id of %s is not in m_grantStatusTable.", name.c_str());
        return;
    }
}

void MmitssPriorityRequestServer::requestForSolveAfterDeleteRequests()
{
    VehcileIdList grantedVehciles = getGrantedVehciles();
    if (not grantedVehciles.empty())
    {
        requestForSolve();
    }
}

void MmitssPriorityRequestServer::requestForSolve()
{
    VehcileIdList grantedVehciles = getGrantedVehciles();
    if (grantedVehciles.empty())
        return;
    ITSAPP_LOG(" ********Need to solve.************");
    Poco::JSON::Object toSolve;
    ITSAPP_TRACE("send Requests list size: %d to solver.", m_reqList.size());
    toSolve.set("command", "needSolve");
    toSolve.set("list", m_reqList.getRequestListJson(grantedVehciles));

    std::ostringstream buf;
    toSolve.stringify(buf, 0, 0);

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_solver",
        "/com/siemens/c2x/mmitss_prio_solver", "com.siemens.c2x.mmitss_prio_solver.Service",
        "Command", "ss", "json", buf.str().c_str());
}

void MmitssPriorityRequestServer::writeCombinedRequestListToDb(std::string listName)
{
    its::DatabaseListData reqListDb;
    auto& reqList = m_reqList.getReqList();
    ITSAPP_TRACE("Write Requests list to database: %s.", listName.c_str());
    int CurPhase;
    int SplitPhase;
    if (!m_reqList.empty())
    {
        auto numEV = m_reqList.CountVehTypesInList(VEH_PRIORITY::EV);
        if (numEV == 1) // ONLY have one EV will possibly call split phase
        {
            for (auto& req : reqList)
            {
                if ((req.iInLane == 0) && (req.iOutLane == 0) && (req.iStartHourForDisplay == 0) &&
                    (req.iStartMinuteForDisplay == 0) && (req.iStartSecondForDisplay == 0) &&
                    (req.VehClass != VEH_PRIORITY::PEDESTRIAN) &&
                    (req.VehClass != VEH_PRIORITY::COORDINATION))
                {
                    ITSAPP_TRACE(
                        "******************  A residual request detected ******************.");
                }
                else
                {
                    CurPhase = req.Phase; // Current request phase
                    SplitPhase = req.Split_Phase;

                    auto reqObj = std::make_shared<its::DatabaseListItem>();

                    ITSAPP_TRACE(
                        "write into database: %s %s %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d "
                        "%d %d %d %d",
                        m_rsuId.c_str(), Mmitss::forDisplayInHexFormat(req.VehID).c_str(),
                        req.VehClass, (double)req.ETA, req.Phase, (double)req.MinGreen, req.AbsTime,
                        req.iInLane, req.iOutLane, req.iStartHourForDisplay,
                        req.iStartMinuteForDisplay, req.iStartSecondForDisplay,
                        req.iEndHourForDisplay, req.iEndMinuteForDisplay, req.iEndSecondForDisplay,
                        req.iRequestID, req.iVehState, req.iSpeed, req.iMsgCnt);

                    // write in CurPhase first
                    reqObj->content["rsu_id"] = m_rsuId;
                    reqObj->content["veh_id"] = Mmitss::forDisplayInHexFormat(req.VehID);
                    reqObj->content["veh_class"] = std::to_string(
                        static_cast<std::underlying_type<VEH_PRIORITY>::type>(req.VehClass));
                    reqObj->content["eta"] = std::to_string(req.ETA);
                    reqObj->content["req_phase"] = std::to_string(CurPhase);
                    reqObj->content["phase_status"] = std::to_string(req.PhaseStatus);
                    reqObj->content["min_grn"] = std::to_string(req.MinGreen);
                    reqObj->content["in_approach"] = std::to_string(req.iInLane / 10);
                    reqObj->content["in_lane"] = std::to_string(req.iInLane % 10);
                    reqObj->content["out_approach"] = std::to_string(req.iOutLane / 10);
                    reqObj->content["out_lane"] = std::to_string(req.iOutLane % 10);
                    std::stringstream intime;
                    intime << req.iStartHourForDisplay << ":" << req.iStartMinuteForDisplay << ":"
                           << req.iStartSecondForDisplay;
                    reqObj->content["start_time"] = intime.str();
                    std::stringstream outtime;
                    outtime << req.iEndHourForDisplay << ":" << req.iEndMinuteForDisplay << ":"
                            << req.iEndSecondForDisplay;
                    reqObj->content["end_time"] = outtime.str();

                    reqObj->content["abstime"] = std::to_string(req.AbsTime);
                    reqObj->content["req_id"] = std::to_string(req.iRequestID);
                    reqObj->content["veh_state"] = std::to_string(req.iVehState);
                    reqObj->content["speed"] = std::to_string(req.iSpeed);
                    reqObj->content["msg_cnt"] = std::to_string(req.iMsgCnt);

                    ITSAPP_TRACE("write into database: %s with CurPhase %d",
                        (reqObj->content["veh_id"]).c_str(), CurPhase);
                    reqListDb.emplace_back(reqObj);

                    if (!(SplitPhase <= 0 || req.VehClass == VEH_PRIORITY::TRANSIT ||
                            req.VehClass == VEH_PRIORITY::TRUCK))
                    {
                        reqObj->content["req_phase"] = std::to_string(SplitPhase);
                        ITSAPP_TRACE("write into db: %s with SplitPhase %d",
                            (reqObj->content["veh_id"]).c_str(), SplitPhase);
                        reqListDb.emplace_back(reqObj);
                    }
                }
            }
        }
        else // if(times!=1): will only call phases requested without split_phase
        {
            for (auto& req : reqList)
            {

                ITSAPP_TRACE(
                    "write into database: %s %s %d %.2f %d %.2f %ld %d %d %d %d %d %d %d %d %d %d "
                    "%d %d",
                    m_rsuId.c_str(), Mmitss::forDisplayInHexFormat(req.VehID).c_str(), req.VehClass,
                    (double)req.ETA, req.Phase, (double)req.MinGreen, req.AbsTime, req.iInLane,
                    req.iOutLane, req.iStartHourForDisplay, req.iStartMinuteForDisplay,
                    req.iStartSecondForDisplay, req.iEndHourForDisplay, req.iEndMinuteForDisplay,
                    req.iEndSecondForDisplay, req.iRequestID, req.iVehState, req.iSpeed,
                    req.iMsgCnt);

                if ((req.iInLane == 0) && (req.iOutLane == 0) && (req.iStartHourForDisplay == 0) &&
                    (req.iStartMinuteForDisplay == 0) && (req.iStartSecondForDisplay == 0) &&
                    (req.VehClass != VEH_PRIORITY::PEDESTRIAN) &&
                    (req.VehClass != VEH_PRIORITY::COORDINATION))
                {
                    ITSAPP_TRACE(
                        "******************  A residual request detected ******************.");
                }
                else
                {
                    auto reqObj = std::make_shared<its::DatabaseListItem>();
                    reqObj->content["rsu_id"] = m_rsuId;
                    reqObj->content["veh_id"] = Mmitss::forDisplayInHexFormat(req.VehID);
                    reqObj->content["veh_class"] = std::to_string(
                        static_cast<std::underlying_type<VEH_PRIORITY>::type>(req.VehClass));
                    reqObj->content["eta"] = std::to_string(req.ETA);
                    reqObj->content["req_phase"] = std::to_string(req.Phase);
                    reqObj->content["phase_status"] = std::to_string(req.PhaseStatus);
                    reqObj->content["min_grn"] = std::to_string(req.MinGreen);
                    reqObj->content["in_approach"] = std::to_string(req.iInLane / 10);
                    reqObj->content["in_lane"] = std::to_string(req.iInLane % 10);
                    reqObj->content["out_approach"] = std::to_string(req.iOutLane / 10);
                    reqObj->content["out_lane"] = std::to_string(req.iOutLane % 10);
                    std::stringstream intime;
                    intime << req.iStartHourForDisplay << ":" << req.iStartMinuteForDisplay << ":"
                           << req.iStartSecondForDisplay;
                    reqObj->content["start_time"] = intime.str();
                    std::stringstream outtime;
                    outtime << req.iEndHourForDisplay << ":" << req.iEndMinuteForDisplay << ":"
                            << req.iEndSecondForDisplay;
                    reqObj->content["end_time"] = outtime.str();

                    reqObj->content["abstime"] = std::to_string(req.AbsTime);
                    reqObj->content["req_id"] = std::to_string(req.iRequestID);
                    reqObj->content["veh_state"] = std::to_string(req.iVehState);
                    reqObj->content["speed"] = std::to_string(req.iSpeed);
                    reqObj->content["msg_cnt"] = std::to_string(req.iMsgCnt);

                    reqListDb.emplace_back(reqObj);
                }
            }
        }
    } // end of if (!m_reqList.ListEmpty())
    ITSAPP_VERIFY(m_parent.database().replaceList(listName.c_str(), reqListDb));
}

bool MmitssPriorityRequestServer::ReadInCoordinationConfig(const std::string& filename)
{
    char TempStr[64];
    double dCoordinationWeight;
    int iCoordinatedPhase[2];
    double dTransitWeight;
    double dTruckWeight;
    double dCoordOffset;
    double dCoordCycle;
    double dCoordSplit;
    std::string lineread;
    std::fstream FileRead2;
    FileRead2.open(filename.c_str(), ios::in);
    if (!FileRead2)
    {
        ITSAPP_LOG("Unable to open file! %s", filename.c_str());
        return false;
    }
    //----------------- Read in the parameters---------------
    while (!FileRead2.eof())
    {
        getline(FileRead2, lineread);

        if ((lineread.size() != 0) && (lineread.size() < 64))
        {
            sscanf(lineread.c_str(), "%s", TempStr);
            if (strcmp(TempStr, "coordination") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dCoordinationWeight);
            }
            else if (strcmp(TempStr, "cycle") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dCoordCycle);
            }
            else if (strcmp(TempStr, "offset") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dCoordOffset);
            }
            else if (strcmp(TempStr, "split") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dCoordSplit);
            }
            else if (strcmp(TempStr, "coordinated_phase") == 0)
            {
                sscanf(
                    lineread.c_str(), "%*s %d %d ", &iCoordinatedPhase[0], &iCoordinatedPhase[1]);
            }
            else if (strcmp(TempStr, "transit_weight") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dTransitWeight);
            }
            else if (strcmp(TempStr, "truck_weight") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dTruckWeight);
            }
        }
    }
    FileRead2.close();
    m_coordCycle = dCoordCycle;
    m_coordPhaseSplit = dCoordSplit;
    m_coordPhase[0] = iCoordinatedPhase[0];
    m_coordPhase[1] = iCoordinatedPhase[1];
    return true;
}

} // namespace Mmitss
} // namespace WaveApp
