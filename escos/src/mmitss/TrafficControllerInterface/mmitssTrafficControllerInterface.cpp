/* Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* MMITSS_MRP_TrafficControllerInterface.cpp
 * Created by :Yiheng Feng
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 */

/* MMITSS MRP Traffic Controller Interface
 * This component is reponsible to receive signal timing schedule from
 * MRP_TrafficControl and MRP_PriorityRequestServer and send control
 * commands (NTCIP: FORCE_OFF, VEH_CALL, PHASE_OMIT, PHASE_HOLD) to
 * Econolite ASC3/Cobalt controller
 *
 * Created by: Yiheng Feng 10/28/2014
 *
 * Department of Systems and Industrial Engineering
 * University of Arizona
 */

#include "mmitssTrafficControllerInterface.hpp"

#include <Poco/Net/SocketAddress.h>
#include <sigc++/functors/mem_fun.h>
#include <stddef.h>
#include <sys/types.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <thread>

#include "facilityBase.hpp"
#include "facilityDatabaseAdapter.hpp"
#include "facilityLayer.hpp"
#include "mmitssTrafficControllerInterfaceApp.hpp"
#include "timeUtil.hpp"
#include "GetInfo.h"
#include "SieMmitssConfig.hpp"

#ifndef DEG2ASNunits
#define DEG2ASNunits (1000000.0) // used for ASN 1/10 MICRO deg to unit converts
#endif

namespace WaveApp
{
namespace Mmitss
{

static const int RX_SCHEDULE_BUF_SIZE = 1024;

MmitssTrafficControllerInterface::MmitssTrafficControllerInterface(
    MmitssTrafficControllerInterfaceApp& parent)
    : m_parent(parent)
    , m_udpReceiver(SieMmitss::getTrafficControllerInterfacePort(),
          RX_SCHEDULE_BUF_SIZE,
          [this](const void* data, size_t len) {
              return this->onSignalCtrlScheduleReceive(data, len);
          })
    , m_cfg()
    , m_phases(m_cfg)
    , m_mib()
{
}

bool MmitssTrafficControllerInterface::onStartup()
{
    ITSAPP_TRACE("Startup");

    return mmitssInit();
}

void MmitssTrafficControllerInterface::onShutdown()
{
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    m_udpReceiver.stop();
}

void MmitssTrafficControllerInterface::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

void MmitssTrafficControllerInterface::resetEventTableInDb()
{
    ITSAPP_VERIFY(m_parent.database().clearList("its/us/mmitss/control-if/event_table_list"));
    ITSAPP_VERIFY(m_parent.database().write("its/us/mmitss/control-if/time_diff", "N/A"));
    ITSAPP_VERIFY(m_parent.database().write("its/us/mmitss/control-if/start_time", "N/A"));
}

bool MmitssTrafficControllerInterface::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    // reset priority request table data
    resetEventTableInDb();
    m_newSchedule = 0;
    m_eventListR1.clear();
    m_eventListR2.clear();

    // clear current arrival table
    for (auto& e : m_eventTimeTable)
        e.fill(0);

    m_mib.setControllerAddress(get_ip_address());
    // Get the current RSU ID from "rsuid.txt" into string "RSUID"
    m_rsuId = get_rsu_id();

    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });
    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);

    return true;
}

void MmitssTrafficControllerInterface::onTimer(siekdbus::timerid_t id)
{
    (void)(id);
    if (not m_started)
        return;

    m_currentTime = GetSeconds();

    int forceoff_cmd = 0;
    int vehcall_cmd = 0;
    int hold_cmd = 0;
    int omit_cmd = 0;
    int pedcall_cmd = 0;
    int pedclear_cmd = 0;

    if (m_eventListR1.empty() && m_eventListR2.empty())
    {
        ITSAPP_VERIFY(m_parent.database().write("its/us/mmitss/control-if/start_time", "N/A"));
        ITSAPP_VERIFY(m_parent.database().write("its/us/mmitss/control-if/time_diff", "N/A"));
        return;
    }

    auto timeDiff = m_currentTime - m_beginTime;
    ITSAPP_VERIFY(m_parent.database().write(
        "its/us/mmitss/control-if/time_diff", std::to_string(static_cast<int>(timeDiff))));

    for (auto& event : m_eventListR1)
    {
        if (timeDiff < event.time)
        { // used as hold command or omit
            if ((event.action == Hold) && (m_currentTime - m_holdTimerR1 > 1.0))
            { // hold the current phase every one second
                hold_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("HOLD Phase %d at time %.2lf", event.phase, m_currentTime);
                m_holdTimerR1 = m_currentTime;
            }
            if (event.action == Omit)
            { // omit the current phase every one second
                omit_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("OMIT Phase %d at time %.2lf", event.phase, m_currentTime);
            }
        }

        if ((timeDiff >= event.time) && (timeDiff <= (event.time + 2)))
        { // trigger event in the the time list
            if (event.action == ForceOff)
            { // force off
                forceoff_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("FORCE_OFF Phase %d at time %.2lf", event.phase, m_currentTime);
            }
            if (event.action == VehCall)
            { // vehicle call
                vehcall_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("VEH_CALL Phase %d at time %.2lf", event.phase, m_currentTime);
            }
            if (event.action == PedCall)
            { // ped call
                pedcall_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("PED_CALL Phase %d at time %.2lf", event.phase, m_currentTime);
            }
            if (event.action == PedClear)
            { // ped clear
                pedclear_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("PED_CLEAR Phase %d at time %.2lf", event.phase, m_currentTime);
            }
        }
    }
    for (auto& event : m_eventListR2)
    {
        if (timeDiff < event.time) // used as hold command
        {
            if ((event.action == Hold) && ((m_currentTime - m_holdTimerR2) > 1.0))
            { // hold the current phase every one second
                hold_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("HOLD Phase %d at time %.2lf", event.phase, m_currentTime);
                m_holdTimerR2 = m_currentTime;
            }
            if (event.action == Omit)
            { // omit the current phase every one second
                omit_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("OMIT Phase %d at time %.2lf", event.phase, m_currentTime);
            }
        }

        if ((timeDiff >= event.time) && (timeDiff <= (event.time + 2)))
        { // trigger event in the the time list
            if (event.action == ForceOff)
            { // force off
                forceoff_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("FORCE_OFF Phase %d at time %2lf", event.phase, m_currentTime);
            }
            if (event.action == VehCall)
            { // vehicle call
                vehcall_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("VEH_CALL Phase %d at time %2lf", event.phase, m_currentTime);
            }
            if (event.action == PedCall)
            { // ped call
                pedcall_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("PED_CALL Phase %d at time %2lf", event.phase, m_currentTime);
            }
            if (event.action == PedClear)
            { // ped clear
                pedclear_cmd += (int)pow(2.0, event.phase - 1);
                ITSAPP_LOG("PED_CLEAR Phase %d at time %.2lf", event.phase, m_currentTime);
            }
        }
    }

    if (vehcall_cmd != 0)
    {
        ITSAPP_LOG("send VehCall with: %d", vehcall_cmd);
        m_mib.IntersectionPhaseControl(VehCall, vehcall_cmd, 'Y');
    }

    if (forceoff_cmd != 0)
    {
        ITSAPP_LOG("send ForceOff with: %d", forceoff_cmd);
        m_mib.IntersectionPhaseControl(ForceOff, forceoff_cmd, 'Y');
    }

    if (hold_cmd != 0)
    {
        ITSAPP_LOG("send Hold with: %d", hold_cmd);
        m_mib.IntersectionPhaseControl(Hold, hold_cmd, 'Y');
    }

    if (pedcall_cmd != 0)
    {
        ITSAPP_LOG("send PedCall with: %d", pedcall_cmd);
        m_mib.IntersectionPhaseControl(PedCall, pedcall_cmd, 'Y');
    }

    if (pedclear_cmd != 0)
    {
        ITSAPP_LOG("send PedClear.");
        m_mib.IntersectionPhaseControl(PedCall, 0, 'Y');
    }

    bool doPhaseCtrl = false;
    if (m_eventListR1.size() > 0)
    {
        auto event1 = m_eventListR1.end() - 1;
        // already reach the end of the barrier, back to COP
        doPhaseCtrl = (timeDiff <= (event1->time + 2));
    }

    if (m_eventListR2.size() > 0)
    {
        auto event2 = m_eventListR2.end() - 1;
        doPhaseCtrl = (timeDiff <= event2->time + 2);
    }

    if (doPhaseCtrl && (pedcall_cmd == 0) && (pedclear_cmd == 0))
        m_mib.IntersectionPhaseControl(Omit, omit_cmd, 'Y');
}

void MmitssTrafficControllerInterface::tick()
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
                    sigc::mem_fun(*this, &MmitssTrafficControllerInterface::onTimer));
            }
            else
            {
                ITSAPP_TRACE("Setting interval %i", m_timerInterval);
                siekdbus::timerSetInterval(m_timerId, m_timerInterval);
                siekdbus::timerSetEnabled(m_timerId, true);
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

bool MmitssTrafficControllerInterface::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..");
    //    int curPlan = m_mib.CurTimingPlanRead();
    //    ITSAPP_TRACE("Current timing plan is: %d", curPlan);
    //
    //    // Generate: configinfo_XX.txt
    //    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getConfigInfoFilePath());
    m_cfg.ReadInConfig(SieMmitss::getConfigInfoFilePath());

    //-----------------Read in the ConfigIS Every time in case of changing
    // plan-----------------------//

    //    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getSignalConfigCOPFilePath());
    //    // Read configured ped walking time and clearance time
    //    m_mib.IntersectionPedConfigRead(curPlan, SieMmitss::getSignalConfigCOPFilePath());
    //
    // Read Signal Parameters for traffic control to phase_seq, MinGreen, MaxGreen, Yellow and Red
    if (!ReadSignalParameters())
        return false;
    ITSAPP_TRACE("Phase Sequence: %d %d %d %d %d %d %d %d", m_phaseSeq[0], m_phaseSeq[1],
        m_phaseSeq[2], m_phaseSeq[3], m_phaseSeq[4], m_phaseSeq[5], m_phaseSeq[6], m_phaseSeq[7]);

    for (int i = 4; i > 0; i--) // warm up...
    {
        m_mib.PhaseTimingStatusRead();
        m_phases.UpdatePhase(m_mib.getPhaseRead(), m_mib.getPhaseStatus());
        m_phases.Display();
        // Phases.RecordPhase(signal_plan_file);
        for (int j = 0; j < 2; j++)
            m_currPhase[j] = m_phases.CurPhase[j] + 1;

        ITSAPP_TRACE("Current phase is: %d %d", m_currPhase[0], m_currPhase[1]);

        if (CheckConflict() == 1)
        { // phase confliction
            ITSAPP_WRN("Phase conflict!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ModifyCurPhase();
            ITSAPP_TRACE("Current phase is: %d %d", m_currPhase[0], m_currPhase[1]);
        }

        std::this_thread::sleep_for(1000ms);
    }
    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssTrafficControllerInterface::storeEventTimeTableInDb()
{
    its::DatabaseListData eventTbl;
    for (int i = 0; i < 6; i++)
    {
        auto eventObj = std::make_shared<its::DatabaseListItem>();
        for (uint j = 0; j < 8; j++)
        {
            if (!eventObj->content["events"].empty())
                eventObj->content["events"] += ",";
            eventObj->content["events"] += std::to_string(m_eventTimeTable[i][j]);
        }
        eventTbl.emplace_back(eventObj);
    }
    ITSAPP_VERIFY(
        m_parent.database().replaceList("its/us/mmitss/control-if/event_table_list", eventTbl));

    ITSAPP_VERIFY(m_parent.database().write("its/us/mmitss/control-if/start_time", m_beginTimeUTC));
}

void MmitssTrafficControllerInterface::onSignalCtrlScheduleReceive(const void* data, size_t len)
{
    if (len == 0)
        return;

    m_beginTime = GetSeconds();

    auto curMinute = WaveApp::timeUtil::toMinuteOfTheYear(WaveApp::Clock::now());

    auto startHour = (curMinute / 60) % 24;
    auto startMinute = curMinute % 60;
    auto startSecond = (WaveApp::timeUtil::toDSecond(WaveApp::Clock::now()) / 1000) % 60;

    m_beginTimeUTC = std::to_string(startHour) + ":" + std::to_string(startMinute) + ":" +
                     std::to_string(startSecond);

    // Reset Hold timer
    m_holdTimerR1 = 0.0;
    m_holdTimerR2 = 0.0;

    m_newSchedule = 1; // new schedule is received
    // Re-create the event list
    m_eventListR1.clear();
    m_eventListR2.clear();
    // clear current arrival table
    for (auto& e : m_eventTimeTable)
        e.fill(0);
    UnpackEventData(reinterpret_cast<const char*>(data));
    ITSAPP_LOG("Received New Schedule at time %.2lf", m_beginTime);
    // print the new schedule
    for (auto& event : m_eventListR1)
    {
        ITSAPP_TRACE("Time: %lf Phase: %d Action: %d", event.time, event.phase, event.action);
        m_eventTimeTable[event.action][event.phase - 1] = event.time;
    }
    for (auto& event : m_eventListR2)
    {
        ITSAPP_TRACE("Time: %lf Phase: %d Action: %d", event.time, event.phase, event.action);
        m_eventTimeTable[event.action][event.phase - 1] = event.time;
    }

    storeEventTimeTableInDb();
}

int MmitssTrafficControllerInterface::CheckConflict()
{
    int conflict = 0;
    if (m_currPhase[0] == 1 || m_currPhase[0] == 2) // P11
    {
        if (m_currPhase[1] != 5 && m_currPhase[1] != 6)
            conflict = 1;
    }
    if (m_currPhase[0] == 3 || m_currPhase[0] == 4) // P11
    {
        if (m_currPhase[1] != 7 && m_currPhase[1] != 8)
            conflict = 1;
    }
    if (m_currPhase[0] == 5 || m_currPhase[0] == 6) // P11
    {
        if (m_currPhase[1] != 1 && m_currPhase[1] != 2)
            conflict = 1;
    }
    if (m_currPhase[0] == 7 || m_currPhase[0] == 8) // P11
    {
        if (m_currPhase[1] != 3 && m_currPhase[1] != 4)
            conflict = 1;
    }

    return conflict;
}

void MmitssTrafficControllerInterface::ModifyCurPhase()
{
    if (m_phaseSeq[m_currPhase[0] + 4 - 1] == 0) // missing phase is in ring 2
        m_currPhase[1] = m_currPhase[0] + 4;     // change phase in ring 2
    if (m_phaseSeq[m_currPhase[1] - 4 - 1] == 0) // missing phase is in ring 1
        m_currPhase[0] = m_currPhase[1] - 4;     // change phase in ring 1
}

void MmitssTrafficControllerInterface::UnpackEventData(const char* ablob)
{
    int offset = 0;
    long tempLong;
    unsigned char byteA; // force to unsigned this time,
    unsigned char byteB; // we do not want a bunch of sign extension
    unsigned char byteC; // math mucking up our combine logic
    unsigned char byteD;

    // Header
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    offset = offset + 2;
    // message id
    offset = offset + 1; // move past to next item

    // Do No. Event in R1
    int No_R1;
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    No_R1 = (byteA << 8) + byteB; // in fact unsigned
    offset = offset + 2;

    // Do each event in R1
    for (int i = 0; i < No_R1; i++)
    {
        Schedule TempSche;
        // do time
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempSche.time = (tempLong / DEG2ASNunits); // convert and store as float
        offset = offset + 4;
        // do phase
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempSche.phase = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;
        // do action
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempSche.action = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;
        m_eventListR1.emplace_back(TempSche); // add the new event to the list
    }

    // Do No. Event in R2
    int No_R2;
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    No_R2 = (byteA << 8) + byteB; // in fact unsigned
    offset = offset + 2;

    // Do each event in R1
    for (int i = 0; i < No_R2; i++)
    {
        Schedule TempSche;
        // do time
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempSche.time = (tempLong / DEG2ASNunits); // convert and store as float
        offset = offset + 4;
        // do phase
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempSche.phase = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;
        // do action
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempSche.action = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;
        m_eventListR2.emplace_back(TempSche); // add the new event to the list
    }
}

bool MmitssTrafficControllerInterface::ReadSignalParameters()
{
    fstream fs;
    fs.open(SieMmitss::getSignalConfigCOPFilePath());
    if (!fs.good())
    {
        ITSAPP_WRN("Error opening file %s", SieMmitss::getSignalConfigCOPFilePath().c_str());
        return false;
    }
    std::string temp_string;
    char temp[128];
    // read number of phases
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d", &m_phaseNum);
    // read phase sequence
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_phaseSeq[0], &m_phaseSeq[1], &m_phaseSeq[2],
        &m_phaseSeq[3], &m_phaseSeq[4], &m_phaseSeq[5], &m_phaseSeq[6], &m_phaseSeq[7]);
    fs.close();
    return true;
}

} // namespace Mmitss
} // namespace WaveApp
