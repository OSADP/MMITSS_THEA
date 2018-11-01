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
 */

/*
 * MMITSS MRP Traffic Controller Interface
 * This component is reponsible to receive signal timing schedule from
 * MRP_TrafficControl and MRP_PriorityRequestServer and send control
 * commands (NTCIP: FORCE_OFF, VEH_CALL, PHASE_OMIT, Hold) to
 * Econolite ASC3/Cobalt controller
 *
 * Created by: Yiheng Feng 10/28/2014
 *
 * Department of Systems and Industrial Engineering
 * University of Arizona
 */

#include "mmitssSignalControl.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <Poco/Net/SocketAddress.h>
#include <sigc++/functors/mem_fun.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iterator>
#include <memory>
#include <sstream>
#include <thread>

#include "facilityBase.hpp"
#include "facilityDatabaseAdapter.hpp"
#include "mmitssSignalControlApp.hpp"
#include "GetInfo.h"
#include "EVLS.h"
#include "SieMmitssConfig.hpp"

#ifndef DEG2ASNunits
#define DEG2ASNunits (1000000.0) // used for ASN 1/10 MICRO deg to unit converts
#endif

namespace WaveApp
{
namespace Mmitss
{

static const int RX_CP_BUF_SIZE = 1024;
static const int RX_TRAJECTORY_BUF_SIZE = 500000;

static const int MAX_ARRIVAL_TABLE_REPORT = 30;

MmitssSignalControl::MmitssSignalControl(MmitssSignalControlApp& parent)
    : m_parent(parent)
    , m_criticalPointsReceiver(SieMmitss::getSignalControlOPTPort(),
          RX_CP_BUF_SIZE,
          [this](const void* data, size_t len) { return this->onCriticalPointsReceive(data, len); })
    , m_trajectoryReceiver(SieMmitss::getSignalControlTrajectoryPort(),
          RX_TRAJECTORY_BUF_SIZE,
          [this](const void* data, size_t len) { return this->onTrajectoryReceive(data, len); })
    , m_occupancy(8)
    , m_cfg()
    , m_phases(m_cfg)
    , m_mib()
    , m_cop(*this)
{
}

bool MmitssSignalControl::onStartup()
{
    return mmitssInit();
}

void MmitssSignalControl::onShutdown()
{
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    m_criticalPointsReceiver.stop();
    m_trajectoryReceiver.stop();
}

void MmitssSignalControl::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

void MmitssSignalControl::resetArrivalTableInDb()
{
    auto& db = m_parent.database();
    ITSAPP_VERIFY(db.clearList("/its/us/mmitss/isig/arrival_table_list"));
}

bool MmitssSignalControl::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    // reset arrival table data
    resetArrivalTableInDb();

    auto peneration = 100;
    ITSAPP_VERIFY(m_parent.database().read("its/us/mmitss/control/penetration", peneration));

    m_penetrationRate = static_cast<float>(peneration) / 100;

    m_trackedVeh.clear();
    m_vehListEachPhase.clear();
    m_eventListR1.clear();
    m_eventListR2.clear();
    m_occupancy.clear();
    // clear current arrival table
    for (auto& e : m_arrivalTable)
        e.fill(0);

    // --- getting the configurations
    m_mib.setControllerAddress(get_ip_address());
    // Get the current RSU ID from "rsuid.txt" into string "m_rsuId"
    m_rsuId = get_rsu_id();
    //------------init: Begin of Network connection------------------------------------
    int broadcast = 1;

    if ((m_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        ITSAPP_ERROR("sockfd");
        return false;
    }

    if ((setsockopt(m_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))) == -1)
    {
        ITSAPP_ERROR("setsockopt - SO_SOCKET ");
        close(m_sockfd);
        return false;
    }

    m_recvAddr.sin_family = AF_INET;
    m_recvAddr.sin_port = htons(SieMmitss::getTrajectoryAwarePort());
    m_recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(m_recvAddr.sin_zero, '\0', sizeof m_recvAddr.sin_zero);

    //-----------------------End of Network Connection------------------//
    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });
    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);

    return true;
}

void MmitssSignalControl::onTimer(siekdbus::timerid_t id)
{
    (void)id;
    if (not m_started)
        return;

    m_currentTime = GetSeconds();

    m_mib.PhaseTimingStatusRead();
    m_phases.UpdatePhase(m_mib.getPhaseRead(), m_mib.getPhaseStatus());

    for (int i = 0; i < 2; i++)
    {
        m_initPhase[i] = m_phases.InitPhase[i] + 1; //{1-8}
        m_InitTime[i] = m_phases.InitTime[i]; // ALSO is the (Yellow+Red) Left for the counting down
                                              // time ***Important***
        m_grnElapse[i] = m_phases.GrnElapse[i]; // If in Green
        m_currPhase[i] = m_phases.CurPhase[i] +
                         1; // if in yellow or red, current phase will be different from Initphase
    }

    if (CheckConflict() == 1)
    { // phase conflict
        ITSAPP_WRN("Phase conflict!");
        ModifyCurPhase();
        ITSAPP_WRN("Current phase is: %d, %d", m_currPhase[0], m_currPhase[1]);
    }

    if (CheckConflict() == 1)
    { // phase conflict again
        ITSAPP_WRN("Phase conflict again!!");
        m_currPhase[0] = m_phases.InitPhase[0];
        m_currPhase[1] = m_phases.InitPhase[1];
    }

    for (int i = 0; i < 8; i++)
    { // Calculate red elapse time for each phase, if the signal turns to green,
        // keep the previous red duration until turns to red again
        if ((m_previousSignalColor[i] != 1) && (m_mib.getPhaseRead().phaseColor[i] == 1))
        { // signal changed from other to red
            m_redStartTime[i] = GetSeconds();
        }
        if (m_mib.getPhaseRead().phaseColor[i] == 1)
        {
            m_redElapseTime[i] = GetSeconds() - m_redStartTime[i];
        }
        m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
    }
    // Calculate previous phases
    for (int i = 0; i < 1; i++)
    {
        if (m_phases.CurPhase[i] != m_phases.PrePhase[i])
        {
            m_previousPhase[i] = m_phases.PrePhase[i] + 1;
            m_phases.PrePhase[i] = m_phases.CurPhase[i];
        }
    }

    if (m_controlFlag == 0) // solving COP
    {
        ITSAPP_LOG("Back to COP, %f, %f", m_grnElapse[0], m_grnElapse[1]);
        // COP starts here!!!!!!!!!!!!!!!!
        // Case 1: if the current signal status is yellow or red, wait until yellow or red ends
        if ((m_InitTime[0] != 0) && (m_InitTime[1] != 0))
            ITSAPP_TRACE("Yellow/Red interval, wait until %f to apply COP", m_InitTime[0]);

        // Case 2: if the current signal status is green, apply COP from the current phase
        if ((m_grnElapse[0] != 0) || (m_grnElapse[1] != 0))
        {
            // already passed min green time -6

            // if (m_grnElapse[0]>=ConfigIS.Gmin[InitPhase[0]-1]-6 ||
            // m_grnElapse[1]>=ConfigIS.Gmin[InitPhase[1]-1]-6)
            if ((m_grnElapse[0] > 0) || (m_grnElapse[1] > 0))
            { // green already start
                // request for trajectory data
                requestTrajectory();
            }
            else
            { // still red or yellow time
                // begintime=(int)currenttime-rolling_horizon+ceil(ConfigIS.Gmin[InitPhase[0]-1]-m_grnElapse[0])-1;
                ITSAPP_TRACE("Still within Min Green time, wait until %f to apply COP",
                    m_cfg.getConfig().Gmin[m_initPhase[0] - 1] - m_grnElapse[0] - 3);
            }
            m_beginTime = time(NULL);
            ITSAPP_LOG("m_beginTime set to time %d", m_beginTime);
        }
    }
    else if (m_controlFlag == 1)
    {
        m_currentTime = time(NULL);

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Changed here!!!!!!!!!!!!!!!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Soving every 5 seconds!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //~ if (currenttime-solving_timer>5)
        //~ {
        //~ solving_timer=currenttime;
        //~ control_flag=0;
        //~ cout<<"5 seconds passed, solve again!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        //~ }

        // Check ped call after the left turn phase
        if ((m_currentTime - m_beginTime) > m_check_TimePoint)
        {
            ITSAPP_TRACE("Reach check point");
            m_mib.PedStatusRead();
            if ((m_pedStatus == 0) && (m_mib.CheckPedPhaseInfo() == 1))
            { // previous no ped phase required, but current is
                ITSAPP_TRACE("Ped Status Changed during Left Turn Green, need to re-plan");
                // need to replan to include the ped request
                ITSAPP_TRACE("m_controlFlag = 0");
                m_controlFlag = 0;
            }
            else
                m_check_TimePoint = 9999;
        }

        if ((m_eventListR1.size() == 0) && (m_eventListR2.size() == 0))
        {
            if ((m_currentTime - m_beginTime) > 2)
            {
                m_controlFlag = 0;
                ITSAPP_TRACE("m_controlFlag = 0");
            }
        }
        else
        {
            auto event1 = m_eventListR1.end() - 1;
            auto event2 = m_eventListR2.end() - 1;
            ITSAPP_TRACE("m_currentTime is %d,m_beginTime %d", m_currentTime, m_beginTime);
            ITSAPP_TRACE("event1->time is %f,event2->time %f", event1->time, event2->time);
            if ((m_currentTime - m_beginTime) > (event1->time + 2) ||
                (m_currentTime - m_beginTime) > (event2->time + 2))
            { // already reach the end of the barrier, back to COP
                m_eventListR1.clear();
                m_eventListR2.clear();
                ITSAPP_LOG("End of Planning time, back to COP at time %d", m_currentTime);
                ITSAPP_LOG("m_controlFlag = 0");
                m_controlFlag = 0;
            }
        }
    }

#if 0
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Need to take out later
    //Put one vehicle to each phase in the arrival to make sure they will be served at least minimum time
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    for (i=0;i<8;i++)
    {
            ArrivalTable[0][i]+=5;
    }

    //find the passed phase in the same barrier
    Find_Passed_Phase_In_Same_Barrier(previous_phase,CurrPhase);

    //This calculating of passed phased is only applied to leading left turn of fixed phase sequence in the field!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    passed_phase[0]=0;
    passed_phase[1]=0;

    if(CurrPhase[0]==2)
        {passed_phase[0]=1;}
    if(CurrPhase[0]==4)
        {passed_phase[0]=3;}
    if(CurrPhase[1]==6)
        {passed_phase[1]=5;}
    if(CurrPhase[1]==8)
        {passed_phase[1]=7;}

#endif
}

void MmitssSignalControl::onCriticalPointsReceive(const void* data, size_t len)
{
    (void)data;
    (void)len;
    // TODO: critical point processing
    return;
}

void MmitssSignalControl::onTrajectoryReceive(const void* data, size_t len)
{
    if (len == 0)
        return;

    m_lastAckCount = m_reqCount;
    // clear the list for new set of data
    m_trackedVeh.clear();
    // unpack the trajectory data and save to trackedveh list
    UnpackTrajData1(reinterpret_cast<const char*>(data));

    // output of the signal data
    auto& phase_read = m_mib.getPhaseRead();
    ITSAPP_TRACE("phase_read is: %d %d %d %d %d %d %d %d", phase_read.phaseColor[0],
        phase_read.phaseColor[1], phase_read.phaseColor[2], phase_read.phaseColor[3],
        phase_read.phaseColor[4], phase_read.phaseColor[5], phase_read.phaseColor[6],
        phase_read.phaseColor[7]);
    ITSAPP_TRACE("The red elapse time for each phase is:");
    ITSAPP_TRACE("%f %f %f %f %f %f %f %f", m_redElapseTime[0], m_redElapseTime[1],
        m_redElapseTime[2], m_redElapseTime[3], m_redElapseTime[4], m_redElapseTime[5],
        m_redElapseTime[6], m_redElapseTime[7]);
    double cur_time = GetSeconds();

    // clear current arrival table
    for (auto& e : m_arrivalTable)
        e.fill(0);

    if (m_penetrationRate >= 0.95f)
    { // construct arrival table directly

        for (const auto& veh : m_trackedVeh)
        {
            if (veh.req_phase > 0) // Requested phase
            {
                if (veh.Speed > 1)
                {
                    int ETA = ceil(veh.stopBarDistance / veh.Speed);
                    if (ETA < 50)
                    {
                        m_arrivalTable[ETA][veh.req_phase - 1]++;
                    }
                }
                else
                    m_arrivalTable[0][veh.req_phase - 1]++;
            }
        }
    }
    else
    {
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // Algorithm to infer vehicle location and speed based on sample vehicle data and create
        // arrival table
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ITSAPP_TRACE("Estimating Location and Position of Unequipped Vehicle:");
        for (int i = 0; i < 8; i++)
        {
            // Assign vehicles of each phase to the vehlist_each_phase
            m_vehListEachPhase.clear();
            for (const auto& veh : m_trackedVeh)
            {
                if (veh.req_phase == i + 1) // assign to the same phase
                {
                    m_vehListEachPhase.emplace_back(veh);
                }
            }

            ITSAPP_TRACE("Phase: %d", i + 1);

            // sprintf(temp_log,"The input list is:\n");
            for (const auto& veh : m_vehListEachPhase)
            {
                ITSAPP_TRACE("%d %lf %lf %lf %d %d %d %lf\n", veh.TempID, veh.Speed,
                    veh.stopBarDistance, veh.acceleration, veh.approach, veh.lane, veh.req_phase,
                    veh.time_stop);
            }

            ITSAPP_TRACE("The number of lanes is: %d", m_laneNo[i]);

            if (m_vehListEachPhase.size() != 0) // There is vehicle in the list
                EVLS_Isig(m_vehListEachPhase, m_trackedVeh, i + 1, cur_time, m_redElapseTime[i],
                    m_laneNo[i], m_dsrcRange, m_penetrationRate, m_arrivalTable);
        }
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Need to take out later
    // Put one vehicle to each phase in the arrival to make sure they will be served at least
    // minimum time
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /*for(int i = 0;i < 8; i++)
    {
        m_arrivalTable[0][i] += 5;
    }*/

    // read arrival table from file
    /*
    fstream fs;
    fs.open(arrivaltablefile,ios::in);
    string line;
    for(i=0;i<131;i++)
    {
        getline(fs,line);
        //sprintf(temp_log,line.c_str());
        //outputlog(temp_log);
        sscanf(line.c_str(),"%d %d %d %d %d %d %d
    %d",&ArrivalTable[i][0],&ArrivalTable[i][1],&ArrivalTable[i][2],&ArrivalTable[i][3],&ArrivalTable[i][4],&ArrivalTable[i][5],&ArrivalTable[i][6],&ArrivalTable[i][7]);
    }
    fs.close();
    */

    // calculate skipped phase based on ELVS
    int temp_arr = 0;
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 131; j++)
            temp_arr += m_arrivalTable[j][i];
        // make sure the current phase is not skipped, added by YF: 12/08/2014
        if ((temp_arr == 0) && ((i + 1) != m_currPhase[0]) && ((i + 1) != m_currPhase[1]))
            m_skipPhase[i] = 0; // skip the phase
        else
            m_skipPhase[i] = 1; // don't skip the phase
        temp_arr = 0;
    }

    // Check skip phase's detector and see whether they have calls
    // (useful for left turn phases under low penetration rate)
    m_mib.DetReadOccupancy(m_occupancy);
    ITSAPP_TRACE("Detector Occupancy: %d %d %d %d %d %d %d %d", m_occupancy[0], m_occupancy[1],
        m_occupancy[2], m_occupancy[3], m_occupancy[4], m_occupancy[5], m_occupancy[6],
        m_occupancy[7]);

    for (int i = 0; i < 8; i++)
    {
        if ((m_skipPhase[i] == 0) && (m_occupancy[i] > 0))
        {                       // no demand from connected vehicle, but the detector is occupied
            m_skipPhase[i] = 1; // Do not skip phase
            m_arrivalTable[0][i] = 1; // Add one vehicle to the arrival table at queue
            ITSAPP_TRACE("No connected vehicle data but vehicle present at "
                         "the detector for phase %d",
                i + 1);
        }
    }
    // Complete the barrier by adding missing phases (if the phase is not in use)
    Complete_Barrier();

    storeArrivalTableInDb();

    if (m_controlFlag == 0)
    {
        ITSAPP_TRACE("phase_read is: %d %d %d %d %d %d %d %d", phase_read.phaseColor[0],
            phase_read.phaseColor[1], phase_read.phaseColor[2], phase_read.phaseColor[3],
            phase_read.phaseColor[4], phase_read.phaseColor[5], phase_read.phaseColor[6],
            phase_read.phaseColor[7]);
        ITSAPP_TRACE("Initphase is %d %d; InitTime is %f %f; grnElapse is %f %f; CurPhase is %d %d",
            m_initPhase[0], m_initPhase[1], m_InitTime[0], m_InitTime[1], m_grnElapse[0],
            m_grnElapse[1], m_currPhase[0], m_currPhase[1]);
        ITSAPP_TRACE("previous phase is %d %d", m_previousPhase[0], m_previousPhase[1]);
        // find the passed phase in the same barrier
        Find_Passed_Phase_In_Same_Barrier();

        // This calculation of passed phased is only applied to leading left turn of
        // fixed phase sequence in the field
        /*passed_phase[0]=0;
        passed_phase[1]=0;

        if(CurrPhase[0]==2)
            {passed_phase[0]=1;}
        if(CurrPhase[0]==4)
            {passed_phase[0]=3;}
        if(CurrPhase[1]==6)
            {passed_phase[1]=5;}
        if(CurrPhase[1]==8)
            {passed_phase[1]=7;}*/

        //////////////////////////////////////////////////////////////////

        ITSAPP_TRACE("Green interval, start COP:");
        ITSAPP_TRACE("The Passed phases are %d %d", m_passedPhase[0], m_passedPhase[1]);
        ITSAPP_TRACE("The skipped phases are %d %d %d %d %d %d %d %d", m_skipPhase[0],
            m_skipPhase[1], m_skipPhase[2], m_skipPhase[3], m_skipPhase[4], m_skipPhase[5],
            m_skipPhase[6], m_skipPhase[7]);

        // skip_phase_sum=0;  //total number of skip phases
        // for(i=0;i<8;i++)
        //{
        //  skip_phase_sum+=skip_phase[i];
        //}

        m_conflictFlag = ConflictCall();

        // Set queue warning flag
        //~ if(ArrivalTable[0][0]>=9 )  //Phase 1 have very long queue
        //~ {
        //~ queue_warning_P1=1;
        //~ sprintf(temp_log,"Queue warning!!!!!!!!!!! need to serve left turn ASAP");
        //~ outputlog(temp_log); cout<<temp_log;
        //~ }
        //~ else
        //~ queue_warning_P1=0;
        //~
        //~ if(ArrivalTable[0][4]>=9 )  //phase 5 have very long queue
        //~ {
        //~ queue_warning_P5=1;
        //~ sprintf(temp_log,"Queue warning!!!!!!!!!!! need to serve left turn ASAP");
        //~ outputlog(temp_log); cout<<temp_log;
        //~ }
        //~ else
        //~ queue_warning_P5=0;

        if (m_conflictFlag == 0)
        { // no any vehicle arrival at conflict phase
            ITSAPP_LOG("No vehicle arrival at conflict phase, Wait... ");
            // msleep(1000);  //wait for one second and check again
        }
        else
        { // has at least one vehicle arrival at conflict phase
            //---time stamps used to calculate the time needed for COP
            double t_1 = GetSeconds();
            double t_2;

            int int_GrnElapse[2];
            // int_GrnElapse[0]= min(MinGreen[CurrPhase[0]-1],(int) m_grnElapse[0]);
            // int_GrnElapse[1]= min(MinGreen[CurrPhase[1]-1],(int) m_grnElapse[1]);

            int_GrnElapse[0] = (int)m_grnElapse[0];
            int_GrnElapse[1] = (int)m_grnElapse[1];

            ITSAPP_LOG("The int green elapse time is %d %d", int_GrnElapse[0], int_GrnElapse[1]);

            // Check Ped Call and Current Ped Phase,
            // return which ped phases need to be considered
            m_mib.PedStatusRead();
            m_pedStatus = m_mib.CheckPedPhaseInfo();
            // m_pedStatus = Check_Ped_Phase_Info();
            ITSAPP_TRACE("Ped_Status is: %d", m_pedStatus);
            // Before solving COP, read the signal parameters again, in case COP changes the min
            // green and max green time
            ReadSignalParameters();

            // run ring barrier COP algorithm
            m_cop.COP_DUAL_RING(m_initPhase, int_GrnElapse, m_passedPhase, m_skipPhase);
            t_2 = GetSeconds();
            auto cop_solve_time = (int)t_2 - t_1 + 1;
            ITSAPP_LOG("The time for COP is: %lf second", cop_solve_time);

            // solving_timer=currenttime;
            ITSAPP_LOG("The optimal phase sequences are:");
            ITSAPP_LOG("Ring 1: %d %d %d %d", m_optSigPlan[0][0], m_optSigPlan[0][1],
                m_optSigPlan[0][2], m_optSigPlan[0][3]);
            ITSAPP_LOG("Ring 2: %d %d %d %d\n", m_optSigPlan[1][0], m_optSigPlan[1][1],
                m_optSigPlan[1][2], m_optSigPlan[1][3]);
            ITSAPP_LOG("Optimal Signal Sequence is:");
            ITSAPP_LOG("Ring 1: %d %d %d %d", m_optSigSeq[0][0], m_optSigSeq[0][1],
                m_optSigSeq[0][2], m_optSigSeq[0][3]);
            ITSAPP_LOG("Ring 2: %d %d %d %d", m_optSigSeq[1][0], m_optSigSeq[1][1],
                m_optSigSeq[1][2], m_optSigSeq[1][3]);

            // Setup the check time point
            if (m_optSigPlan[0][0] != 0 || m_optSigPlan[1][0] != 0)
                m_check_TimePoint =
                    max(m_optSigPlan[0][0], m_optSigPlan[1][0]) - cop_solve_time + 2;
            else
                m_check_TimePoint = 9999;

            // construct the control Eventlist

            m_eventListR1.clear();
            m_eventListR2.clear();
            Construct_eventlist();

            for (auto& event : m_eventListR1)
            {
                event.time -= cop_solve_time;
                if (event.time < 0)
                    event.time = 0;
                ITSAPP_TRACE(
                    "Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
            }
            for (auto& event : m_eventListR2)
            {
                event.time -= cop_solve_time;
                if (event.time < 0)
                    event.time = 0;
                ITSAPP_TRACE(
                    "Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
            }

            m_controlFlag = 1;
            ITSAPP_LOG("m_controlFlag = 1");

            // Pack the Event_list and Send to the SignalControlInterface  Added by YF: 10/28/2014
            if (m_eventListR1.size() != 0 || m_eventListR2.size() != 0)
            {
                uint8_t tmp_event_data[500];
                int size = 0;
                Pack_Event_List(tmp_event_data, size);
                // Send trajectory data
                ITSAPP_LOG(
                    "Send Signal Control Event to SignalControllerInterface, The size is %d", size);
                m_recvAddr.sin_port = htons(SieMmitss::getTrafficControllerInterfacePort());
                sendto(m_sockfd, tmp_event_data, size + 1, 0, (struct sockaddr*)&m_recvAddr,
                    sizeof m_recvAddr);
            }
        }
    }
}

void MmitssSignalControl::storeArrivalTableInDb()
{
    its::DatabaseListData arrivalTbl;
    for (int i = 0; i < MAX_ARRIVAL_TABLE_REPORT; i++)
    {
        auto arrivalObj = std::make_shared<its::DatabaseListItem>();
        for (uint j = 0; j < m_arrivalTable[i].size(); j++)
        {
            if (!arrivalObj->content["arrivals"].empty())
                arrivalObj->content["arrivals"] += ",";
            arrivalObj->content["arrivals"] += std::to_string(m_arrivalTable[i][j]);
        }
        arrivalTbl.emplace_back(arrivalObj);
    }
    ITSAPP_VERIFY(
        m_parent.database().replaceList("/its/us/mmitss/isig/arrival_table_list", arrivalTbl));
}

void MmitssSignalControl::requestTrajectory()
{
    std::ostringstream reqStr;
    reqStr << "Request signal_control ";
    reqStr << m_reqCount;

    std::string req = reqStr.str();
    if (m_reqCount - m_lastAckCount < MAX_UNACK_REQ)
    {
        ITSAPP_TRACE("Send: %s", req.c_str());
        // send requested
        m_recvAddr.sin_port = htons(SieMmitss::getTrajectoryAwarePort());
        sendto(m_sockfd, req.c_str(), req.length(), 0, (struct sockaddr*)&m_recvAddr,
            sizeof(m_recvAddr));
    }
    else
    {
        ITSAPP_TRACE("restart receiver");
        // restart receiver
        m_trajectoryReceiver.stop();
        m_trajectoryReceiver.start();
        m_lastAckCount = m_reqCount;
    }
    m_reqCount++;
}

void MmitssSignalControl::tick()
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
                    sigc::mem_fun(*this, &MmitssSignalControl::onTimer));
            }
            else
            {
                ITSAPP_TRACE("Setting interval %i", m_timerInterval);
                siekdbus::timerSetInterval(m_timerId, m_timerInterval);
                siekdbus::timerSetEnabled(m_timerId, true);
            }
            if (m_trajectoryReceiver.start() == false)
            {
                ITSAPP_ERROR("m_trajectoryReceiver started failed.");
                return;
            }
            if (m_criticalPointsReceiver.start() == false)
            {
                ITSAPP_ERROR("m_criticalPointsReceiver started failed.");
                return;
            }
            ITSAPP_TRACE("Started successfully..");
            m_started = true;
        }
    }
    ITSAPP_TRACE("tick end..started: %d (0: not started yet, 1: started)", m_started);
}

bool MmitssSignalControl::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..");
    // Get number of lanes each phase
    if (!get_lane_no())
        return false;
    ITSAPP_TRACE("Number of lanes of each phase is: %d %d %d %d %d %d %d %d", m_laneNo[0],
        m_laneNo[1], m_laneNo[2], m_laneNo[3], m_laneNo[4], m_laneNo[5], m_laneNo[6], m_laneNo[7]);

    //-----------------Read in the ConfigIS Every time in case of changing
    // plan-----------------------//
    auto curPlan = m_mib.CurTimingPlanRead();
    ITSAPP_TRACE("Current timing plan is:\t%d", curPlan);
    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getSignalConfigCOPFilePath());
    // Read configured ped walking time and clearance time
    m_mib.IntersectionPedConfigRead(curPlan, SieMmitss::getSignalConfigCOPFilePath());
    // Read Signal Parameters for traffic control to phase_seq, MinGreen, MaxGreen, Yellow and Red
    if (!ReadSignalParameters())
        return false;

    // Generate: configinfo_XX.txt
    //    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getConfigInfoFilePath());
    // Find the missing phases of the signal table, save to ConfigIS
    //    PrintPlan2Log(SieMmitss::getConfigInfoFilePath());
    if (!m_cfg.ReadInConfig(SieMmitss::getConfigInfoFilePath()))
        return false;
    m_cfg.PrintRSUConfig();
    for (int i = 4; i > 0; i--) // warm up...
    {
        m_mib.PhaseTimingStatusRead();
        m_phases.UpdatePhase(m_mib.getPhaseRead(), m_mib.getPhaseStatus());
        m_phases.Display();
        // Phases.RecordPhase(signal_plan_file);
        std::this_thread::sleep_for(1s);
    }
    m_phases.PrePhase[0] = 0;
    m_phases.PrePhase[1] = 0;

    m_previousPhase[0] = 0;
    m_previousPhase[1] = 0;

    m_conflictFlag = 0;
    m_controlFlag = 0;

    // Record current red elapse time information
    // Notes: since COP always runs at the beginning of the green time (2 phases),
    // actually we should calculate the duration of previous red phase for the queue length
    // estimation
    // for the 2 phases just turned red, the red duration time could be 0
    // (or in actually calculation they are very small)
    // for other four phases, just in the middle of red duration

    // initialize red start time
    for (int i = 0; i < 8; i++)
    {
        m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
        if (m_mib.getPhaseRead().phaseColor[i] == 1)
            m_redStartTime[i] = GetSeconds();
        else
            m_redStartTime[i] = 0;
    }
    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssSignalControl::Construct_eventlist()
{
    Schedule Temp_event;

    if (m_currPhase[0] == 1)
    {
        if (m_optSigPlan[0][1] != 0)
        {
            // hold phase 1
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 1;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            // call phase 2
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 2;
            Temp_event.action = VehCall;
            m_eventListR1.emplace_back(Temp_event);
            // force off phase 1
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 1;
            Temp_event.action = ForceOff;
            m_eventListR1.emplace_back(Temp_event);
            // hold phase 2
            Temp_event.time = m_optSigPlan[0][0] + m_yellow[0] + m_red[0] + m_optSigPlan[0][1];
            Temp_event.phase = 2;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {

                // call phase in next barrier: 3 or 4
                Temp_event.time = m_optSigPlan[0][0] + m_yellow[0] + m_red[0] + m_optSigPlan[0][1];
                if (m_optSigPlan[0][2] != 0 &&
                    m_optSigPlan[0][3] != 0) // call the first phase in sequence
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 3;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 4;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);

                // force off phase 2
                Temp_event.time = m_optSigPlan[0][0] + m_yellow[0] + m_red[0] + m_optSigPlan[0][1];
                Temp_event.phase = 2;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 1
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 1;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 3 or 4
                Temp_event.time = m_optSigPlan[0][0];
                if (m_optSigPlan[0][2] != 0 &&
                    m_optSigPlan[0][3] != 0) // call the first phase in sequence
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 3;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 4;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 1
                Temp_event.time = m_optSigPlan[0][0];
                Temp_event.phase = 1;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[0] == 2)
    {
        if (m_optSigPlan[0][0] != 0)
        {
            // hold phase 2
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 2;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            // call phase 1
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 1;
            Temp_event.action = VehCall;
            m_eventListR1.emplace_back(Temp_event);
            // force off phase 2
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 2;
            Temp_event.action = ForceOff;
            m_eventListR1.emplace_back(Temp_event);
            // hold phase 1
            Temp_event.time = m_optSigPlan[0][1] + m_yellow[1] + m_red[1] + m_optSigPlan[0][0];
            Temp_event.phase = 1;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 3 or 4
                Temp_event.time = m_optSigPlan[0][1] + m_yellow[1] + m_red[1] + m_optSigPlan[0][0];
                if (m_optSigPlan[0][2] != 0 &&
                    m_optSigPlan[0][3] != 0) // call the first phase in sequence
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 3;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 4;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 1
                Temp_event.time = m_optSigPlan[0][1] + m_yellow[1] + m_red[1] + m_optSigPlan[0][0];
                Temp_event.phase = 1;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 2
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 2;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 3 or 4
                Temp_event.time = m_optSigPlan[0][1];
                if (m_optSigPlan[0][2] != 0 &&
                    m_optSigPlan[0][3] != 0) // call the first phase in sequence
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 3;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 4;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 2
                Temp_event.time = m_optSigPlan[0][1];
                Temp_event.phase = 2;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[0] == 3)
    {
        if (m_optSigPlan[0][1] != 0)
        {
            // hold phase 3
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 3;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            // call phase 4
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 4;
            Temp_event.action = VehCall;
            m_eventListR1.emplace_back(Temp_event);
            // force off phase 3
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 3;
            Temp_event.action = ForceOff;
            m_eventListR1.emplace_back(Temp_event);
            // hold phase 4
            Temp_event.time = m_optSigPlan[0][0] + m_yellow[2] + m_red[2] + m_optSigPlan[0][1];
            Temp_event.phase = 4;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 1 or 2
                Temp_event.time = m_optSigPlan[0][0] + m_yellow[2] + m_red[2] + m_optSigPlan[0][1];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 1;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 2;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 4
                Temp_event.time = m_optSigPlan[0][0] + m_yellow[2] + m_red[2] + m_optSigPlan[0][1];
                Temp_event.phase = 4;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 3
            Temp_event.time = m_optSigPlan[0][0];
            Temp_event.phase = 3;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 1 or 2
                Temp_event.time = m_optSigPlan[0][0];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 1;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 2;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 3
                Temp_event.time = m_optSigPlan[0][0];
                Temp_event.phase = 3;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[0] == 4)
    {
        if (m_optSigPlan[0][0] != 0)
        {
            // hold phase 4
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 4;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            // call phase 3
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 3;
            Temp_event.action = VehCall;
            m_eventListR1.emplace_back(Temp_event);
            // force off phase 4
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 4;
            Temp_event.action = ForceOff;
            m_eventListR1.emplace_back(Temp_event);
            // hold phase 3
            Temp_event.time = m_optSigPlan[0][1] + m_yellow[3] + m_red[3] + m_optSigPlan[0][0];
            Temp_event.phase = 3;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 1 or 2
                Temp_event.time = m_optSigPlan[0][1] + m_yellow[3] + m_red[3] + m_optSigPlan[0][0];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 1;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 2;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 3
                Temp_event.time = m_optSigPlan[0][1] + m_yellow[3] + m_red[3] + m_optSigPlan[0][0];
                Temp_event.phase = 3;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 4
            Temp_event.time = m_optSigPlan[0][1];
            Temp_event.phase = 4;
            Temp_event.action = Hold;
            m_eventListR1.emplace_back(Temp_event);
            if (m_optSigPlan[0][2] != 0 || m_optSigPlan[0][3] != 0)
            {
                // call phase in next barrier: 1 or 2
                Temp_event.time = m_optSigPlan[0][1];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = m_optSigSeq[0][2];
                if (m_optSigPlan[0][2] != 0 && m_optSigPlan[0][3] == 0)
                    Temp_event.phase = 1;
                if (m_optSigPlan[0][2] == 0 && m_optSigPlan[0][3] != 0)
                    Temp_event.phase = 2;
                Temp_event.action = VehCall;
                m_eventListR1.emplace_back(Temp_event);
                // force off phase 4
                Temp_event.time = m_optSigPlan[0][1];
                Temp_event.phase = 4;
                Temp_event.action = ForceOff;
                m_eventListR1.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[1] == 5)
    {
        if (m_optSigPlan[1][1] != 0)
        {
            // hold phase 5
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 5;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            // call phase 6
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 6;
            Temp_event.action = VehCall;
            m_eventListR2.emplace_back(Temp_event);
            // force off phase 5
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 5;
            Temp_event.action = ForceOff;
            m_eventListR2.emplace_back(Temp_event);
            // hold phase 6
            Temp_event.time = m_optSigPlan[1][0] + m_yellow[4] + m_red[4] + m_optSigPlan[1][1];
            Temp_event.phase = 6;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 7 or 8
                Temp_event.time = m_optSigPlan[1][0] + m_yellow[4] + m_red[4] + m_optSigPlan[1][1];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 7;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 8;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 6
                Temp_event.time = m_optSigPlan[1][0] + m_yellow[4] + m_red[4] + m_optSigPlan[1][1];
                Temp_event.phase = 6;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 5
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 5;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 7 or 8
                Temp_event.time = m_optSigPlan[1][0];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 7;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 8;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 5
                Temp_event.time = m_optSigPlan[1][0];
                Temp_event.phase = 5;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[1] == 6)
    {
        if (m_optSigPlan[1][0] != 0)
        {
            // hold phase 6
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 6;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            // call phase 5
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 5;
            Temp_event.action = VehCall;
            m_eventListR2.emplace_back(Temp_event);
            // force off phase 6
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 6;
            Temp_event.action = ForceOff;
            m_eventListR2.emplace_back(Temp_event);
            // hold phase 5
            Temp_event.time = m_optSigPlan[1][1] + m_yellow[5] + m_red[5] + m_optSigPlan[1][0];
            Temp_event.phase = 5;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 3 or 4
                Temp_event.time = m_optSigPlan[1][1] + m_yellow[5] + m_red[5] + m_optSigPlan[1][0];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 7;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 8;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 5
                Temp_event.time = m_optSigPlan[1][1] + m_yellow[5] + m_red[5] + m_optSigPlan[1][0];
                Temp_event.phase = 5;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 6
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 6;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 7 or 8
                Temp_event.time = m_optSigPlan[1][1];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 7;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 8;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 6
                Temp_event.time = m_optSigPlan[1][1];
                Temp_event.phase = 6;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[1] == 7)
    {
        if (m_optSigPlan[1][1] != 0)
        {
            // hold phase 7
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 7;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            // call phase 8
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 8;
            Temp_event.action = VehCall;
            m_eventListR2.emplace_back(Temp_event);
            // force off phase 7
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 7;
            Temp_event.action = ForceOff;
            m_eventListR2.emplace_back(Temp_event);
            // hold phase 8
            Temp_event.time = m_optSigPlan[1][0] + m_yellow[6] + m_red[6] + m_optSigPlan[1][1];
            Temp_event.phase = 8;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 5 or 6
                Temp_event.time = m_optSigPlan[1][0] + m_yellow[6] + m_red[6] + m_optSigPlan[1][1];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 5;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 6;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 8
                Temp_event.time = m_optSigPlan[1][0] + m_yellow[6] + m_red[6] + m_optSigPlan[1][1];
                Temp_event.phase = 8;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 7
            Temp_event.time = m_optSigPlan[1][0];
            Temp_event.phase = 7;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 5 or 6
                Temp_event.time = m_optSigPlan[1][0];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 5;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 6;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 7
                Temp_event.time = m_optSigPlan[1][0];
                Temp_event.phase = 7;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
    }

    if (m_currPhase[1] == 8)
    {
        if (m_optSigPlan[1][0] != 0)
        {
            // hold phase 8
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 8;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            // call phase 7
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 7;
            Temp_event.action = VehCall;
            m_eventListR2.emplace_back(Temp_event);
            // force off phase 8
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 8;
            Temp_event.action = ForceOff;
            m_eventListR2.emplace_back(Temp_event);
            // hold phase 7
            Temp_event.time = m_optSigPlan[1][1] + m_yellow[7] + m_red[7] + m_optSigPlan[1][0];
            Temp_event.phase = 7;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 5 or 6
                Temp_event.time = m_optSigPlan[1][1] + m_yellow[7] + m_red[7] + m_optSigPlan[1][0];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 5;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 6;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 7
                Temp_event.time = m_optSigPlan[1][1] + m_yellow[7] + m_red[7] + m_optSigPlan[1][0];
                Temp_event.phase = 7;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
        else
        {
            // hold phase 8
            Temp_event.time = m_optSigPlan[1][1];
            Temp_event.phase = 8;
            Temp_event.action = Hold;
            m_eventListR2.emplace_back(Temp_event);
            if (m_optSigPlan[1][2] != 0 || m_optSigPlan[1][3] != 0)
            {
                // call phase in next barrier: 5 or 6
                Temp_event.time = m_optSigPlan[1][1];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = m_optSigSeq[1][2];
                if (m_optSigPlan[1][2] != 0 && m_optSigPlan[1][3] == 0)
                    Temp_event.phase = 5;
                if (m_optSigPlan[1][2] == 0 && m_optSigPlan[1][3] != 0)
                    Temp_event.phase = 6;
                Temp_event.action = VehCall;
                m_eventListR2.emplace_back(Temp_event);
                // force off phase 8
                Temp_event.time = m_optSigPlan[1][1];
                Temp_event.phase = 8;
                Temp_event.action = ForceOff;
                m_eventListR2.emplace_back(Temp_event);
            }
        }
    }
}

void MmitssSignalControl::Pack_Event_List(uint8_t* tmp_event_data, int& size)
{
    //    int i,j;
    int offset = 0;
    uint8_t* pByte; // pointer used (by cast)to get at each byte
                    // of the shorts, longs, and blobs
                    //    byte    tempByte;   // values to hold data once converted to final format
    unsigned short tempUShort;
    long tempLong;
    // header 2 bytes
    tmp_event_data[offset] = 0xFF;
    offset += 1;
    tmp_event_data[offset] = 0xFF;
    offset += 1;
    // MSG ID: 0x03 for signal event data send to Signal Control Interface
    tmp_event_data[offset] = 0x03;
    offset += 1;
    // No. events in R1
    int No_Event_R1 = m_eventListR1.size();
    tempUShort = (unsigned short)No_Event_R1;
    pByte = (uint8_t*)&tempUShort;
    tmp_event_data[offset + 0] = *(pByte + 1);
    tmp_event_data[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    // Events in R1
    for (auto& event : m_eventListR1)
    {
        // Time
        tempLong = (long)(event.time * DEG2ASNunits);
        pByte = (uint8_t*)&tempLong;
        tmp_event_data[offset + 0] = *(pByte + 3);
        tmp_event_data[offset + 1] = *(pByte + 2);
        tmp_event_data[offset + 2] = *(pByte + 1);
        tmp_event_data[offset + 3] = *(pByte + 0);
        offset = offset + 4;
        // phase
        tempUShort = (unsigned short)event.phase;
        pByte = (uint8_t*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        // action
        tempUShort = (unsigned short)event.action;
        pByte = (uint8_t*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        //        m_eventListR1.Next();
    }
    // No. events in R2
    int No_Event_R2 = m_eventListR2.size();
    tempUShort = (unsigned short)No_Event_R2;
    pByte = (uint8_t*)&tempUShort;
    tmp_event_data[offset + 0] = *(pByte + 1);
    tmp_event_data[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    // Events in R1
    for (auto& event : m_eventListR2)
    {
        // Time
        tempLong = (long)(event.time * DEG2ASNunits);
        pByte = (uint8_t*)&tempLong;
        tmp_event_data[offset + 0] = *(pByte + 3);
        tmp_event_data[offset + 1] = *(pByte + 2);
        tmp_event_data[offset + 2] = *(pByte + 1);
        tmp_event_data[offset + 3] = *(pByte + 0);
        offset = offset + 4;
        // phase
        tempUShort = (unsigned short)event.phase;
        pByte = (uint8_t*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        // action
        tempUShort = (unsigned short)event.action;
        pByte = (uint8_t*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        //        m_eventListR2.Next();
    }
    size = offset;
}

void MmitssSignalControl::Find_Passed_Phase_In_Same_Barrier()
{
    // default: no passed phase
    m_passedPhase[0] = 0;
    m_passedPhase[1] = 0;
    for (int i = 0; i < 2; i++) // 2 rings
    {
        if ((m_previousPhase[i] == (1 + i * 4)) && (m_currPhase[i] == (2 + i * 4)))
            m_passedPhase[i] = 1 + i * 4;
        if ((m_previousPhase[i] == (2 + i * 4)) && (m_currPhase[i] == (1 + i * 4)))
            m_passedPhase[i] = 2 + i * 4;
        if ((m_previousPhase[i] == (3 + i * 4)) && (m_currPhase[i] == (4 + i * 4)))
            m_passedPhase[i] = 3 + i * 4;
        if ((m_previousPhase[i] == (4 + i * 4)) && (m_currPhase[i] == (3 + i * 4)))
            m_passedPhase[i] = 4 + i * 4;

        //  if(passed_phase[i]!=0)
        //  m_passedPhase[i]++;   //changed to phase number [1-8]
    }
}

int MmitssSignalControl::ConflictCall()
{
    for (int i = 0; i < 8; i++)
    {
        if (m_skipPhase[i] == 1) // this phase has vehicle
            if (((i + 1) != m_currPhase[0]) && ((i + 1) != m_currPhase[1]))
                return 1;
    }
    return 0;
}

void MmitssSignalControl::UnpackTrajData1(const char* ablob)
{
    int No_Veh;
    //    int i,j;
    int offset = 0;

    long tempLong;
    uint8_t byteA; // force to unsigned this time,
    uint8_t byteB; // we do not want a bunch of sign extension
    uint8_t byteC; // math mucking up our combine logic
    uint8_t byteD;

    // Header
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    //    int temp = (byteA << 8) + byteB; // in fact unsigned
    offset = offset + 2;

    // cout<<temp<<endl;

    // id
    //    temp = (char)ablob[offset];
    offset = offset + 1; // move past to next item
    // cout<<temp<<endl;

    // Do vehicle number
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    No_Veh = (byteA << 8) + byteB; // in fact unsigned
    offset = offset + 2;

    // cout<<No_Veh<<endl;

    // Do each vehicle
    for (int i = 0; i < No_Veh; i++)
    {
        ConnectedVehicle TempVeh;
        // Do Veh ID
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempVeh.TempID = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;

        // do speed
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.Speed = (tempLong / DEG2ASNunits); // convert and store as float
        offset = offset + 4;

        // do distance to stop bar
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.stopBarDistance = (tempLong / DEG2ASNunits); // convert and store as float
        offset = offset + 4;

        // do acceleration
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.acceleration = (tempLong / DEG2ASNunits); // convert and store as float
        offset = offset + 4;

        // do approach
        TempVeh.approach = (char)ablob[offset];
        offset = offset + 1; // move past to next item

        // do lane
        TempVeh.lane = (char)ablob[offset];
        offset = offset + 1; // move past to next item

        // do req_phase
        TempVeh.req_phase = (char)ablob[offset];
        offset = offset + 1; // move past to next item

        // do stop time
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        byteC = ablob[offset + 2];
        byteD = ablob[offset + 3];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.time_stop = (tempLong); // convert and store as float
        offset = offset + 4;

        // cout<<"E_Offset["<<j<<"] is:"<<TempVeh.E_Offset[j]<<endl;
        // cout<<"Done with one vehicle"<<endl;

        // Here reflects the penetration rate!!!!!!!!!!!!!!!!!!!!!!!!!!! Add only half of the
        // vehicles
        //  if (penetration>0.99)  //%100 penetration rate
        //  {
        m_trackedVeh.emplace_back(TempVeh);
        //  }
        //  if (penetration>0.74 && penetration<0.76) //75% penetration rate
        //  {
        //      if (TempVeh.TempID%4==3 || TempVeh.TempID%4==1 ||TempVeh.TempID%4==2)
        //          trackedveh.InsertRear(TempVeh);
        //  }
        //  if (penetration>0.49 && penetration<0.51)   //50% penetration rate
        //  {
        //      if (TempVeh.TempID%2==1)
        //          trackedveh.InsertRear(TempVeh);
        //  }
        //  if (penetration>0.24 && penetration<0.26)   //25% penetration rate
        //  {
        //      if (TempVeh.TempID%4==1)
        //          trackedveh.InsertRear(TempVeh);
        //  }
    }
}

int MmitssSignalControl::CheckConflict()
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

void MmitssSignalControl::ModifyCurPhase()
{
    if (m_phaseSeq[m_currPhase[0] + 4 - 1] == 0) // missing phase is in ring 2
        m_currPhase[1] = m_currPhase[0] + 4;     // change phase in ring 2
    if (m_phaseSeq[m_currPhase[1] - 4 - 1] == 0) // missing phase is in ring 1
        m_currPhase[0] = m_currPhase[1] - 4;     // change phase in ring 1
}

void MmitssSignalControl::Complete_Barrier()
{
    // phase 1,2,3,4
    for (int i = 0; i < 4; i++)
    {
        if (m_skipPhase[i] == 1) // phase i not skip
        {
            if (m_phaseSeq[i + 4] == 0) // phase i+4 is not active
            {
                m_skipPhase[i + 4] = 1; // add phase i+4
            }
        }
    }
    // phase 5,6,7,8
    for (int i = 7; i > 3; i--)
    {
        if (m_skipPhase[i] == 1) // phase i not skip
        {
            if (m_phaseSeq[i - 4] == 0) // phase i-4 is not active
            {
                m_skipPhase[i - 4] = 1; // add phase i-4
            }
        }
    }
}

bool MmitssSignalControl::ReadSignalParameters()
{
    fstream fs;
    fs.open(SieMmitss::getSignalConfigCOPFilePath());
    if (!fs.good())
    {
        ITSAPP_WRN("Unable to open file: %s", SieMmitss::getSignalConfigCOPFilePath().c_str());
        return false;
    }
    string temp_string;
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
    // read min green time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_minGreen[0], &m_minGreen[1], &m_minGreen[2],
        &m_minGreen[3], &m_minGreen[4], &m_minGreen[5], &m_minGreen[6], &m_minGreen[7]);
    // read yellow time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_yellow[0], &m_yellow[1], &m_yellow[2],
        &m_yellow[3], &m_yellow[4], &m_yellow[5], &m_yellow[6], &m_yellow[7]);
    // read red time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_red[0], &m_red[1], &m_red[2], &m_red[3],
        &m_red[4], &m_red[5], &m_red[6], &m_red[7]);
    // read max green time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_maxGreen[0], &m_maxGreen[1], &m_maxGreen[2],
        &m_maxGreen[3], &m_maxGreen[4], &m_maxGreen[5], &m_maxGreen[6], &m_maxGreen[7]);
    // read ped walk time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_pedWalk[0], &m_pedWalk[1], &m_pedWalk[2],
        &m_pedWalk[3], &m_pedWalk[4], &m_pedWalk[5], &m_pedWalk[6], &m_pedWalk[7]);
    // read ped clear time
    getline(fs, temp_string);
    strcpy(temp, temp_string.c_str());
    sscanf(temp, "%*s %d %d %d %d %d %d %d %d", &m_pedClr[0], &m_pedClr[1], &m_pedClr[2],
        &m_pedClr[3], &m_pedClr[4], &m_pedClr[5], &m_pedClr[6], &m_pedClr[7]);
    fs.close();

    ITSAPP_LOG("Phase Sequence: %d %d %d %d %d %d %d %d\n", m_phaseSeq[0], m_phaseSeq[1],
        m_phaseSeq[2], m_phaseSeq[3], m_phaseSeq[4], m_phaseSeq[5], m_phaseSeq[6], m_phaseSeq[7]);
    ITSAPP_LOG("Min Green Time: %d %d %d %d %d %d %d %d\n", m_minGreen[0], m_minGreen[1],
        m_minGreen[2], m_minGreen[3], m_minGreen[4], m_minGreen[5], m_minGreen[6], m_minGreen[7]);
    ITSAPP_LOG("Yellow Time: %d %d %d %d %d %d %d %d\n", m_yellow[0], m_yellow[1], m_yellow[2],
        m_yellow[3], m_yellow[4], m_yellow[5], m_yellow[6], m_yellow[7]);
    ITSAPP_LOG("Red Time: %d %d %d %d %d %d %d %d\n", m_red[0], m_red[1], m_red[2], m_red[3],
        m_red[4], m_red[5], m_red[6], m_red[7]);
    ITSAPP_LOG("Max Green Time: %d %d %d %d %d %d %d %d\n", m_maxGreen[0], m_maxGreen[1],
        m_maxGreen[2], m_maxGreen[3], m_maxGreen[4], m_maxGreen[5], m_maxGreen[6], m_maxGreen[7]);
    ITSAPP_LOG("Ped Walk Time: %d %d %d %d %d %d %d %d\n", m_pedWalk[0], m_pedWalk[1], m_pedWalk[2],
        m_pedWalk[3], m_pedWalk[4], m_pedWalk[5], m_pedWalk[6], m_pedWalk[7]);
    ITSAPP_LOG("Ped Clr Time: %d %d %d %d %d %d %d %d\n", m_pedClr[0], m_pedClr[1], m_pedClr[2],
        m_pedClr[3], m_pedClr[4], m_pedClr[5], m_pedClr[6], m_pedClr[7]);

    return true;
}

bool MmitssSignalControl::get_lane_no()
{
    fstream fs(SieMmitss::getLaneNumberFilePath(), fstream::in);
    string temp_string;

    getline(fs, temp_string); // First line is comment
    getline(fs, temp_string); // Second line contains information

    if (temp_string.size() == 0)
    {
        ITSAPP_WRN("Reading Lane_No_File problem");
        fs.close();
        return false;
    }
    sscanf(temp_string.c_str(), "%d %d %d %d %d %d %d %d", &m_laneNo[0], &m_laneNo[1], &m_laneNo[2],
        &m_laneNo[3], &m_laneNo[4], &m_laneNo[5], &m_laneNo[6], &m_laneNo[7]);

    fs.close();
    return true;
}

} // namespace Mmitss
} // namespace WaveApp
