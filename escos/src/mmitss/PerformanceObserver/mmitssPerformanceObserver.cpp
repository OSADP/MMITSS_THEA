/* Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* MMITSS_rsu_BSM_receiver_Gavilan_Savari.cpp
 * Created by Yiheng Feng on 7/11/14.
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 *
 */

// Work with Vissim 6 DriverModel through UDP
// Need drivermodel_udp_R.dll from "DriverModel_DLL_UDP_InFusion" running

// 2014.4.16: Added function
// Receive request from Signal_Control and Performance Measure to send trajectory data

// 2014.7.11: Changed Function
// BSM unpacking is now using Savari function.

#include "mmitssPerformanceObserver.hpp"

#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <facilityBase.hpp>
#include <facilityDatabaseAdapter.hpp>
#include <facilityLayer.hpp>
#include <netinet/in.h>
#include <Poco/Dynamic/Struct.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/IPAddress.h>
#include <Poco/Net/SocketAddress.h>
#include <sigc++/functors/mem_fun.h>
#include <stddef.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include "GetInfo.h"
#include "mmitssUtils.h"
#include "SieMmitss.hpp"
#include "SieMmitssConfig.hpp"
#include "SieMmitssPerfData.hpp"
#include "mmitssPerformanceObserverApp.hpp"

#ifndef DEG2ASNunits
#define DEG2ASNunits (1000000.0) // used for ASN 1/10 MICRO deg to unit converts
#endif

/*
 * Siemens added
 */

namespace WaveApp
{
namespace Mmitss
{

static const int BSM_RX_BUF_SIZE = 500000;

void obtainEndofService(int ihour, int imin, int isec, int iETA, int& iHour, int& iMin, int& iSec);

const uint32_t UDP_RX_TIMEOUT_MS = 500;

static const int MMITSS_MAX_APPROACHES = 8;
static const int MMITSS_MAX_LANES = 5;
static const int MMITSS_MAX_MOVEMENTS = 12;

static const std::vector<std::string> MovementFromIndex = {
    "NB", "EB", "SB", "WB", "NBLT", "NBRT", "EBLT", "EBRT", "SBLT", "SBRT", "WBLT", "WBRT"};

static const vector<string> MOVEMENTS{
    "Northbound_Through",
    "Eastbound_Through",
    "Southbound_Through",
    "Westbound_Through",
    "Northbound_Left_Turn",
    "Northbound_Right_Turn",
    "Eastbound_Left_Turn",
    "Eastbound_Right_Turn",
    "Southbound_Left_Turn",
    "Southbound_Right_Turn",
    "Westbound_Left_Turn",
    "Westbound_Right_Turn",
};

MmitssPerformanceObserver::MmitssPerformanceObserver(MmitssPerformanceObserverApp& parent)
    : m_parent(parent)
    , m_udpReceiver(SieMmitss::getPerfDataPort(),
          BSM_RX_BUF_SIZE,
          [this](const void* data, size_t len) { return this->onTrajectoryReceive(data, len); })
    , m_cfg()
    , m_phases(m_cfg)
    , m_mib()
{
}

bool MmitssPerformanceObserver::onStartup()
{
    ITSAPP_TRACE("Startup");

    return mmitssInit();
}

void MmitssPerformanceObserver::onShutdown()
{
    ITSAPP_TRACE("Shutdown");
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    m_udpReceiver.stop();
}

void MmitssPerformanceObserver::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

void MmitssPerformanceObserver::tick()
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
                    sigc::mem_fun(*this, &MmitssPerformanceObserver::onTimer));
                ITSAPP_TRACE("create a new timer %d: %i", m_timerId, m_timerInterval);
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

void MmitssPerformanceObserver::addDetectedVeh(int lane, double speed, double distance)
{
    // for Twiggs, approach is 1, real lanes are 1.2, 1.3, 1.4
    DetectedVeh TempVeh(speed, distance);
    if (m_radarReportLaneInSameOrder)
    {
        // reported parameter lane is 0,1,2 from radar
        m_forwardedVehList[lane + 2] =
            TempVeh; // the most recent vehicle would replace old one on the same lane
    }
    else
    {
        // reported parameter lane is 2,1,0 from radar
        m_forwardedVehList[4 - lane] =
            TempVeh; // the most recent vehicle would replace old one on the same lane
    }
    ITSAPP_TRACE("one vehicle is detected to enter lane 1.%d", lane + 2);
}

void MmitssPerformanceObserver::updateMaximumEstimation(int approach, int lane, double currentTime)
{
    int i = (approach - 1) / 2 * 5 + lane - 1;
    int temp_phase = m_lanePhase[i];

    ITSAPP_TRACE("lane:%d.%d, i:%d, ", approach, lane, i);

    auto veh_it = m_unequippedVehList[i].begin();
    while (veh_it != m_unequippedVehList[i].end())
    {
        if (currentTime < veh_it->time_join)
        {
            veh_it++;
        }
        else
        {
            ITSAPP_TRACE(
                "i: %d, phaseColor:%d", i, m_mib.getPhaseRead().phaseColor[temp_phase - 1]);
            ITSAPP_TRACE("LQV_stop_flag[%d][%d]: %d", approach, lane - 1,
                m_perf.LQV_stop_flag[approach][lane - 1]);
            if (m_mib.getPhaseRead().phaseColor[temp_phase - 1] == Red)
            {
                ITSAPP_TRACE("maximum_estimation is added 6.75 to %f because of red light",
                    m_perf.maximum_estimation[approach][lane - 1]);
                m_perf.maximum_estimation[approach][lane - 1] += 6.75; // add one vehicle
            }
            else if (m_perf.LQV_stop_flag[approach][lane - 1] == 1) // stopped
            {
                ITSAPP_TRACE("maximum_estimation is added 6.75 to %f because of stopped",
                    m_perf.maximum_estimation[approach][lane - 1]);
                m_perf.maximum_estimation[approach][lane - 1] += 6.75; // add one vehicle
            }
            else
            // if(m_perf.LQV_stop_flag[approach][lane - 1] == 0) // not stopped
            {
                ITSAPP_TRACE("maximum_estimation unchanged because of not stopped.");
            }
            veh_it = m_unequippedVehList[i].erase(veh_it);
        }
    }
}

void MmitssPerformanceObserver::addIntoUnequippedVehList(int approach, int lane, double currentTime)
{
    UnequippedVeh temp_veh;
    int i = (approach - 1) / 2 * 5 + lane - 1;

    // Generating a random number between 0 and 1 for comparing the turning movement probabilities
    float r = ((double)rand() / (RAND_MAX));

    ITSAPP_TRACE("lane:1.%d, i:%d, r is %f, m_turningProportion[%d][0] is %f", lane, i, (double)r,
        i, (double)m_turningProportion[i][0]);

    if (r < m_turningProportion[i][0]) // Vehicle Change one lane to the LEFT
    {
        ITSAPP_TRACE("LEFT turn: distance is %f, maximum_estimation is %f",
            (double)m_detectorStopbarDist[i + 1], m_perf.maximum_estimation[approach][lane]);
        m_disDetQ[i + 1] =
            (double)m_detectorStopbarDist[i + 1] -
            m_perf.maximum_estimation[approach][lane]; // distance from detector to end of queue
        m_timeToJoin[i + 1] = currentTime + ((sqrt(144 + 0.77 * 4 * m_disDetQ[i + 1]) - 12) / 1.55);
        ITSAPP_TRACE("m_timeToJoin is %f later than current time",
            ((sqrt(144 + 0.77 * 4 * m_disDetQ[i + 1]) - 12) / 1.55));
        temp_veh.time_join = m_timeToJoin[i + 1];
        m_unequippedVehList[i + 1].emplace_back(temp_veh);
        ITSAPP_TRACE("add vehicle to m_unequippedVehList[%d]", i + 1);
        // cout<<"Vehicle change its lane to the LEFT"<<endl;
    }
    if (r >= m_turningProportion[i][0] &&
        r < m_turningProportion[i][0] +
                m_turningProportion[i][1]) // Vehicle does NOT change its lane and go THROUGH
    {
        ITSAPP_TRACE("THROUGH: distance is %f, maximum_estimation is %f",
            (double)m_detectorStopbarDist[i], m_perf.maximum_estimation[approach][lane - 1]);
        m_disDetQ[i] =
            (double)m_detectorStopbarDist[i] -
            m_perf.maximum_estimation[approach][lane - 1]; // distance from detector to end of queue
        m_timeToJoin[i] = currentTime + ((sqrt(144 + 0.77 * 4 * m_disDetQ[i]) - 12) / 1.55);
        ITSAPP_TRACE("m_timeToJoin is %f later than current time",
            ((sqrt(144 + 0.77 * 4 * m_disDetQ[i]) - 12) / 1.55));
        temp_veh.time_join = m_timeToJoin[i];
        m_unequippedVehList[i].emplace_back(temp_veh);
        ITSAPP_TRACE("add vehicle to m_unequippedVehList[%d]", i);
        // cout<<"Vehicle doesn't change its lane"<<endl;
        // cout<<"one Unequipped Vehicle will be added at "<<m_timeToJoin<<" seconds"<<endl;
    }
    if (r >= m_turningProportion[i][0] + m_turningProportion[i][1] &&
        r < 1) // Vehicle Change one lane to the RIGHT
    {
        ITSAPP_TRACE("RIGHT turn: distance is %f, maximum_estimation is %f",
            (double)m_detectorStopbarDist[i - 1], m_perf.maximum_estimation[approach][lane - 2]);
        m_disDetQ[i - 1] =
            (double)m_detectorStopbarDist[i - 1] -
            m_perf.maximum_estimation[approach][lane - 2]; // distance from detector to end of queue
        ITSAPP_TRACE("RIGHT turn: m_disDetQ[i - 1] is %f", m_disDetQ[i - 1]);
        m_timeToJoin[i - 1] = currentTime + ((sqrt(144 + 0.77 * 4 * m_disDetQ[i - 1]) - 12) / 1.55);
        ITSAPP_TRACE("m_timeToJoin is %f later than current time",
            ((sqrt(144 + 0.77 * 4 * m_disDetQ[i - 1]) - 12) / 1.55));
        temp_veh.time_join = m_timeToJoin[i - 1];
        m_unequippedVehList[i - 1].emplace_back(temp_veh);
        ITSAPP_TRACE("add vehicle to m_unequippedVehList[%d]", i - 1);
        // cout<<"Vehicle change its lane to the RIGHT"<<endl;
    }
}

bool MmitssPerformanceObserver::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..");
    //    int curPlan = m_mib.CurTimingPlanRead();
    //    ITSAPP_LOG("Current timing plan is: %d", curPlan);
    //
    //    // Generate: configinfo_XX.txt
    //    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getConfigInfoFilePath());

    m_cfg.ReadInConfig(SieMmitss::getConfigInfoFilePath());
    m_mib.setControllerAddress(get_ip_address());

    if (!getDetNumbers())
        return false;

    m_disDetQ.clear();
    m_disDetQ.resize(m_totNumDet);
    m_timeToJoin.clear();
    m_timeToJoin.resize(m_totNumDet);

    if (!getLaneMovementMapping())
        return false;
    // Parse the MAP file
    if (!m_map.ParseIntersection(SieMmitss::getMapFilePath().c_str()))
        return false;

    for (int i = 4; i > 0; i--) // warm up...
    {
        m_mib.PhaseTimingStatusRead();
        m_phases.UpdatePhase(m_mib.getPhaseRead(), m_mib.getPhaseStatus());
        // DetRead();
        // Phases.Display();
        // Phases.RecordPhase(signal_plan_file);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

bool MmitssPerformanceObserver::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    // reset queue and travel time data
    resetQueueAndTravelTimeData();
    auto peneration = 100;
    ITSAPP_VERIFY(m_parent.database().read("its/us/mmitss/control/penetration", peneration));

    m_penetrationRate = static_cast<float>(peneration) / 100;

    int port = SieMmitss::getTrajectoryAwarePort(); // This is the default port in case no port was
                                                    // inputed as an argument
    // For simulation: port = 30000            For Field: port = 3333

    // Initialize the Penetration Rate to be 50% for now
    for (int x = 0; x < 8; x++)
    {
        for (int y = 0; y < 5; y++)
            m_PR[x][y] = m_penetrationRate;
    }
    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });

    // Self defined value for sending frequency; to get the desired port from the user
    /*if (argc==4)
    {
        sscanf(argv[1],"%d", &port);
        sscanf(argv[2],"%f", &MPR);
        for(int xx=0; xx<8 ;xx++)
        {for(int yy=0; yy<5; yy++)
            m_PR[xx][yy] = MPR;
        }
    }
    else if (argc==2)
        sscanf(argv[1],"%d", &port);*/

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
    m_recvAddr.sin_port = htons(port);
    m_recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(m_recvAddr.sin_zero, '\0', sizeof m_recvAddr.sin_zero);

    //-----------------------End of Network Connection------------------//

    //------------init: Begin of Network connection for sending to Data
    // Archiver------------------------------------
    m_sockMmitssCtl = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockMmitssCtl == -1)
    {
        ITSAPP_ERROR("sockfd_mmitssctl");
        close(m_sockfd);
        return false;
    }

    m_testSystemAddr = SieMmitss::getTestSystemSocketAddress();

    //-----------------------End of Network Connection for sending to Data
    // Archiver------------------//

    m_sendTimer = GetSeconds();
    m_calcTimer = GetSeconds();
    m_aggTimer = GetSeconds();

    // initialize red and green start times
    for (int i = 0; i < 8; i++)
    {
        m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
        if (m_mib.getPhaseRead().phaseColor[i] == Red)
            m_redStartTime[i] = GetSeconds();
        else
            m_redStartTime[i] = 0;

        if (m_mib.getPhaseRead().phaseColor[i] == Yellow)
            m_greenStartTime[i] = GetSeconds();
        else
            m_greenStartTime[i] = 0;
    }

    char timestamp[128];
    xTimeStamp(timestamp);

    // write to a file only for _TT_Acc
    // strcat(Performance_Acc_file,timestamp);
    // strcat(Performance_Acc_file,".log");
    // printf("%s\n",Performance_Acc_file);

    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);

    return true;
}

void MmitssPerformanceObserver::onTrajectoryReceive(const void* data, size_t len)
{

    if ((data == nullptr) || (len == 0))
        return;
    m_lastAckCount = m_reqCount;
    m_receivedVehList.clear();
    UnpackTrajData(reinterpret_cast<const uint8_t*>(data));

    double currtime = GetSeconds();

    // Distance-wise
    double maximum[8][5] = {{0}};  // To identify the maximum Queue Length
    double maximum2[8][5] = {{0}}; // To identify the second maximum Queue Length

    // Time-wise
    long max[8][5] = {{0}};  // To identify the last stopped vehicle
    long max2[8][5] = {{0}}; // To identify the second last stopped vehicle

    // Delta Objects for Distance and Time
    double delta_d[8][5];
    long delta_t[8][5];

    double I[8][5];               // Parameter to take into consideration of vehicle arrivals
    double q_speed[8][5] = {{0}}; // Queuing Speed based on MPR

    // Getting Red Elapse Time from the controller
    double red_elapse_time[8];

    m_mib.PhaseTimingStatusRead();
    m_phases.UpdatePhase(m_mib.getPhaseRead(), m_mib.getPhaseStatus());
    m_mib.DetReadVolume(m_detNumbers, m_totNumDet);

    int Det_Vol_Out[m_totNumDet]; // The complete output of the detectors volume data
    int jj = 0;
    for (int i = 0; i < m_totNumDet; i++)
    {
        if (m_detectorStopbarDist[i] > 0)
            Det_Vol_Out[i] = m_mib.getOut()[jj++];
        else
            Det_Vol_Out[i] = 0;
        // cout<<"The final output of the detectors is:"<<Det_Vol_Out[i]<<endl;
    }

    // Getting Red Elapse Time for Each Phase: if signal gets green, keep the previous red
    // duration--------------REMEMBER TO GENERALIZE------------------
    for (int i = 0; i < 8; i++)
    {
        if ((m_previousSignalColor[i] != Green) && (m_mib.getPhaseRead().phaseColor[i] == Green))
        { // Signal just turned Green
            m_greenStartTime[i] = GetSeconds();
            if (i == 1)
            {
                // cout<<"The TOTAL number of arrivals on green phase 2:
                // "<<arrival_on_green[1]<<endl;
                // cout<<"The TOTAL number of arrivals on red phase 2: "<<arrival_on_red[1]<<endl;
                m_arrivalOnGreen[1] = 0;
                m_arrivalOnRed[1] = 0;
            }
        }

        if (m_previousSignalColor[i] != Red &&
            m_mib.getPhaseRead().phaseColor[i] == Red) // Signal just turned red
        {
            m_redStartTime[i] = GetSeconds();
        }
        if (m_mib.getPhaseRead().phaseColor[i] == Red) // Meaning that signal was already red
        {
            red_elapse_time[i] = GetSeconds() - m_redStartTime[i];
        }
        m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
    }

    time_t theTime = time(NULL);
    struct tm* aTime = localtime(&theTime);
    int iHour = 0;
    int iMin = 0;
    int iSec = 0;
    obtainEndofService(
        aTime->tm_hour, aTime->tm_min, aTime->tm_sec, m_timerInterval, iHour, iMin, iSec);

    // MMITSS_LOG("Observation_Period_is: %d:%d:%d - %d:%d:%d", aTime->tm_hour, aTime->tm_min,
    // aTime->tm_sec, iHour,iMin, iSec);

    int connect_veh_flag[20] = {0}; // Whether the vehicle that passing the detector is a connected
                                    // vehcile or not 0: not 1:yes

    // initialize the update_flag before going through the received list: 0:Delete; 1:Update; 2:Add
    for (auto& veh : m_trackedVeh)
    {
        veh.update_flag = 0;
    }

    // Begin of doing Update or Add vehicle to the m_trackedVeh list and dealing with received_list
    for (auto& recVeh : m_receivedVehList)
    {
        ConnectedVehicle TempVeh;
        TempVeh = recVeh;
        int found = 0; // To decide whether the vehicle was found in the list or not
        int veh_id = recVeh.TempID;

        for (auto& curVeh : m_trackedVeh)
        {
            if (curVeh.TempID == veh_id)
            { // vehicle found in list
                int temp_delay = curVeh.delay;
                double temp_distance = curVeh.Distance;
                int temp_passed_flag = curVeh.passed_flag;
                int temp_pre_requested_phase = curVeh.pre_requested_phase;
                // Put whatever in the received list to the trackeveh list
                curVeh = recVeh;
                curVeh.delay = temp_delay;
                curVeh.Distance = temp_distance;
                curVeh.passed_flag = temp_passed_flag;
                curVeh.pre_requested_phase = temp_pre_requested_phase;
                // sprintf(temp_log,"Curr time is: %.2lf and Entry Time is:
                // %.2lf\n",currtime,m_trackedVeh.Data().entry_time);
                // cout<<temp_log;
                // cout<<"Curr time is: "<<currtime<< " and Entry Time:
                // "<<m_trackedVeh.Data().entry_time<<endl;

                if (currtime - curVeh.entry_time <= 2)
                { // To understand if the vehicle was deleted
                    // and added again because of a U-Turn.
                    // So set the delay and Distance back to zero!
                    curVeh.Distance = 0;
                    curVeh.delay = 0;
                    temp_distance = 0;
                }

                // Travel Time Calculation
                curVeh.TT = curVeh.leaving_time - curVeh.entry_time;

                // Distance Traversed Calculation
                // In order to calculate the Distance Traversed at least two points are needed!
                if (curVeh.nFrame >= 2)
                {
                    if (GetSeconds() - curVeh.leaving_time < 2)
                    {
                        double x1 = curVeh.N_Offset[1];
                        double x2 = curVeh.N_Offset[0];
                        double y1 = curVeh.E_Offset[1];
                        double y2 = curVeh.E_Offset[0];
                        curVeh.Distance =
                            temp_distance + sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
                        // sprintf(temp_log,"%.2lf; %d; %.2f;
                        // %.2f\n",GetSeconds(),m_trackedVeh.Data().TempID,
                        // m_trackedVeh.Data().Distance, temp_distance);
                    }
                }

                // Delay Calculation
                if ((curVeh.Speed < 0.8) && (curVeh.stop_flag == 0) && ((curVeh.approach % 2) == 1))
                { // vehicle has not stopped before
                    curVeh.delay += 1;
                }
                if ((curVeh.stop_flag == 1) && ((curVeh.approach % 2) == 1))
                {
                    curVeh.delay += 1;
                }
                if ((curVeh.Speed > 0.8) && (curVeh.stop_flag == 1))
                { // to keep track of the last stopped vehicle
                    curVeh.stop_flag = 0;
                }
                curVeh.update_flag = 1; // updated
                found = 1;
                // to jump out of the current for loop and
                // not go through the end of the m_trackedVeh list!!
                // Because the vehicle was just found.
                break;
            }
        }

        if (found == 0) // It's a new vehicle which should be added to the m_trackedVeh list
        {
            TempVeh.update_flag = 2;
            TempVeh.delay = 0;
            TempVeh.Distance = 0;
            TempVeh.passed_flag = 0;
            TempVeh.pre_requested_phase = -1;
            m_trackedVeh.emplace_back(TempVeh);
        }
    }

    auto vehIt = m_trackedVeh.begin();
    while (vehIt != m_trackedVeh.end())
    {
        //-------------------------------Arrivals on Green-----------------------------------------
        // cout<<"Passed Flag value is: "<<m_m_trackedVeh.Data().passed_flag<<endl;
        // cout<<"Requested phase is: "<<m_m_trackedVeh.Data().req_phase<<endl;
        // cout<<"m_m_trackedVeh.Data().time_stop is:"<<m_m_trackedVeh.Data().time_stop<<endl;
        // cout<<"m_m_trackedVeh.Data().pre_requested_phase
        // is:"<<m_m_trackedVeh.Data().pre_requested_phase<<endl;
        if (vehIt->req_phase >= 0)
        {
            vehIt->pre_requested_phase = vehIt->req_phase;
        }

        if (vehIt->req_phase < 0)
        {
            if (vehIt->passed_flag == 0)
            {
                // cout<<"Vehicle passed the stop bar!!"<<endl;
                vehIt->passed_flag = 1;
                if (vehIt->time_stop <= 0.1)
                { // The vehicle didn't stop and arrived on Green
                    if (vehIt->pre_requested_phase == 2)
                    {
                        m_arrivalOnGreen[1]++;
                        // cout<<"The number of arrivals on Green phase 2:
                        // "<<arrival_on_green[1]<<endl;
                    }
                }
                else
                { // The vehicle stopped and arrived on Red
                    if (vehIt->pre_requested_phase == 2)
                    {
                        m_arrivalOnRed[1]++;
                        // cout<<"The number of arrivals on Red phase 2: "<<arrival_on_red[1]<<endl;
                    }
                }
            }
        }
        else
        { // NOTE MV: Added this 'else' because mmitss added vehicles that passed the stopbar to the
            // queue
            //-----------------------To deal with QLE---------------------------
            for (int i = 0; i < m_totNumDet;
                 i++) // going through approaches 1,3,5,7 and lanes 1,2,3,4,5
            {
                // Build based on the matrices of Approach*lane & stopbar distance of the associated
                // detectors.
                if ((m_detectorStopbarDist[i] > 0) &&
                    (fabs(vehIt->stopBarDistance - (double)m_detectorStopbarDist[i]) < 20))
                {
                    connect_veh_flag[i] = 1;
                    // cout<<"A Connected Vehicle is Passing the
                    // Detector!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                }
            }
            auto& appr = vehIt->approach;
            auto lane = vehIt->lane - 1;
            // cout<<"Vehicle ID of "<<m_trackedVeh.Data().TempID<<" with stop flag of:
            // "<<m_trackedVeh.Data().stop_flag<<endl;
            if (vehIt->stop_flag == 1) // if the vehicle is stopped
            {
                // to get the distance difference for the last two vehicles
                if (vehIt->stopBarDistance > maximum[appr][lane])
                {
                    // cout<<<<endl;
                    // To keep track of the second last vehicle
                    maximum2[appr][lane] = maximum[appr][lane];
                    maximum[appr][lane] = vehIt->stopBarDistance;
                    // cout<<"Maximum for approach "<<m_trackedVeh.Data().approach<<" lane
                    // "<<m_trackedVeh.Data().lane<<" is:
                    // "<<maximum[m_trackedVeh.Data().approach][m_trackedVeh.Data().lane -1]<<endl;
                    // This is the last queued vehicle
                    m_perf.LQV[appr][lane] = vehIt->TempID;
                }

                // to get the time difference for the last two vehicles
                if (vehIt->time_stop > max[appr][lane])
                {
                    max2[appr][lane] = max[appr][lane];
                    max[appr][lane] = vehIt->time_stop;
                }

                //----------------------Attention-----------------
                if (currtime - vehIt->time_stop < 3.1)
                {
                    // if (phase_read.phaseColor[3]==1)
                    //{
                    // SHOULD BE CALIBRATED...!!! 3 was added to consider the rear position
                    // of the vehicle instead of the center
                    m_perf.maximum_estimation[appr][lane] = maximum[appr][lane] + 3;
                    // For the second last vehicle
                    m_perf.maximum_estimation2[appr][lane] = maximum2[appr][lane] + 3;
                    // cout<<" First: "<<TempPerf.maximum_estimation[1][0]<<" and Second:
                    // "<<TempPerf.maximum_estimation[1][1]<<endl;
                }
            }

            // Tracking the last queued vehicle as it starts moving
            if (vehIt->TempID == m_perf.LQV[appr][lane])
            {
                /*if (m_trackedVeh.Data().stop_flag == 0)
                {
                    maximum[m_trackedVeh.Data().approach][m_trackedVeh.Data().lane -1] =
                m_trackedVeh.Data().stopBarDistance;
                }*/
                m_perf.LQV_stop_flag[appr][lane] = vehIt->stop_flag;
            }
        }
        //-----------------------------------To remove from the list------------------------------
        if (vehIt->update_flag == 0)
        {
            // get movement of vehicle (i.e. NBLT, etc...)
            auto move = getVehicleMovementOnEgress(*vehIt);
            // Write to a file for TT&D Accumulation Data
            if (move != 0)
            {
                m_perf.App_Ext_counter[move - 1]++;
                float temp_tardity = vehIt->TT / vehIt->Distance;
                m_perf.vehicle_tardity[move - 1] += temp_tardity;

                m_perf.total_distance[move - 1] += (float)vehIt->Distance;
                m_perf.App_Ext_Tardity[move - 1] =
                    (m_perf.vehicle_tardity[move - 1] * m_perf.total_distance[move - 1]) /
                    (m_perf.App_Ext_counter[move - 1] * m_perf.App_Ext_counter[move - 1]);
                m_perf.num_observation[move - 1]++;
            }
            if (currtime - m_aggTimer > m_aggFrequency)
            { // Logs every 5 minutes
                m_aggTimer = currtime;
                for (int i = 0; i < 12; i++)
                {
                    if (m_perf.App_Ext_Tardity[i] > 0)
                        ITSAPP_TRACE(
                            "Extended Tardity with %d vehicles for movement %d is: % f at time "
                            "%.2lf",
                            m_perf.num_observation[i], i + 1, (double)m_perf.App_Ext_Tardity[i],
                            GetSeconds());
                    m_perf.num_observation[i] = 0;
                    m_perf.App_Ext_Tardity[i] = 0;
                }
            }
            vehIt = m_trackedVeh.erase(vehIt);
        }
        else
            vehIt++;
    }

    // store number of tracked vehicles per ingress lane
    for (int app = 1; app < 8; app += 2) // MMITSS only processes 1,3,5,7
    {
        for (int lane = 0; lane < 5; lane++)
        {
            m_perf.vehicle_count[app][lane] = 0;
            for (const auto& veh : m_trackedVeh)
            {
                if ((veh.approach == app) && (veh.lane == (lane + 1)))
                {
                    m_perf.vehicle_count[app][lane]++;
                }
            }
        }
    }

    //---------------------------------- End of Updating the m_trackedVeh list and everything will
    // be based on it from now on-----------------------------------------//

    // add veh into m_unequippedVehList based on m_forwardedVehList
    for (const auto& veh_p : m_forwardedVehList)
    {
        ITSAPP_TRACE("m_forwardedVehList[%d] is %.2f:", veh_p.first, veh_p.second.distance);
        addIntoUnequippedVehList(1, veh_p.first, currtime);
    }
    m_forwardedVehList.clear();
    for (int i = 1; i <= (MAX_LANES_RADAR_DETECTED + 1); i++)
    {
        updateMaximumEstimation(1, i, currtime);
    }

    //-----------------Queue Length Estimation Continuation---------------------------
    for (int i = 0; i < m_totNumDet; i++)
    {

        // mapping between the approach and i: i=[0 1 2 3 ... 19] ----> App=[1 1 1 1 1 3 3 3 3 3 5 5
        // 5 5 5 7 7 7 7 7]
        int temp_app = (i / 5) * 2 + 1;
        // mapping between the lane and i: i=[0 1 2 3 ... 19] ----> Lane=[1 2 3 4 5 1 2 3 4 5 1 2 3
        // 4 5 1 2 3 4 5]
        int temp_lane = (i % 5) + 1;
        // Reading from the config file
        int temp_phase = m_lanePhase[i];

        // cout<<"Approach: "<<temp_app<<" Lane: "<<temp_lane<<" Phase: "<<temp_phase<<endl;

        // never set this for radar detector
        if (m_detNumbers[i] > 0)
        { // Case in which the lane has a detector
            // cout<<"Inside IF Condition, w Detector"<<endl;
            ITSAPP_TRACE("m_detNumbers[%d]: %d, temp_app: %d, temp_lane: %d", i, m_detNumbers[i],
                temp_app, temp_lane);
            if (connect_veh_flag[i] == 0)
            {
                // cout<<"Dealing with Unequipped Vehicles Started!"<<endl;
                // Yiheng and I QLE

                if (Det_Vol_Out[i] > 0)
                {
                    addIntoUnequippedVehList(temp_app, temp_lane, currtime);
                }
                // cout<<"Distance from Detector to LQV is: "<< m_disDetQ<<endl;

                updateMaximumEstimation(temp_app, temp_lane, currtime);
                //                if (temp_app == 1)
                //                    ITSAPP_TRACE(
                //                        "%d.%d: %f", temp_app, temp_lane,
                //                        m_perf.maximum_estimation[temp_app][temp_lane - 1]);
            }
        }
        else // Case in which there is no system detector
        {
            // cout<<"Inside Else Condition, w/o Detector"<<endl;
            if (m_perf.maximum_estimation[temp_app][temp_lane - 1] >= 0)
            {
                // sprintf(temp_log, "Q Length and second max Q length for approach %d and lane %d
                // is: %f %f\n",
                //  temp_app, temp_lane, maximum_estimation[temp_app][temp_lane-1],
                //  maximum_estimation2[temp_app][temp_lane-1]);

                if (m_perf.maximum_estimation2[temp_app][temp_lane - 1] != 0)
                    delta_d[temp_app][temp_lane - 1] =
                        m_perf.maximum_estimation[temp_app][temp_lane - 1] -
                        m_perf.maximum_estimation2[temp_app][temp_lane - 1];
                else
                    delta_d[temp_app][temp_lane - 1] =
                        m_perf.maximum_estimation[temp_app][temp_lane - 1];
            }

            if (max[temp_app][temp_lane - 1] >= 0)
            {
                // sprintf(temp_log, "Last Time stop and second last time stop for approach %d and
                // lane %d is: %li %li\n",
                //  temp_app, temp_lane, max[temp_app][temp_lane-1], max2[temp_app][temp_lane-1]);

                if (max2[temp_app][temp_lane - 1] != 0)
                    delta_t[temp_app][temp_lane - 1] =
                        max[temp_app][temp_lane - 1] - max2[temp_app][temp_lane - 1];
                else
                    //---------------------------------------------ATTENTION------------------------------------
                    // Last time stop - (beginning of Red of corresponding Phase for that Approach)
                    delta_t[temp_app][temp_lane - 1] =
                        max[temp_app][temp_lane - 1] - (currtime - red_elapse_time[temp_phase - 1]);
            }

            // Estimated Queue Length
            I[temp_app][temp_lane - 1] = (1 - m_PR[temp_app][temp_lane - 1]) * 10;
            q_speed[temp_app][temp_lane - 1] = I[temp_app][temp_lane - 1] / 14;

            // cout<<"I for approach "<<temp_app<<" and lane "<<temp_lane-1<<" is:
            // "<<I[temp_app][temp_lane-1]<<endl;
            // cout<<"q_speed for approach "<<temp_app<<" and lane "<<temp_lane-1<<" is:
            // "<<q_speed[temp_app][temp_lane-1]<<endl;

            if (delta_d[temp_app][temp_lane - 1] / delta_t[temp_app][temp_lane - 1] >
                q_speed[temp_app][temp_lane - 1])
            {
                m_perf.maximum_estimation[temp_app][temp_lane - 1] =
                    m_perf.maximum_estimation[temp_app][temp_lane - 1] +
                    q_speed[temp_app][temp_lane - 1];
            }
            else
            {
                m_perf.maximum_estimation[temp_app][temp_lane - 1] =
                    m_perf.maximum_estimation[temp_app][temp_lane - 1] +
                    delta_d[temp_app][temp_lane - 1] / delta_t[temp_app][temp_lane - 1];
            }
        }

        // LQV_discharging[temp_app][temp_lane-1] = 2 + m_greenStartTime[temp_phase-1] +
        // floor(maximum_estimation[temp_app][temp_lane-1]/6.75)*1.25; //SHOULD BE
        // CALIBRATED...!!!!!
        m_perf.LQV_discharging[temp_app][temp_lane - 1] =
            2 + m_greenStartTime[temp_phase - 1] +
            sqrt((m_perf.maximum_estimation[temp_app][temp_lane - 1] * 2) /
                 1.5); // SHOULD BE CALIBRATED...!!!!!
        m_perf.queue_counter[temp_app][temp_lane - 1] =
            ceil(m_perf.maximum_estimation[temp_app][temp_lane - 1] /
                 6.75); // SHOULD BE CALIBRATED...!!!!!
        // To avoid getting wrong value for queue_counter because of the ceil function
        if (m_perf.maximum_estimation[temp_app][temp_lane - 1] < 3)
            m_perf.queue_counter[temp_app][temp_lane - 1] = 0;

        if ((m_mib.getPhaseRead().phaseColor[temp_phase - 1] != Red) &&
            (currtime >= m_perf.LQV_discharging[temp_app][temp_lane - 1]) &&
            (m_perf.maximum_estimation[temp_app][temp_lane - 1] >
                0)) // Added the last part to avoid getting negative values
        {
            // cout<<"Discharging part started"<<endl;
            m_perf.maximum_estimation[temp_app][temp_lane - 1] -=
                9.75; // SHOULD BE CALIBRATED...!!!!!
        }

        if (m_perf.maximum_estimation[temp_app][temp_lane - 1] < 0)
            m_perf.maximum_estimation[temp_app][temp_lane - 1] = 0;

        // Logging the First line of the log file for the Queue Length
        // sprintf(temp_log, "%.2f ", m_tempPerf.maximum_estimation[temp_app][temp_lane-1]);
        // queue_fs<<temp_log; //cout<<temp_log;
    }
    // Logging the Second line of the log file for the number of vehicles in the queue

    // Store queue length once per sec
    storeQueueData();
    // sendto(m_sockMmitssCtl,oneSecData.data(),oneSecData.size() + 1,0,
    //        (struct sockaddr *)&m_mmitssCtlAddress, sizeof(m_mmitssCtlAddress));

    if (!m_testSystemAddr.empty())
    { // send data to test system as well
        ITSAPP_TRACE("Sent queue to test system %s", m_testSystemAddr.c_str());

        Poco::Net::DatagramSocket dest;
        Poco::Net::SocketAddress addr(m_testSystemAddr);
        std::string oneSecData = SieMmitss::queueLengthToJSON(m_perf, m_trackedVeh.size());
        dest.sendTo(oneSecData.data(), oneSecData.size() + 1,
            Poco::Net::SocketAddress(addr.host(), addr.port() + 1));
    }

    //----------------------Going through the list every 15 seconds now for the purpose of webpage!!
    // printf ("ppooiinntt b: %.2f\n", GetSeconds());
    if (currtime - m_calcTimer > m_calcFrequency) // Every 15 seconds
    {
        m_calcTimer = currtime;

        // write to a file only for the sake of RSE webserver
        // fstream perf_fs;
        // perf_fs.open(Performance_data_file,ios::out);

        // This part is for testing and validating the received data

        // Get the Observation Period
        time(&theTime);
        aTime = localtime(&theTime);
        obtainEndofService(
            aTime->tm_hour, aTime->tm_min, aTime->tm_sec, m_calcFrequency, iHour, iMin, iSec);
        // MMITSS_LOG("Next observation period: %d:%d:%d - %d:%d:%d", aTime->tm_hour, aTime->tm_min,
        // aTime->tm_sec, iHour,iMin, iSec);

        float sum_TT[12] = {0};
        float sum_Delay[12] = {0};
        float sum_numstops[12] = {0};

        int count[12] = {0};

        for (auto& curVeh : m_trackedVeh)
        {
            // TT & Delay Calculation for each individual vehicle
            curVeh.TT = curVeh.leaving_time - curVeh.entry_time;

            // only for the sake of RSE webserver
            // MMITSS_LOG("%d %d %d %.2f %d %.2f",m_trackedVeh.Data().TempID,
            // m_trackedVeh.Data().approach,
            //      m_trackedVeh.Data().lane, m_trackedVeh.Data().Distance,
            //      m_trackedVeh.Data().TT,m_trackedVeh.Data().delay);
            // Average TT & Delay for each individual approach
            auto moveIdx = getMovementIdx(curVeh);
            if (moveIdx >= 0)
            {
                ++count[moveIdx];
                sum_TT[moveIdx] += curVeh.TT;
                sum_Delay[moveIdx] += (float)curVeh.delay;
                sum_numstops[moveIdx] += curVeh.queue_flag;
            }
        }

        if (m_trackedVeh.size() > 0) // Logs only when there is a vehicle in the range!
        {
            // for logging the average travel times and delay for each approach
            for (int i = 0; i < 12; i++)
            {
                m_perf.App_TT[i] = sum_TT[i] / count[i];
                m_perf.App_Delay[i] = sum_Delay[i] / count[i];
                m_perf.App_numstops[i] = sum_numstops[i] / count[i];
                m_perf.App_throughput[i] = count[i];
                // Acc_App_TT[i][two_minute_counter] = App_TT[i];
            }
        }
    } // End of if(currtime - send_timer > calc_frequency) //Every 15 Seconds

    // send travel time data every 15 sec
    storeTravelTimeData();

    // std::string ttData = SieMmitss::ttAccToJSON(m_tempPerf);
    // MMITSS_LOG("Send TT data: %s", ttData.c_str());
    // sendto(m_sockMmitssCtl,ttData.data(),ttData.size() + 1,0,
    //        (struct sockaddr *)&m_mmitssCtlAddress, sizeof(m_mmitssCtlAddress));

    if (!m_testSystemAddr.empty())
    { // send data to test system as well
        ITSAPP_TRACE("Sent ttData to test system %s", m_testSystemAddr.c_str());
        Poco::Net::DatagramSocket dest;
        Poco::Net::SocketAddress addr(m_testSystemAddr);
        std::string ttData = SieMmitss::ttAccToJSON(m_perf);
        dest.sendTo(ttData.data(), ttData.size() + 1,
            Poco::Net::SocketAddress(addr.host(), addr.port() + 1));
    }
}

void MmitssPerformanceObserver::onTimer(siekdbus::timerid_t id)
{
    NOTUSED(id);
    if (not m_started)
        return;

    std::ostringstream reqStr;
    reqStr << "Request performance_observer ";
    reqStr << m_reqCount;

    std::string req = reqStr.str();
    if (m_reqCount - m_lastAckCount < MAX_UNACK_REQ)
    {
        // send requested
        sendto(m_sockfd, req.c_str(), req.length(), 0, (struct sockaddr*)&m_recvAddr,
            sizeof(m_recvAddr));
    }
    else
    {
        // restart receiver
        m_udpReceiver.stop();
        m_udpReceiver.start();
        m_lastAckCount = m_reqCount;
    }
    m_reqCount++;
}

void MmitssPerformanceObserver::UnpackTrajData(const uint8_t* ablob)
{
    int No_Veh;
    int i, j;
    int offset;
    offset = 0;
    long tempLong;
    uint8_t byteA; // force to unsigned this time,
    uint8_t byteB; // we do not want a bunch of sign extension
    uint8_t byteC; // math mucking up our combine logic
    uint8_t byteD;

    // header 2 bytes
    byteA = ablob[offset++];
    byteB = ablob[offset++];
    // MSG ID: 0x01 for trajectory data send to performance observer
    offset++; // move past to next item
    // No. of Vehicles in the list
    byteA = ablob[offset++];
    byteB = ablob[offset++];
    No_Veh = (byteA << 8) + byteB; // in fact unsigned

    // MMITSS_LOG("NoVeh:%d", No_Veh);

    // Do each vehicle
    for (i = 0; i < No_Veh; i++)
    {

        ConnectedVehicle TempVeh;
        // Do Veh ID
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.TempID = (byteA << 8) + byteB; // in fact unsigned

        // Do nFrame
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.nFrame = (byteA << 8) + byteB; // in fact unsigned

        // Do Speed
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        byteC = ablob[offset++];
        byteD = ablob[offset++];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.Speed = (tempLong / DEG2ASNunits); // convert and store as float

        // Do entry_time
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        byteC = ablob[offset++];
        byteD = ablob[offset++];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.entry_time = (tempLong); // convert and store as float

        // Do leaving_time
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        byteC = ablob[offset++];
        byteD = ablob[offset++];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.leaving_time = (tempLong); // convert and store as float

        // Do Veh Approach
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempVeh.approach = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;

        // Do Veh lane
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.lane = (byteA << 8) + byteB; // in fact unsigned

        // Do Distance to Stop Bar
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        byteC = ablob[offset++];
        byteD = ablob[offset++];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.stopBarDistance = (tempLong / DEG2ASNunits); // convert and store as float

        // Do Veh stop_flag

        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.stop_flag = (byteA << 8) + byteB; // in fact unsigned

        // Do Veh queue_flag
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.queue_flag = (byteA << 8) + byteB; // in fact unsigned

        // Do time when vehicle stops
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        byteC = ablob[offset++];
        byteD = ablob[offset++];
        tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
        TempVeh.time_stop = (tempLong); // convert and store as float

        // Do Veh Previous Approach
        byteA = ablob[offset + 0];
        byteB = ablob[offset + 1];
        TempVeh.previous_approach = (byteA << 8) + byteB; // in fact unsigned
        offset = offset + 2;

        // Do Veh Previous Lane
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.previous_lane = (byteA << 8) + byteB; // in fact unsigned

        // Do Veh req_phase
        byteA = ablob[offset++];
        byteB = ablob[offset++];
        TempVeh.req_phase = (byteA << 8) + byteB; // in fact unsigned
        if (TempVeh.req_phase >= 65535)
            TempVeh.req_phase = -1;

        // for(j=TempVeh.nFrame-2;j<TempVeh.nFrame;j++)
        for (j = 0; j < 2; j++)
        {
            // N_Offset
            byteA = ablob[offset++];
            byteB = ablob[offset++];
            byteC = ablob[offset++];
            byteD = ablob[offset++];
            tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
            TempVeh.N_Offset[j] = (tempLong / DEG2ASNunits); // convert and store as float

            // cout<<"N_Offset["<<j<<"] is:"<<TempVeh.N_Offset[j]<<endl;

            // E_Offset
            byteA = ablob[offset++];
            byteB = ablob[offset++];
            byteC = ablob[offset++];
            byteD = ablob[offset++];
            tempLong = (unsigned long)((byteA << 24) + (byteB << 16) + (byteC << 8) + (byteD));
            TempVeh.E_Offset[j] = (tempLong / DEG2ASNunits); // convert and store as float

            // cout<<"E_Offset["<<j<<"] is:"<<tempLong<<endl;
            // cout<<"E_Offset["<<j<<"] is:"<<TempVeh.E_Offset[j]<<endl;
        }
        // cout<<"Done with one vehicle"<<endl;

        /*MMITSS_LOG("ID:%d|Spd:%lf|T1:%lf|T2%lf|Appr:%d|Lane:%d|Dist:%lf|Stop:%d|Q:%d|Phase:%d|",
                TempVeh.TempID, TempVeh.Speed, TempVeh.entry_time, TempVeh.leaving_time,
                TempVeh.approach, TempVeh.lane, TempVeh.stopBarDistance, TempVeh.stop_flag,
                TempVeh.queue_flag, TempVeh.req_phase);*/

        m_receivedVehList.emplace_back(TempVeh); // 100% Market Penetration Rate

        //--------------------------------------------Market Penetration
        // Rate---------------------------------------------------
        /* if(TempVeh.TempID%4==0) //25% Penetration Rate
           {
        // if(TempVeh.TempID%4==0 || TempVeh.TempID%4==1 || TempVeh.TempID%4==3) //75% Penetration
        Rate!!!
        trackedveh.InsertRear(TempVeh);   //add the new vehicle to the tracked list
        }  */
    }
}

void MmitssPerformanceObserver::resetQueueAndTravelTimeData()
{
    auto& db = m_parent.database();
    ITSAPP_VERIFY(db.clearList("/its/us/mmitss/performance/queue_list"));
    ITSAPP_VERIFY(db.clearList("/its/us/mmitss/performance/traf_perf_list"));
}

void MmitssPerformanceObserver::storeQueueData()
{
    ITSAPP_TRACE("number of vehicles: %d", m_trackedVeh.size());
    auto& db = m_parent.database();
    // store number of tracked vehicles
    ITSAPP_VERIFY(db.write("/its/us/mmitss/performance/num_veh", m_trackedVeh.size()));
    // store queue data per lane
    its::FacilityDatabaseAdapter::IndexList queueLaneIdList;
    ITSAPP_VERIFY(db.getListIndices("/its/us/mmitss/performance/queue_list", queueLaneIdList));

    for (int app = 1; app < MMITSS_MAX_APPROACHES; app += 2) // MMITSS only processes 1,3,5,7
    {
        for (int lane = 0; lane < MMITSS_MAX_LANES; lane++)
        {
            // if((m_perf.maximum_estimation[app][lane] > 0) || (m_perf.queue_counter[app][lane] >
            // 0) ||
            //   (m_perf.vehicle_count.at(app).at(lane) > 0))
            //{
            auto qObj = std::make_shared<its::DatabaseListItem>();
            auto laneId = m_map.getLaneID(app - 1, lane);
            qObj->content["queue_len"] =
                std::to_string(static_cast<int>((m_perf.maximum_estimation[app][lane])));
            qObj->content["queue_count"] = std::to_string(m_perf.queue_counter[app][lane]);
            qObj->content["vehicle_count"] = std::to_string(m_perf.vehicle_count.at(app).at(lane));
            qObj->content["approach"] = std::to_string(app);
            qObj->content["lane"] = std::to_string(lane + 1);
            if (laneId > 0)
            {
                ITSAPP_VERIFY(
                    db.setListItem("/its/us/mmitss/performance/queue_list", qObj, laneId));
                // remove id from current id list
                auto it = std::find(queueLaneIdList.begin(), queueLaneIdList.end(), laneId);
                if (it != queueLaneIdList.end())
                    queueLaneIdList.erase(it);
            }
            // }
        }
    }
    // remove remaining old items
    for (const auto& id : queueLaneIdList)
        ITSAPP_VERIFY(db.delListItem("/its/us/mmitss/performance/queue_list", id));
}

void MmitssPerformanceObserver::storeTravelTimeData()
{
    auto& db = m_parent.database();
    its::DatabaseListData perfList;
    for (int mov = 0; mov < MMITSS_MAX_MOVEMENTS; mov++)
    {
        Poco::DynamicStruct tt;
        if ((m_perf.App_TT[mov] > 0) || (m_perf.App_Delay[mov] > 0) ||
            (m_perf.App_numstops[mov] > 0) || (m_perf.App_throughput[mov] > 0))
        {
            auto perfObj = std::make_shared<its::DatabaseListItem>();
            perfObj->content["movement"] = MovementFromIndex[mov];
            perfObj->content["travel_time"] = std::to_string(static_cast<int>(m_perf.App_TT[mov]));
            perfObj->content["delay"] = std::to_string(static_cast<int>(m_perf.App_Delay[mov]));
            perfObj->content["num_stops"] =
                std::to_string(static_cast<int>(m_perf.App_numstops[mov]));
            perfObj->content["throughput"] = std::to_string(m_perf.App_throughput[mov]);
            perfList.emplace_back(perfObj);
        }
    }
    if (perfList.size())
    {
        ITSAPP_VERIFY(db.replaceList("/its/us/mmitss/performance/traf_perf_list", perfList));
    }
    else
    {
        ITSAPP_VERIFY(db.clearList("/its/us/mmitss/performance/traf_perf_list"));
    }
    ITSAPP_TRACE("perfList.size(): %d", perfList.size());
}

int MmitssPerformanceObserver::getVehicleMovementOnEgress(const ConnectedVehicle& veh)
{
    int movement = 0;
    switch (veh.approach)
    {
        case 8:
            switch (veh.previous_approach)
            {
                // NBLT
                case 5:
                    movement = m_movementMapping[2];
                    break;
                // SBRT
                case 1:
                    movement = m_movementMapping[7];
                    break;
                // WBTh
                case 3:
                    movement = m_movementMapping[3];
                    break;
            }
            break;
        // EBRT, SBTh, WBLT
        case 6:
            switch (veh.previous_approach)
            {
                // WBLT
                case 3:
                    movement = m_movementMapping[5];
                    break;
                // SBTh
                case 1:
                    movement = m_movementMapping[6];
                    break;
                // EBRT
                case 7:
                    movement = m_movementMapping[10];
                    break;
            }
            break;
        // NBRT, EBTh, SBLT
        case 4:
            switch (veh.previous_approach)
            {
                // SBLT
                case 1:
                    movement = m_movementMapping[8];
                    break;
                // NBRT
                case 5:
                    movement = m_movementMapping[1];
                    break;
                // EBTh
                case 7:
                    movement = m_movementMapping[9];
                    break;
            }
            break;
        // WBRT, NBTh, EBLT
        case 2:
            switch (veh.previous_approach)
            {
                // EBLT
                case 7:
                    movement = m_movementMapping[11];
                    break;
                // WBRT
                case 3:
                    movement = m_movementMapping[4];
                    break;
                // NBTh
                case 5:
                    movement = m_movementMapping[0];
                    break;
            }
            break;
    }
    return movement;
}

int MmitssPerformanceObserver::getMovementIdx(const ConnectedVehicle& veh)
{
    std::string movement;
    switch (veh.approach)
    {
        // NB
        case 5:
            // NBLT
            if (veh.lane == m_laneMapping[5])
                movement = "Northbound_Left_Turn";
            // NBRT
            else if (veh.lane == m_laneMapping[4])
                movement = "Northbound_Right_Turn";
            // NBTh
            else
                movement = "Northbound_Through";
            break;
        // EB
        case 7:
            // EBLT
            if (veh.lane == m_laneMapping[7])
                movement = "Eastbound_Left_Turn";
            // EBRT
            else if (veh.lane == m_laneMapping[6])
                movement = "Eastbound_Right_Turn";
            // EBTh
            else
                movement = "Eastbound_Through";
            break;
        // SB
        case 1:
            // SBLT
            if (veh.lane == m_laneMapping[1])
                movement = "Southbound_Left_Turn";
            // SBTh
            else
                movement = "Southbound_Through";
            break;
        // WB
        case 3:
            // WBLT
            if (veh.lane == m_laneMapping[3])
                movement = "Westbound_Left_Turn";
            // WBRT
            else if (veh.lane == m_laneMapping[2])
                movement = "Westbound_Right_Turn";
            // WBTh
            else
                movement = "Westbound_Through";
            break;
        // NBLT, SBRT, WBTh
        case 8:
            switch (veh.previous_approach)
            {
                // NBLT
                case 5:
                    movement = "Northbound_Left_Turn";
                    break;
                // SBRT
                case 1:
                    movement = "Southbound_Right_Turn";
                    break;
                // WBTh
                case 3:
                    movement = "Westbound_Through";
                    break;
            }
            break;
        // EBRT, SBTh, WBLT
        case 6:
            switch (veh.previous_approach)
            {
                // WBLT
                case 3:
                    movement = "Westbound_Left_Turn";
                    break;
                // SBTh
                case 1:
                    movement = "Southbound_Through";
                    break;
                // EBRT
                case 7:
                    movement = "Eastbound_Right_Turn";
                    break;
            }
            break;
        // NBRT, EBTh, SBLT
        case 4:
            switch (veh.previous_approach)
            {
                // SBLT
                case 1:
                    movement = "Southbound_Left_Turn";
                    break;
                // NBRT
                case 5:
                    movement = "Northbound_Right_Turn";
                    break;
                // EBTh
                case 7:
                    movement = "Eastbound_Through";
                    break;
            }
            break;
        // WBRT, NBTh, EBLT
        case 2:
            switch (veh.previous_approach)
            {
                // EBLT
                case 7:
                    movement = "Eastbound_Left_Turn";
                    break;
                // WBRT
                case 3:
                    movement = "Westbound_Right_Turn";
                    break;
                // NBTh
                case 5:
                    movement = "Northbound_Through";
                    break;
            }
            break;
    }
    auto it = std::find(MOVEMENTS.begin(), MOVEMENTS.end(), movement);
    if (it == MOVEMENTS.end())
        return -1;
    return std::distance(MOVEMENTS.begin(), it);
}

bool MmitssPerformanceObserver::getLaneMovementMapping()
{
    fstream sh;

    sh.open(SieMmitss::getLaneMovementMapFilePath().c_str());

    string temp_string;

    getline(sh, temp_string); // First line is comment explaining the order of the lanes
    getline(sh, temp_string); // Second line shows the corresponding lane numbers: SBRT, SBLT, WBRT,
                              // WBLT, NBRT, NBLT, EBRT, EBLT

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Lane_Movement_Mapping_File problem");
        sh.close();
        return false;
    }

    char tmp[128];
    strcpy(tmp, temp_string.c_str());
    sscanf(tmp, "%d %d %d %d %d %d %d %d", &m_laneMapping[0], &m_laneMapping[1], &m_laneMapping[2],
        &m_laneMapping[3], &m_laneMapping[4], &m_laneMapping[5], &m_laneMapping[6],
        &m_laneMapping[7]);

    getline(sh, temp_string); // Third line is comment explaining the order of the Movements
    getline(sh, temp_string); // Fourth line shows the corresponding movement numbers

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Lane_Movement_Mapping_File problem");
        sh.close();
        return false;
    }

    strcpy(tmp, temp_string.c_str());
    sscanf(tmp, "%d %d %d %d %d %d %d %d %d %d %d %d", &m_movementMapping[0], &m_movementMapping[1],
        &m_movementMapping[2], &m_movementMapping[3], &m_movementMapping[4], &m_movementMapping[5],
        &m_movementMapping[6], &m_movementMapping[7], &m_movementMapping[8], &m_movementMapping[9],
        &m_movementMapping[10], &m_movementMapping[11]);

    sh.close();
    return true;
}

bool MmitssPerformanceObserver::getDetNumbers()
{
    fstream sh;
    sh.open(SieMmitss::getDetNumbersFilePath().c_str());
    char tmp[128];

    string temp_string;

    getline(sh, temp_string); // Reading the first line which explains the detectors assignment
    getline(sh, temp_string); // Read the second line which indicate the number of detectors

    sscanf(temp_string.c_str(), "%d", &m_totNumDet);
    ITSAPP_TRACE("m_totNumDet is %d", m_totNumDet);

    m_detNumbers.resize(m_totNumDet, 0);

    getline(sh, temp_string); // Reading the third line which has the Detector Number for each lane

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Det_Numbers_file problem!!");
        sh.close();
        return false;
    }

    strncpy(tmp, temp_string.c_str(), temp_string.size());

    char* pch;
    pch = strtok(tmp, " ");

    for (int i = 0; i < m_totNumDet; i++)
    {
        sscanf(pch, "%d", &m_detNumbers[i]);
        // printf("%d\n",Det_Number[i]);
        pch = strtok(NULL, " ,.-");
    }

    getline(sh, temp_string); // Reading the fourth line which explains the Approach & Lane
                              // Combination for Distance from System Detectors to stop bar
    getline(sh, temp_string); // Reading the fifth line which has the values for the corresponding
                              // approach and lane

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Stop bar distance of detectors problem!!");
        sh.close();
        return false;
    }

    strcpy(tmp, temp_string.c_str());

    pch = strtok(tmp, " ");
    for (int i = 0; i < m_totNumDet; i++) // going through approaches 1,3,5,7 and lanes 1,2,3,4,5
    {
        sscanf(pch, "%f", &m_detectorStopbarDist[i]);
        ITSAPP_TRACE("m_detectorStopbarDist[%d] is %f", i, (double)m_detectorStopbarDist[i]);
        // printf("%.2f\n",m_detectorStopbarDist[i]);
        pch = strtok(NULL, " ,-");
    }

    getline(sh, temp_string); // Reading the sixth line which explains the Association between the
                              // Approach & Lane Combination and Phases
    getline(sh,
        temp_string); // Reading the seventh line which has the values for the corresponding phases

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Phase Lane Mapping problem!!");
        sh.close();
        return false;
    }

    strcpy(tmp, temp_string.c_str());
    pch = strtok(tmp, " ");
    for (int i = 0; i < m_totNumDet; i++) // going through approaches 1,3,5,7 and lanes 1,2,3,4,5
    {
        sscanf(pch, "%d", &m_lanePhase[i]);
        ITSAPP_TRACE("m_lanePhase[%d] is %d", i, m_lanePhase[i]);
        // printf("%.2f\n",m_lanePhase[i]);
        pch = strtok(NULL, " ,-");
    }

    getline(
        sh, temp_string); // Reading the eighth line which explains the Turning Movement Proportions
    for (int i = 0; i < m_totNumDet; i++) // Reading line by line the next 20 lines which has the
                                          // values for the corresponding TMPs
    {
        getline(sh, temp_string);
        if ((temp_string.size() == 0) || (temp_string.size() >= 128))
        {
            ITSAPP_WRN("Reading Turning Movement Proportions problem!!");
            sh.close();
            return false;
        }

        strcpy(tmp, temp_string.c_str());
        ITSAPP_TRACE("Reading %s", tmp);
        pch = strtok(tmp, " ");

        for (int j = 0; j < 3; j++)
        {
            sscanf(pch, "%f", &m_turningProportion[i][j]);
            ITSAPP_TRACE(
                "m_turningProportion[%d][%d] is %f", i, j, (double)m_turningProportion[i][j]);
            // printf("%f\n",m_turningProportion[i][j]);
            pch = strtok(NULL, " ,-");
        }

        /*sscanf(pch,"%f %f
        %f",&m_turningProportion[i][0],&m_turningProportion[i][1],&m_turningProportion[i][2]);
        printf("%f %f
        %f\n",m_turningProportion[i][0],m_turningProportion[i][1],m_turningProportion[i][2]);
        pch=strtok(NULL," ,-");
        */
    }
    // printf("%f\n",m_turningProportion[7][1]); //For checking if it reads properly or not
    sh.close();
    return true;
}

void obtainEndofService(
    int ihour, int imin, int isec, int iETA, int& iHour, int& iMin, int& iSec) // Adde by Mehdi
{
    if ((iETA >= 0) && (iETA < 59))
    {
        if (imin == 59)
        {
            if ((isec + iETA) >= 60)
            {
                if (ihour == 23)
                {
                    iHour = 0;
                    iMin = 0;
                    iSec = isec + iETA - 60;
                }
                else
                {
                    iHour = ihour + 1;
                    iMin = 0;
                    iSec = isec + iETA - 60;
                }
            }
            else
            {
                iHour = ihour;
                iMin = imin;
                iSec = isec + iETA;
            }
        }
        else
        {
            if ((isec + iETA) >= 60)
            {
                iHour = ihour;
                iMin = imin + 1;
                iSec = isec + iETA - 60;
            }
            else
            {
                iHour = ihour;
                iMin = imin;
                iSec = isec + iETA;
            }
        }
    }
    if ((iETA >= 60) && (iETA < 119))
    {
        if (imin == 59)
        {
            if ((isec + iETA) < 120)
            {
                if (ihour == 23)
                {
                    iHour = 0;
                    iMin = 0;
                    iSec = isec + iETA - 60;
                }
                else
                {
                    iHour = ihour + 1;
                    iMin = 0;
                    iSec = isec + iETA - 60;
                }
            }
            else
            {
                if (ihour == 23)
                {
                    iHour = 0;
                    iMin = 1;
                    iSec = isec + iETA - 120;
                }
                else
                {
                    iHour = ihour + 1;
                    iMin = 1;
                    iSec = isec + iETA - 120;
                }
            }
        }
        else if (imin == 58)
        {
            if ((isec + iETA) < 120)
            {
                iHour = ihour;
                iMin = 59;
                iSec = isec + iETA - 60;
            }
            else
            {
                if (ihour == 23)
                {
                    iHour = 0;
                    iMin = 0;
                    iSec = isec + iETA - 120;
                }
                else
                {
                    iHour = ihour + 1;
                    iMin = 0;
                    iSec = isec + iETA - 120;
                }
            }
        }
        else if (imin < 58)
        {
            if ((isec + iETA) < 120)
            {
                iHour = ihour;
                iMin = imin + 1;
                iSec = isec + iETA - 60;
            }
            else
            {
                iHour = ihour;
                iMin = imin + 2;
                iSec = isec + iETA - 120;
            }
        }
    }
}

} // namespace Mmitss
} // namespace WaveApp
