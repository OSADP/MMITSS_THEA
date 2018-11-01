/* Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* mprSolver.cpp
 * Created by Mehdi Zamanipour
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 * 1. For saturated condition, use -s 1 argument
 *
 */

#include "mmitssPrioritySolver.hpp"

#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <netinet/in.h>
#include <Poco/Net/SocketAddress.h>
#include <sigc++/functors/mem_fun.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>

#include "glpk.h"
#include "Array.h"
#include "EVLS.h"
#include "GetInfo.h"
#include "Print2Log.h"

#include "facilityBase.hpp"
#include "facilityDatabaseAdapter.hpp"
#include "facilityLayer.hpp"
#include "SieMmitss.hpp"
#include "SieMmitssConfig.hpp"
#include "mmitssPrioritySolverApp.hpp"

//---- The following two for function RunScheduleG()
#define NEW 1     //??
#define REGULAR 0 //??

#ifndef DEG2ASNunits
#define DEG2ASNunits (1000000.0) // used for ASN 1/10 MICRO deg to unit converts
#endif

// Lower bound in second: add to the ETA for ROBUST control (should be positive)
#define ROBUSTTIME_LOW 2
// Upper bound in second: add to the ETA for ROBUST control
#define ROBUSTTIME_UP 4

// Lower bound in second: add to the ETA for ROBUST control (should be positive)
#define ROBUSTTIME_LOW_EV 4
// Upper bound in second: add to the ETA for ROBUST control
#define ROBUSTTIME_UP_EV 3

#define MaxGrnTime 50 // ONLY change the phase {2,4,6,8} for EV requested phases

namespace WaveApp
{
namespace Mmitss
{

int numberOfPlannedPhase(double adCritPont[3][5], int rng);
double findLargestCP(double cp[3][5], int rng, int maxNumOfPlannedPhases);
int FindRingNo(int phase);
// find the position of the request in the PriorityRequestList that has the earliest arrival
// time among the requests in (Earliest Requested Phase ) (ERP) requests set !)

// this is the value mmitss used, TODO: check if that is anyhow reasonable at all
static const int TRAJ_RX_BUF_SIZE = 500000;
static const int OPT_MSG_SIZE = 256;

MmitssPrioritySolver::MmitssPrioritySolver(MmitssPrioritySolverApp& parent)
    : m_parent(parent)
    , m_udpReceiver(::SieMmitss::getPrioSolverPort(),
          TRAJ_RX_BUF_SIZE,
          [this](const void* data, size_t len) { return this->onUdpReceive(data, len); })
    , m_reqListCombined()
    , m_cfg()
    , m_phases(m_cfg)
    , m_mib()
{
}

bool MmitssPrioritySolver::onStartup()
{
    return mmitssInit();
}

void MmitssPrioritySolver::onShutdown()
{
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    m_udpReceiver.stop();
}

void MmitssPrioritySolver::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

bool MmitssPrioritySolver::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    // --- getting the configurations
    m_mib.setControllerAddress(get_ip_address());
    // Get the current RSU ID from "rsuid.txt" into string "m_rsuId"
    m_rsuId = get_rsu_id();
    m_reqListCombined.setRSUID(m_rsuId);

    auto peneration = 100;
    ITSAPP_VERIFY(m_parent.database().read("its/us/mmitss/control/penetration", peneration));

    m_penetrationRate = static_cast<float>(peneration) / 100;

    //-------(1) Read the name of "configinfo_XX.txt" into "ConfigFile" from
    // ConfigInfo.txt--------//
    //-------(2) Read in the CombinedPhase[8] for finding the split phases--
    // get_configfile();

    if (!setupConnection())
        return false;

    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });
    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);

    return true;
}

void MmitssPrioritySolver::onUdpReceive(const void* data, size_t len)
{
    if ((data == nullptr) || (len == 0))
        return;

    ITSAPP_LOG("Received Trajectory Data!");
    m_lastAckCount = m_reqCount;
    ITSAPP_TRACE("m_lastAckCount: %d", m_lastAckCount);
    m_trackedVeh.clear(); // clear the list for new set of data

    // unpack the trajectory data and save to m_trackedVeh list
    UnpackTrajData1(reinterpret_cast<const char*>(data));
    ITSAPP_LOG("The Number of Vehicle Received is: %d", m_trackedVeh.size());

    if (m_penetrationRate >= 0.95f) // construct arrival table directly
    {
        calculateQ();
        extractOptModInputFromTraj();
    }
    else // calling EVLS
    {
        // Algorithm to infer vehicle location and speed based on sample vehicle data and create
        // arrival table
        ITSAPP_LOG("Estimating Location and Position of Unequipped Vehicle:");
        for (int i = 0; i < 8; i++)
        {

            // Assign vehicles of each phase to the m_vehlistEachPhase

            m_vehListEachPhase.clear();
            for (const auto& veh : m_trackedVeh)
            {
                if (veh.req_phase == i + 1)
                {
                    m_vehListEachPhase.emplace_back(veh);
                }
            }

            ITSAPP_LOG("Phase: %d", i + 1);

            // sprintf(temp_log,"The input list is:\n");
            // outputlog(temp_log); cout << temp_log;
            for (const auto& veh : m_vehListEachPhase)
            {
                ITSAPP_LOG("%d %lf %lf %lf %d %d %d %lf\n", veh.TempID, veh.Speed,
                    veh.stopBarDistance, veh.acceleration, veh.approach, veh.lane, veh.req_phase,
                    veh.time_stop);
            }

            ITSAPP_LOG("The number of lanes is %d", m_laneNo[i]);

            if (m_vehListEachPhase.size() != 0) // There is vehicle in the list
                EVLS_Prio(m_vehListEachPhase, m_trackedVeh, i + 1, m_timeRequestTraj,
                    m_redElapseTime[i], m_laneNo[i], m_dsrcRange, m_penetrationRate);
        }
        calculateQ();
        extractOptModInputFromTraj();
    }
    // construct .dat file for the glpk
    LinkList2DatFile(m_cfg);
    // Log the .dat file for glpk solver
    PrintFile2Log(SieMmitss::getPriorityDataFilePath());
    double t1_Solve = GetSeconds();
    GLPKSolver();
    double t2_Solve = GetSeconds();
    ITSAPP_LOG("Time for solving the problem is about: {%.3f}.", t2_Solve - t1_Solve);
    int success2 = GLPKSolutionValidation();
    ITSAPP_LOG("success2 %d", success2);
    if (success2 == 0)
    {

        m_eventListR1.clear();
        m_eventListR2.clear();
        readOptPlanFromFile();
        deductSolvingTimeFromCPs(t2_Solve, t1_Solve);
        Construct_eventlist();

        char tmp_event_data[500];
        int size = 0;
        Pack_Event_List(tmp_event_data, size);
        m_recvAddr.sin_port = htons(SieMmitss::getTrafficControllerInterfacePort());
        if (sendto(m_sockfd, tmp_event_data, size + 1, 0, (struct sockaddr*)&m_recvAddr,
                sizeof(m_recvAddr)))
        {
            ITSAPP_LOG(
                " The new Event List sent to SignalControllerInterface, The size is %d and time is "
                ": %.2f.......... ",
                size, GetSeconds());
            std::string actionTable[] = {
                "ForceOff", "Omit", "VehCall", "Hold", "PedCall", "PedClear"};

            for (auto& event : m_eventListR1)
            {
                ITSAPP_LOG("Time: %lf Phase: %d Action: %s", event.time, event.phase,
                    actionTable[event.action].c_str());
            }
            for (auto& event : m_eventListR2)
            {
                ITSAPP_LOG("Time: %lf Phase: %d Action: %s", event.time, event.phase,
                    actionTable[event.action].c_str());
            }
        }
    }
}

bool MmitssPrioritySolver::updatePhaseAndElapseTime()
{
    const std::string resultsfile = SieMmitss::getGlpkResultsFilePath();
    const std::string prioritydatafile = SieMmitss::getPriorityDataFilePath();
    auto t_1 = GetSeconds();
    m_mib.PhaseTimingStatusRead();
    auto t_2 = GetSeconds();
    if ((t_2 - t_1) < 2.0) // TODO: This doesn't seem the right way to check for a conenction issue
    {                      // We do connect to ATC
        m_phases.UpdatePhaseMPR(m_mib.getPhaseRead(), m_mib.getPhaseStatus(), m_globalGmax);
        ////------------Begin of recording the phase_status into signal_status.txt----------------//
        // Phases.RecordPhase(signal_plan_file);
        for (int i = 0; i < 2; i++)
        {
            //{1-8}
            m_initPhase[i] = m_phases.InitPhase[i] + 1;
            // ALSO is the (Yellow+Red) Left for the counting down time ***Important***
            m_initTime[i] = m_phases.InitTime[i];
            // If in Green
            m_grnElapse[i] = m_phases.GrnElapse[i];
        }
        // Calculate red elapse time for each phase, if the signal turns to green,
        // keep the previous red duration until turns to red again (TO BE USED IN ELVS)
        for (int i = 0; i < 8; i++)
        {
            if ((m_previousSignalColor[i] != 1) && (m_mib.getPhaseRead().phaseColor[i] == Red))
            { // signal changed from other to red
                m_redStartTime[i] = GetSeconds();
            }
            if (m_mib.getPhaseRead().phaseColor[i] == Red)
            {
                m_redElapseTime[i] = GetSeconds() - m_redStartTime[i];
            }
            m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
        }
        return true;
    }
    return false;
}
void MmitssPrioritySolver::onTimer(siekdbus::timerid_t id)
{
    NOTUSED(id);
    if (not m_started)
        return;
    updatePhaseAndElapseTime();
}

void MmitssPrioritySolver::solve(std::string&& reqListJson)
{
    m_reqListCombined.buildRequestListFromJson(reqListJson);
    if (m_reqListCombined.empty())
        return;

    const std::string resultsfile = SieMmitss::getGlpkResultsFilePath();
    const std::string prioritydatafile = SieMmitss::getPriorityDataFilePath();

    if (updatePhaseAndElapseTime() == true)
    {

        // If there is EV, we need to generate a new model file and new data file
        ITSAPP_LOG("...............Solve the problem..............");
        ITSAPP_LOG("Requests list size :%d", m_reqListCombined.size());

        ITSAPP_LOG("Requests Modified : at %s", GetDate());
        m_haveEVInList = m_reqListCombined.FindVehClassInList(VEH_PRIORITY::EV);
        m_haveCoordInList = m_reqListCombined.FindVehClassInList(VEH_PRIORITY::COORDINATION);
        m_haveTruckInList = m_reqListCombined.FindVehClassInList(VEH_PRIORITY::TRUCK);
        m_haveTransitInList = m_reqListCombined.FindVehClassInList(VEH_PRIORITY::TRANSIT);
        m_havePedCall = m_reqListCombined.FindVehClassInList(VEH_PRIORITY::PEDESTRIAN);

        if (m_haveEVInList)
        {
            ITSAPP_LOG("...... We have EV. Need Dynamic Mod file......\t  At time: %s", GetDate());
            //----------------------------------------Begin to generate the dynamic mod
            // file----------------------------------------//
            int size_init = 2; //--- We always have two initial phases: if has{1,2,6,8}, timing
                               // 8, initial will be {4,8}
            //---------------- EV phase vector------------------
            vector<int> EV_Phase_vc = m_reqListCombined.getEVPhase();
            int EV_Phase_size = EV_Phase_vc.size();
            int ReqPhase = 0;
            int PhaseVCMissing = 0;
            int PhaseInitMissing = 0;
            for (int i = 0; i < EV_Phase_size; i++)
            {
                ReqPhase = EV_Phase_vc[i];
                PhaseVCMissing =
                    BarrierPhaseIsMissing(ReqPhase, EV_Phase_vc); // phase missing in the vector
                PhaseInitMissing = BarrierPhaseIsMissing(ReqPhase, m_initPhase, 2);
                if (PhaseInitMissing != 0 && PhaseVCMissing != 0)
                { // Then they should be the same
                    EV_Phase_vc.push_back(PhaseInitMissing);
                    if (m_addPhantom > 0)
                        m_reqListCombined.addPhantomReqEntry(ReqPhase, PhaseVCMissing);
                }
            }
            EV_Phase_size = EV_Phase_vc.size(); //  EV_Phase_size could be changed.
            int TotalSize = size_init + EV_Phase_size;
            int* Phase_Infom = new int[TotalSize];
            for (int i = 0; i < EV_Phase_size; i++)
                Phase_Infom[i] = EV_Phase_vc[i];
            for (int i = 0; i < size_init; i++)
                Phase_Infom[EV_Phase_size + i] = m_initPhase[i];
            // Sort all the involved phases for removing duplicated phases
            selectionSort(Phase_Infom, TotalSize);
            int NoRepeatSize = removeDuplicates(Phase_Infom, TotalSize);
            m_cfg.RSUConfig2ConfigFile(
                SieMmitss::getConfigInfoEVFilePath(), Phase_Infom, NoRepeatSize);
            PrintFile2Log(SieMmitss::getConfigInfoEVFilePath().c_str());
            delete[] Phase_Infom;

            // We need all {2,4,6,8}
            GenerateMod(SieMmitss::getConfigInfoEVFilePath(), SieMmitss::getModEVFilePath(),
                m_haveEVInList);

            m_cfg_EV = RsuConfig(SieMmitss::getConfigInfoEVFilePath(), true);

            m_cfg_EV.PrintRSUConfig();
            // construct .dat file for the glpk
            LinkList2DatFileForEV(m_cfg_EV, m_haveEVInList);
            PrintFile2Log(prioritydatafile);
            if ((PhaseVCMissing != 0) && (PhaseInitMissing == 0))
            {
                m_reqListCombined.deleteRequestPhaseInList(PhaseVCMissing);
            }
            //----------------------------------------End of generating the dynamic mod file for
            // EV case --------------------------------------//
        }
        if ((m_haveEVInList == false) &&
            (m_reqListCombined.size() >
                0)) // Atleast one priority vehicle except EV is in the list!
        {           // construct .dat file for the glpk
            LinkList2DatFile(m_cfg);
            PrintFile2Log(prioritydatafile); // Log the .dat file for glpk solver
        }                                    // End of "if(HaveEVInList==1)"

        double t1 = GetSeconds();
        GLPKSolver();
        double t2 = GetSeconds();
        ITSAPP_LOG("Time for solving the problem is about: {%.3f}.", t2 - t1);
        if (GLPKSolutionValidation() == 0)
        {
            ITSAPP_LOG("...............New plan..............:\t At time: %.2f", GetSeconds());
            PrintPlan2Log(resultsfile);
            if (m_haveEVInList)
                readOptPlanFromFileForEV(m_cfg_EV);
            else
                readOptPlanFromFile();

            deductSolvingTimeFromCPs(t2, t1);

            // If the Solver works jointly with COP, we pack the Critical Points and send it to
            // COP, othewise we transfer the CP to event list and send the event list to
            // controller.
            if (m_codeUsage == PRIORITY) // Send to Interface
            {
                m_eventListR1.clear();
                m_eventListR2.clear();

                if (m_haveEVInList)
                {
                    matchTheBarrierInCPs_EV();
                    Construct_eventlist_EV();
                }
                else
                {
                    Construct_eventlist();
                    matchTheBarrierInCPs();
                }
                char tmp_event_data[500];
                int size = 0;
                Pack_Event_List(tmp_event_data, size);
                m_recvAddr.sin_port = htons(SieMmitss::getTrafficControllerInterfacePort());
                if (sendto(m_sockfd, tmp_event_data, size + 1, 0, (struct sockaddr*)&m_recvAddr,
                        sizeof(m_recvAddr)))
                {
                    ITSAPP_LOG(
                        " The new Event List sent to SignalControllerInterface, The size is %d "
                        "and time is : %.2f..........",
                        size, GetSeconds());
                    std::string actionTable[] = {
                        "ForceOff", "Omit", "VehCall", "Hold", "PedCall", "PedClear"};

                    for (auto& event : m_eventListR1)
                    {
                        ITSAPP_LOG("Time: %lf Phase: %d Action: %s", event.time, event.phase,
                            actionTable[event.action].c_str());
                    }
                    for (auto& event : m_eventListR2)
                    {
                        ITSAPP_LOG("Time: %lf Phase: %d Action: %s", event.time, event.phase,
                            actionTable[event.action].c_str());
                    }
                }
            }
            else if (m_codeUsage == COP_AND_PRIORITY) // Send to COP
            {
                char optBuf[OPT_MSG_SIZE];
                m_optMsgCnt++;
                m_optMsgCnt = m_optMsgCnt % 127;
                packOPT(optBuf);
                m_recvAddr.sin_port = htons(SieMmitss::getSignalControlOPTPort());
                if ((sendto(m_sockfd, optBuf, OPT_MSG_SIZE, 0, (struct sockaddr*)&m_recvAddr,
                         sizeof(m_recvAddr)) > 0))
                {
                    ITSAPP_LOG(
                        " The new OPT set sent to SignalControl (COP) At time: %.2f..........",
                        GetSeconds());
                }
            }
            printCriticalPoints();
        }
        else
        {
            ITSAPP_WRN("No feasible solution!");
        }
        //        else if (m_codeUsage == ADAPTIVE_PRIORITY) // Integrated Priority Alg and Adaptive
        //        Control
        //        {
        //            if ((t_2 - m_timeRequestTraj) > 4)     // if is it time to get arrival table
        //            from traj. aware componet and solve the problem. It is supposed to get the
        //            arrival table every 2 seconds
        //            {
        //                requestTrajectory();
        //            } //end of if (t_2-t_RequestTraj>4)     // if is it time to get arrival table
        //            from traj. aware componet and solve the problem. It is supposed to get the
        //            arrival table every 2 seconds
        //        }
        //        else   // if there is the priority flag is zero and if the codeusage is Priority
        //        Control and Actuation
        //        {
        //            //ITSAPP_LOG("No Need to solve, At time: %s",GetDate());
        //            std::this_thread::sleep_for(100ms);
        //        }
    }
    else // // We do not connect to RSE
    {
        ITSAPP_WRN("******Cannot connect to ATC, At time: [%.2f].", GetSeconds());
    } // end of  if( (t_2-t_1)<2.0 ) // We do connect to RSE
}

void MmitssPrioritySolver::requestTrajectory()
{
    m_timeRequestTraj = GetSeconds();

    std::ostringstream reqStr;
    reqStr << "Request priority_solver "; // leave the blank at the end
    reqStr << m_reqCount;

    std::string req = reqStr.str();
    ITSAPP_TRACE("m_reqCount: %d,m_lastAckCount: %d", m_reqCount, m_lastAckCount);
    if (m_reqCount - m_lastAckCount < MAX_UNACK_REQ)
    {
        ITSAPP_TRACE("Send: %s", req.c_str());
        // send request for trajectory data
        m_recvAddr.sin_port = htons(SieMmitss::getTrajectoryAwarePort());
        sendto(m_sockfd, req.c_str(), req.length() + 1, 0, (struct sockaddr*)&m_recvAddr,
            sizeof(m_recvAddr));
    }
    else
    {
        ITSAPP_TRACE("restart receiver");
        // restart receiver
        m_udpReceiver.stop();
        m_udpReceiver.start();
        m_lastAckCount = m_reqCount;
    }
    m_reqCount++;
}

bool MmitssPrioritySolver::GenerateMod(
    const std::string& Filename, const std::string& OutFilename, int haveEVinList)
{
    //-------reading the signal configuration file ---------
    fstream FileRead;
    FileRead.open(Filename, ios::in);
    if (!FileRead)
    {
        ITSAPP_WRN("Unable to open file: %s", Filename.c_str());
        return false;
    }

    string lineread;

    int PhaseNo;
    char TempStr[16];
    vector<int> P11, P12, P21, P22;
    int PhaseSeq[8], Gmin[8], Gmax[8];
    float Yellow[8], Red[8]; // If

    for (int i = 1; i < 8; i = i + 2) // Add {2,4,6,8} into the PhaseSeq: NECESSARY Phases
    {
        PhaseSeq[i] = i + 1;
    }
    //------- Read in the parameters--
    while (!FileRead.eof())
    {
        getline(FileRead, lineread);

        if (lineread.size() != 0)
        {
            sscanf(lineread.c_str(), "%s", TempStr);

            if (strcmp(TempStr, "Phase_Num") == 0)
            {
                sscanf(lineread.c_str(), "%s %d", TempStr, &PhaseNo);
                ITSAPP_LOG("Total Phase Number is: %d", PhaseNo);
            }

            else if (strcmp(TempStr, "Phase_Seq") == 0)
            {
                // sscanf(lineread.c_str(),"%*s %d %d %d %d %d %d %d %d",
                //    &PhaseSeq[0],&PhaseSeq[1],&PhaseSeq[2],&PhaseSeq[3],
                //    &PhaseSeq[4],&PhaseSeq[5],&PhaseSeq[6],&PhaseSeq[7]);
                sscanf(lineread.c_str(), "%*s %d %*d %d %*d %d %*d %d %*d", &PhaseSeq[0],
                    &PhaseSeq[2], &PhaseSeq[4], &PhaseSeq[6]);
            }
            else if (strcmp(TempStr, "Yellow") == 0)
            {
                sscanf(lineread.c_str(), "%*s %f %f %f %f %f %f %f %f", &Yellow[0], &Yellow[1],
                    &Yellow[2], &Yellow[3], &Yellow[4], &Yellow[5], &Yellow[6], &Yellow[7]);
            }
            else if (strcmp(TempStr, "Red") == 0)
            {
                sscanf(lineread.c_str(), "%*s %f %f %f %f %f %f %f %f", &Red[0], &Red[1], &Red[2],
                    &Red[3], &Red[4], &Red[5], &Red[6], &Red[7]);
            }
            else if (strcmp(TempStr, "Gmin") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d %d %d %d %d %d %d %d", &Gmin[0], &Gmin[1],
                    &Gmin[2], &Gmin[3], &Gmin[4], &Gmin[5], &Gmin[6], &Gmin[7]);
            }
            else if (strcmp(TempStr, "Gmax") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d %d %d %d %d %d %d %d", &Gmax[0], &Gmax[1],
                    &Gmax[2], &Gmax[3], &Gmax[4], &Gmax[5], &Gmax[6], &Gmax[7]);
            }
        }
    }

    FileRead.close();

    //-------------Handle the parameters for non-complete phases case----//
    for (int i = 0; i < 8; i++)
    {
        if (PhaseSeq[i] > 0)
        {
            switch (PhaseSeq[i])
            {
                case 1:
                case 2:
                    P11.push_back(PhaseSeq[i]);
                    break;
                case 3:
                case 4:
                    P12.push_back(PhaseSeq[i]);
                    break;
                case 5:
                case 6:
                    P21.push_back(PhaseSeq[i]);
                    break;
                case 7:
                case 8:
                    P22.push_back(PhaseSeq[i]);
                    break;
            }
        }
    }

    //-------READING the priority Configuratio file ---------------
    double dCoordinationWeight;
    int iCoordinatedPhase[2];
    int iCoordinationCycle;
    dCoordinationWeight = m_cfg.getConfig().dCoordinationWeight;
    iCoordinatedPhase[0] = m_cfg.getConfig().iCoordinatedPhase[0];
    iCoordinatedPhase[1] = m_cfg.getConfig().iCoordinatedPhase[1];
    iCoordinationCycle = m_cfg.getConfig().iCoordinationCycle;

    // ---------------- Writing the .mod  file------------------
    fstream FileMod;
    FileMod.open(OutFilename, ios::out);

    if (!FileMod)
    {
        ITSAPP_WRN("Cannot open file: NewModel.mod to write!");
        return false;
    }

    int PhaseSeqArray[8];
    int kk = 0;

    if (P11.size() == 1)
    {
        FileMod << "set P11:={" << P11[0] << "}; \n";
        PhaseSeqArray[kk] = P11[0];
        kk++;
    }
    else if (P11.size() == 2)
    {
        FileMod << "set P11:={" << P11[0] << "," << P11[1] << "};  \n";
        PhaseSeqArray[kk] = P11[0];
        kk++;
        PhaseSeqArray[kk] = P11[1];
        kk++;
    }

    if (P12.size() == 1)
    {
        FileMod << "set P12:={" << P12[0] << "}; \n";
        PhaseSeqArray[kk] = P12[0];
        kk++;
    }
    else if (P12.size() == 2)
    {
        FileMod << "set P12:={" << P12[0] << "," << P12[1] << "};\n";
        PhaseSeqArray[kk] = P12[0];
        kk++;
        PhaseSeqArray[kk] = P12[1];
        kk++;
    }

    if (P21.size() == 1)
    {
        FileMod << "set P21:={" << P21[0] << "};\n";
        PhaseSeqArray[kk] = P21[0];
        kk++;
    }
    else if (P21.size() == 2)
    {
        FileMod << "set P21:={" << P21[0] << "," << P21[1] << "};\n";
        PhaseSeqArray[kk] = P21[0];
        kk++;
        PhaseSeqArray[kk] = P21[1];
        kk++;
    }

    if (P22.size() == 1)
    {
        FileMod << "set P22:={" << P22[0] << "};\n";
        PhaseSeqArray[kk] = P22[0];
        kk++;
    }
    else if (P22.size() == 2)
    {
        FileMod << "set P22:={" << P22[0] << "," << P22[1] << "};\n";
        PhaseSeqArray[kk] = P22[0];
        kk++;
        PhaseSeqArray[kk] = P22[1];
        kk++;
    }

    FileMod << "set P:={";

    for (int i = 0; i < kk; i++)
    {
        if (i != kk - 1)
        {
            FileMod << " " << PhaseSeqArray[i] << ",";
        }
        else
        {
            FileMod << " " << PhaseSeqArray[i];
        }
    }

    FileMod << "};\n";

    //------------- Print the Main body of mode----------------
    FileMod << "set K  := {1,2};\n";
    FileMod << "set J  := {1..10};\n";
    if (m_codeUsage ==
        2) // If arrival table is considered ( in Integrated Priority and Adaptive Control)
    {
        FileMod << "set I  :={1..130};\n";
        FileMod << "set L  :={1..20};\n";
    }
    FileMod << "set P2  :={1..8};\n";

    FileMod << "set T  := {1..10};\n"; // at most 10 different types of vehicle may be considered ,
                                       // EV are 1, Transit are 2, Trucks are 3  \n";
    FileMod << "set C  := {1..4};\n"; // at most 4 rcoordination requests may be considered, 2 phase
                                      // * 2 cycles ahead

    FileMod << "  \n";
    FileMod << "param y    {p in P}, >=0,default 0;\n";
    FileMod << "param red  {p in P}, >=0,default 0;\n";
    FileMod << "param gmin {p in P}, >=0,default 0;\n";
    FileMod << "param gmax {p in P}, >=0,default 0;\n";
    FileMod << "param init1,default 0;\n";
    FileMod << "param init2,default 0;\n";
    FileMod << "param Grn1, default 0;\n";
    FileMod << "param Grn2, default 0;\n";
    FileMod << "param SP1,  integer,default 0;\n";
    FileMod << "param SP2,  integer,default 0;\n";
    FileMod << "param M:=9999,integer;\n";
    FileMod << "param alpha:=100,integer;\n";
    FileMod << "param Rl{p in P, j in J}, >=0,  default 0;\n";
    FileMod << "param Ru{p in P, j in J}, >=0,  default 0;\n";
    FileMod << "param Cl{p in P, c in C}, >=0,  default 0;\n";
    FileMod << "param Cu{p in P, c in C}, >=0,  default 0;\n";
    FileMod << "  \n";

    if (dCoordinationWeight < 0)
        dCoordinationWeight = 0.0;
    if (iCoordinatedPhase[0] ==
        0) // in the case priorityConfiguration.txt file is not set properly!
        iCoordinatedPhase[0] = 2;
    if (iCoordinatedPhase[1] == 0)
        iCoordinatedPhase[1] = 6;
    if (haveEVinList == 1)
        dCoordinationWeight = 0;

    FileMod << "param current,>=0,default 0;\n"; // # current is  mod(time,cycle)
    FileMod << "param coord, :=" << dCoordinationWeight
            << ";\n"; //     # this paramter indicated where we consider coorrdination or not
    FileMod << "param isItCoordinated, :=(if coord>0 then 1 else 0);\n";
    FileMod
        << "param SumOfMinGreen1, :=  ( sum{p in P11} gmin[p] ) + ( sum{p in P12} gmin[p] ) ;\n";
    FileMod
        << "param SumOfMinGreen2, :=  ( sum{p in P21} gmin[p] ) + ( sum{p in P22} gmin[p] ) ;\n";
    FileMod << "param cycle, :=" << iCoordinationCycle
            << ";\n"; //    # if we have coordination, the cycle length
    // FileMod << "param offset,:=" << iCoordinationOffset  << ";   # if we have coordination, the
    // offset  \n";
    // FileMod << "param split, :=" << iCoordinationSplit   << ";    # if we have coordination, the
    // split  \n";
    FileMod << "param coordphase1,:=" << iCoordinatedPhase[0]
            << ";\n"; //    # if we have coordination, thecoordinated phase in ring1
    FileMod << "param coordphase2,:=" << iCoordinatedPhase[1]
            << ";\n"; //    # if we have coordination, thecoordinated phase in ring2
    FileMod << "param isCurPhCoord1, :=(if coordphase1=SP1 then 1 else 0);\n";
    FileMod << "param isCurPhCoord2, :=(if coordphase2=SP2 then 1 else 0);\n";
    // FileMod << "param earlyReturn,  := (if ( ( isCurPhCoord1=1 and ( current>iCoordinationSplit)
    // ) then 1 else 0); #  if we are in earlier coordinated phase green time, this earlyReturnPhase
    // is 1 \n";  // this is an importnt thereshold!!!
    // FileMod << "param earlyReturn,  := (if ( ( isItCoordinated=1 and ( current> (
    // gmax[coordphase1] + SumOfMinGreen1 - gmin[coordphase1] ) ) and coordphase1>0)  or
    // (isCurPhCoord2=1 and (current>( gmax[coordphase2] + SumOfMinGreen2 - gmin[coordphase2] )) and
    // coordphase2>0) ) then 1 else 0); #  if we are in earlier coordinated phase green time, this
    // earlyReturnPhase is 1 \n";  // this is an importnt thereshold!!!
    // FileMod << "param earlyReturnPhase2,  := (if ( (isCurPhCoord2=1 and (current>(
    // gmax[coordphase2] + SumOfMinGreen2 - gmin[coordphase2] ) and coordphase2>0) ) then 1 else 0);
    // \n";
    FileMod << "param earlyReturn,   := (if ( ( isItCoordinated=1 and ( current> 0.75*cycle ) and "
               "coordphase1>0)  or     (isItCoordinated=1 and ( current> 0.75*cycle ) and "
               "coordphase2>0) ) then 1 else 0); \n"; //  if we are in earlier coordinated phase
                                                      //  green time, this earlyReturnPhase is 1
    FileMod << "param afterSplit,    := (if ( ( isItCoordinated=1 and  Grn1>=gmax[coordphase1] and "
               "SP1=coordphase1 and current>= gmax[coordphase1]/"
            << 1 + m_maxGrnExtPortion + 0.1
            << " and current<= gmax[coordphase1])  or     (isItCoordinated=1 and ( "
               "Grn2>=gmax[coordphase2] ) and SP2=coordphase2 and current>= gmax[coordphase1]/"
            << 1 + m_maxGrnExtPortion + 0.1
            << " and current<=gmax[coordphase2]) ) then 1 else 0); \n";
    FileMod << "param inSplit,       := (if ( ( isItCoordinated=1 and  SP1=coordphase1 and "
               "current<gmax[coordphase1]/"
            << 1 + m_maxGrnExtPortion + 0.1
            << "  )  or (isItCoordinated=1 and SP2=coordphase2 and current< gmax[coordphase1]/"
            << 1 + m_maxGrnExtPortion + 0.1 << " ) ) then 1 else 0); \n";
    FileMod << "param PrioType { t in T}, >=0, default 0;  \n";
    FileMod << "param PrioWeigth { t in T}, >=0, default 0;  \n";
    FileMod << "param priorityType{j in J}, >=0, default 0;  \n";
    FileMod << "param priorityTypeWeigth{j in J, t in T}, := (if (priorityType[j]=t) then "
               "PrioWeigth[t] else 0);  \n";
    if (m_codeUsage ==
        2) // If arrival table is considered ( in Integrated Priority and Adaptive Control)
    {
        FileMod << "param s ,>=0,default 0;\n";
        FileMod << "param qSp ,>=0,default 0;\n";
        FileMod << "param Ar{i in I}, >=0,  default 0;\n";
        FileMod << "param Tq{i in I}, >=0,  default 0;\n";
        FileMod << "param active_arr{i in I}, integer, :=(if Ar[i]>0 then 1 else 0);\n";
        FileMod << "param SumOfActiveArr, := (if (sum{i in I} Ar[i])>0 then (sum{i in I} Ar[i]) "
                   "else 1);\n";
        FileMod << "param Ve{i in I}, >=0,  default 0;\n";
        FileMod << "param Ln{i in I}, >=0,  default 0;\n";
        FileMod << "param Ph{i in I}, >=0,  default 0;\n";
        FileMod << "param L0{l in L}, >=0,  default 0;\n";
        FileMod << "param LaPhSt{l in L}, integer;\n";
        FileMod << "param Ratio{i in I}, >=0,  default 0;\n";
        FileMod << "param indic{i in I},:= (if (Ratio[i]-L0[Ln[i]]/(s-qSp))> 0 then 1 else 0);\n";
        FileMod << "  \n";
    }
    FileMod << "param active_pj{p in P, j in J}, integer, :=(if Rl[p,j]>0 then 1 else    0);\n";
    FileMod
        << "param active_coord{p in P, c in C}, integer, :=(if Cl[p,c]>0 then    1 else    0);\n";
    FileMod << "param coef{p in P,k in K}, integer,:=(if ((((p<SP1 and p<5) or (p<SP2 and p>4 )) "
               "and k==1))then 0 else 1);\n";
    //    FileMod
    //        << "param MinGrn1{p in P,k in K},:=(if (((p==SP1 and p<5) and k==1))then Grn1 else
    //        0);\n";
    //    FileMod << "param MinGrn2{p in P,k in K},:=(if (((p==SP2 and p>4 ) and k==1))then    Grn2
    //    else "
    //               "0);\n";
    FileMod << "param PassedGrn1{p in P,k in K},:=(if (((p==SP1 and p<5) and k==1))then Grn1 else "
               "0);\n";
    FileMod << "param PassedGrn2{p in P,k in K},:=(if (((p==SP2 and p>4) and k==1))then Grn2 else "
               "0);\n";
    FileMod << "param sumOfGMax11, := sum{p in P11} (gmax[p]*coef[p,1]);\n";
    FileMod << "param sumOfGMax12, := sum{p in P12} (gmax[p]*coef[p,1]);\n";
    FileMod << "param sumOfGMax21, := sum{p in P21} (gmax[p]*coef[p,1]);\n";
    FileMod << "param sumOfGMax22, := sum{p in P22} (gmax[p]*coef[p,1]);\n";
    FileMod << "param barrier1GmaxSlack, := sumOfGMax11 - sumOfGMax21 ;\n";
    FileMod << "param barrier2GmaxSlack, := sumOfGMax12 - sumOfGMax22 ;\n";
    FileMod << "param gmaxSlack{p in P}, := if coef[p,1]=0 then 0 "
               "  else ( if (p in P11) then gmax[p]*max(0,-barrier1GmaxSlack)/sumOfGMax11"
               "  else ( if (p in P21) then gmax[p]*max(0,+barrier1GmaxSlack)/sumOfGMax21  "
               "  else ( if (p in P12) then gmax[p]*max(0,-barrier2GmaxSlack)/sumOfGMax12"
               "  else ( if (p in P22) then gmax[p]*max(0,barrier2GmaxSlack)/sumOfGMax22  "
               "  else 0) ) ) );\n";
    FileMod << "param gmax_with_barrier_slack{p in P,k in K}, := gmax[p]+gmaxSlack[p] ;\n";
    FileMod << "param gmax_revised{p in P,k in K}, := if (p==SP1 and "
               "gmax_with_barrier_slack[p,1]<PassedGrn1[p,1]) "
               " then gmax_with_barrier_slack[p,1]+PassedGrn1[p,1] "
               " else (if (p==SP2 and gmax_with_barrier_slack[p,1]<PassedGrn2[p,1]) "
               " then gmax_with_barrier_slack[p,1]+PassedGrn2[p,1] else  "
               "gmax_with_barrier_slack[p,k] ) ;\n";

    FileMod << "param ReqNo:=sum{p in P,j in J} active_pj[p,j] + isItCoordinated*sum{p in P,c in "
               "C} active_coord[p,c];  \n";

    // FileMod << "param queue_pj{p in P, j in J}, integer, :=(  if Tq[p,j]>0 then    1  else    0);
    // \n";
    FileMod << "  \n";
    FileMod << "var t{p in P,k in K}, >=0;    # starting time vector  \n";
    FileMod << "var g{p in P,k in K}, >=0;  \n";
    FileMod << "var v{p in P,k in K}, >=0;  \n";
    FileMod << "var d{p in P,j in J}, >=0;  \n";
    FileMod << "var theta{p in P,j in J}, binary;  \n";
    FileMod << "var ttheta{p in P,j in J}, >=0; \n";
    FileMod << "var PriorityDelay,>=0;  \n";
    FileMod << " \n";
    if (m_codeUsage ==
        2) // If arrival table is considered ( in Integrated Priority and Adaptive Control)
    {
        FileMod << "var miu{i in I}, binary; \n";
        FileMod << "var rd{i in I}, >=0; \n";
        FileMod << "var q{i in I}, >=0; \n";
        FileMod << "var qq{i in I}; \n";
        FileMod << "var qIndic{i in I}, binary; \n";
        FileMod << "var qmiu{i in I}, >=0; \n";
        FileMod << "var tmiu{ i in I}, >=0; \n";
        FileMod << "var gmiu{ i in I}, >=0; \n";
        FileMod << "var del{i in I}, binary; \n";
        FileMod << "var TotRegVehDel, >=0; \n";
        FileMod << "var delT1{i in I}, >=0; \n";
        FileMod << "var delT2{i in I}, >=0; \n";
        FileMod << "var ze{i in I}, binary; \n";
        FileMod << "var delZe{i in I}, binary; \n";
        FileMod << "var delZeT1{i in I}, >=0; \n";
        FileMod << "var delZeT2{i in I}, >=0; \n";

        // FileMod << "var dif1{i in I}, >=0; \n";
        // FileMod << "var dif2{i in I}, >=0; \n";
        // FileMod << "var dif3{i in I}, >=0; \n";
        // FileMod << "var dif1Bina{i in I}, binary; \n";
        // FileMod << "var dif2Bina{i in I}, binary; \n";
        // FileMod << "var dif3Bina{i in I}, binary; \n";
    }

    // FileMod << "var ttheta{p in P,j in J}, >=0; \n";
    // FileMod << "var dc1, >=0;\n";
    // FileMod << "var dc2, >=0;\n";
    FileMod << "var zeta1, binary;\n";
    FileMod << "var zetatl , >=0;\n";
    FileMod << "var zeta2, binary;\n";
    FileMod << "var zetatu , >=0;\n";

    FileMod << "var gamma1, binary;\n";
    FileMod << "var gammatl , >=0;\n";
    FileMod << "var gamma2, binary;\n";
    FileMod << "var gammatu , >=0;\n";

    FileMod << "var coord1delay , >=0;\n";
    FileMod << "var coord2delay , >=0;\n";

    FileMod << "  \n";

    //"#================ Begin of cycle 1======================#  \n";
    FileMod << "s.t. initial{p in P:(p<SP1) or (p<SP2 and p>4)}: t[p,1]=0;  \n";
    FileMod << "s.t. initial1{p in P:p=SP1}: t[p,1]=init1;  \n";
    FileMod << "s.t. initial2{p in P:p=SP2}: t[p,1]=init2;  \n";
    // FileMod << "\n # constraints in the same cycle in same P??  \n";
    FileMod
        << "s.t. Prec_11_11_c1{p in P11: (p+1)in P11 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod
        << "s.t. Prec_12_12_c1{p in P12: (p+1)in P12 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod
        << "s.t. Prec_21_21_c1{p in P21: (p+1)in P21 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod
        << "s.t. Prec_22_22_c1{p in P22: (p+1)in P22 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  \n";
    FileMod << "  \n";
    // FileMod << "# constraints in the same cycle in connecting   \n";
    FileMod
        << "s.t. Prec_11_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[2,1]+v[2,1];  \n";
    FileMod
        << "s.t. Prec_11_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[2,1]+v[2,1];  \n";
    FileMod
        << "s.t. Prec_21_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[6,1]+v[6,1];  \n";
    FileMod
        << "s.t. Prec_21_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[6,1]+v[6,1];  \n";
    FileMod << "  \n";
    // FileMod << "#================ END of cycle 1======================#  \n";
    FileMod << "  \n";
    // FileMod << "# constraints in the same cycle in same P??  \n";
    FileMod << "s.t. Prec_11_11_c23{p in P11, k in K: (p+1)in P11 and k>1  }:  "
               "t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod << "s.t. Prec_12_12_c23{p in P12, k in K: (p+1)in P12 and k>1  }:  "
               "t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod << "s.t. Prec_21_21_c23{p in P21, k in K: (p+1)in P21 and k>1  }:  "
               "t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod << "s.t. Prec_22_22_c23{p in P22, k in K: (p+1)in P22 and k>1  }:  "
               "t[p+1,k]=t[p,k]+v[p,k];  \n";
    FileMod << "  \n";
    // FileMod << "# constraints in the same cycle in connecting   \n";
    FileMod << "s.t. Prec_11_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  "
               "t[p,k]=t[2,k]+v[2,k];\n";
    FileMod << "s.t. Prec_11_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  "
               "t[p,k]=t[2,k]+v[2,k];  \n";
    FileMod << "s.t. Prec_21_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  "
               "t[p,k]=t[6,k]+v[6,k];  \n";
    FileMod << "s.t. Prec_21_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  "
               "t[p,k]=t[6,k]+v[6,k];  \n";
    FileMod << "  \n";
    // FileMod << "# constraints in connecting in different cycles  \n";
    FileMod << "s.t. Prec_12_11_c23{p in P11, k in K: (card(P11)+p+1)=4 and k>1 }:    "
               "t[p,k]=t[4,k-1]+v[4,k-1];  \n";
    FileMod << "s.t. Prec_22_11_c23{p in P11, k in K: (card(P11)+p+1+4)=8 and k>1 }:  "
               "t[p,k]=t[8,k-1]+v[8,k-1];  \n";
    FileMod << "s.t. Prec_12_21_c23{p in P21, k in K: (card(P21)+p+1-4)=4 and k>1 }:  "
               "t[p,k]=t[4,k-1]+v[4,k-1];  \n";
    FileMod << "s.t. Prec_22_21_c23{p in P21, k in K: (card(P21)+p+1)=8 and k>1 }:    "
               "t[p,k]=t[8,k-1]+v[8,k-1];  \n";
    FileMod << "  \n";
    // FileMod << "#==================================================#  \n";
    FileMod << "  \n";
    FileMod << "s.t. RD: PriorityDelay=( sum{p in P,j in J, tt in T} "
               "(priorityTypeWeigth[j,tt]*active_pj[p,j]*d[p,j] ) )  + PrioWeigth[6]*(coord1delay "
               "+ coord2delay)/2 ;  \n";
    FileMod << "  \n";
    if (dCoordinationWeight > 0)
    {
        FileMod << "s.t. c0: coord1delay=isItCoordinated*((coef[coordphase1,1]-afterSplit)*(( "
                   "zetatl - Cl[coordphase1,1]*zeta1 ) + (-zetatu + Cu[coordphase1,1]*zeta2)) "
                   "+(1-coef[coordphase1,1] +afterSplit)*(( zetatl - Cl[coordphase1,1]*zeta1 ) + "
                   "(-zetatu + Cu[coordphase1,1]*zeta2)) ); \n";
        FileMod << "s.t. cc0: coord2delay=isItCoordinated*((coef[coordphase2,1]-afterSplit)*(( "
                   "gammatl - Cl[coordphase2,2]*gamma1 ) + (-gammatu + Cu[coordphase2,2]*gamma2)) "
                   "+(1-coef[coordphase2,1]+afterSplit)*(( gammatl - Cl[coordphase2,2]*gamma1 ) + "
                   "(-gammatu + Cu[coordphase2,2]*gamma2)) ); \n";

        FileMod << "s.t. c1: M*zeta1>=isItCoordinated*(coef[coordphase1,1]-afterSplit)*( "
                   "t[coordphase1,1]-Cl[coordphase1,1] ); \n";
        FileMod << "s.t. c2: isItCoordinated*(coef[coordphase1,1]-afterSplit)* ( "
                   "Cl[coordphase1,1]-t[coordphase1,1]) <= M*(1-zeta1);\n";
        FileMod << "s.t. c3: isItCoordinated*(coef[coordphase1,1]-afterSplit)* ( "
                   "t[coordphase1,1]-M*(1-zeta1) ) <= zetatl; \n";
        FileMod << "s.t. c4: isItCoordinated*(coef[coordphase1,1]-afterSplit)* zetatl <=( "
                   "t[coordphase1,1]+M*(1-zeta1) ) ; \n";
        FileMod
            << "s.t. c5: isItCoordinated*(coef[coordphase1,1]-afterSplit)* zetatl <=M*zeta1; \n";

        FileMod << "s.t. c6: M*zeta2>=isItCoordinated*(coef[coordphase1,1]-afterSplit)*( "
                   "Cu[coordphase1,1]-(g[coordphase1,1]+t[coordphase1,1]) ); \n";
        FileMod << "s.t. c7: isItCoordinated*(coef[coordphase1,1]-afterSplit)*( "
                   "(g[coordphase1,1]+t[coordphase1,1])-Cu[coordphase1,1] )<= M*(1-zeta2); \n";
        FileMod << "s.t. c8: isItCoordinated*(coef[coordphase1,1]-afterSplit)*( "
                   "(g[coordphase1,1]+t[coordphase1,1])-M*(1-zeta2) ) <= zetatu; \n";
        FileMod << "s.t. c9: isItCoordinated*(coef[coordphase1,1]-afterSplit)*zetatu <= ( "
                   "(g[coordphase1,1]+t[coordphase1,1]) + M*(1-zeta2) ) ; \n";
        FileMod << "s.t. c10: isItCoordinated*(coef[coordphase1,1]-afterSplit)*zetatu<=M*zeta2; \n";

        FileMod << "s.t. c11: M*zeta1>=isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "t[coordphase1,2]-Cl[coordphase1,1] ); \n";
        FileMod << "s.t. c12: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "Cl[coordphase1,1]-t[coordphase1,2]) <= M*(1-zeta1); \n";
        FileMod << "s.t. c13: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "t[coordphase1,2]-M*(1-zeta1) )<= zetatl; \n";
        FileMod << "s.t. c14: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatl<= ( "
                   "t[coordphase1,2]+M*(1-zeta1) ) ; \n";
        FileMod
            << "s.t. c15: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatl<=M*zeta1; \n";

        FileMod << "s.t. c16: M*zeta2>=isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "Cu[coordphase1,1]-(g[coordphase1,2]+t[coordphase1,2]) ); \n";
        FileMod << "s.t. c17: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "(g[coordphase1,2]+t[coordphase1,2])-Cu[coordphase1,1] )<= M*(1-zeta2); \n";
        FileMod << "s.t. c18: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( "
                   "(g[coordphase1,2]+t[coordphase1,2])-M*(1-zeta2) ) <= zetatu; \n";
        FileMod << "s.t. c19: isItCoordinated*(1-coef[coordphase1,1+afterSplit]) *zetatu <=  ( "
                   "(g[coordphase1,2]+t[coordphase1,2]) + M*(1-zeta2) ) ; \n";
        FileMod
            << "s.t. c20: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatu<=M*zeta2; \n";

        FileMod << "s.t. cc1: M*gamma1>=isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "t[coordphase2,1]-Cl[coordphase2,2] ); \n";
        FileMod << "s.t. cc2: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "Cl[coordphase2,2]-t[coordphase2,1] ) <= M*(1-gamma1); \n";
        FileMod << "s.t. cc3: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "t[coordphase2,1]-M*(1-gamma1) )<= gammatl; \n";
        FileMod << "s.t. cc4: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatl<=( "
                   "t[coordphase2,1]+M*(1-gamma1) ) ; \n";
        FileMod
            << "s.t. cc5: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatl<=M*gamma1; \n";

        FileMod << "s.t. cc6: M*gamma2>=isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "Cu[coordphase2,2]-(g[coordphase2,1]+t[coordphase2,1]) ); \n";
        FileMod << "s.t. cc7: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "(g[coordphase2,1]+t[coordphase2,1])-Cu[coordphase2,2])<= M*(1-gamma2); \n";
        FileMod << "s.t. cc8: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( "
                   "(g[coordphase2,1]+t[coordphase2,1])-M*(1-gamma2) ) <= gammatu; \n";
        FileMod << "s.t. cc9: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatu <= ( "
                   "(g[coordphase2,1]+t[coordphase2,1]) + M*(1-gamma2) ) ; \n";
        FileMod
            << "s.t. cc10: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatu<=M*gamma2; \n";

        FileMod << "s.t. cc11: M*gamma1>=isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "t[coordphase2,2]-Cl[coordphase2,2] ); \n";
        FileMod << "s.t. cc12: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "Cl[coordphase2,2]-t[coordphase2,2] ) <= M*(1-gamma1); \n";
        FileMod << "s.t. cc13: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "t[coordphase2,2]-M*(1-gamma1) )<= gammatl; \n";
        FileMod << "s.t. cc14: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatl<= ( "
                   "t[coordphase2,2]+M*(1-gamma1) ) ; \n";
        FileMod << "s.t. cc15: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatl <= "
                   "M*gamma1; \n";

        FileMod << "s.t. cc16: M*gamma2>=isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "Cu[coordphase2,2]-(g[coordphase2,2]+t[coordphase2,2]) ); \n";
        FileMod << "s.t. cc17: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "(g[coordphase2,2]+t[coordphase2,2])-Cu[coordphase2,2])<= M*(1-gamma2); \n";
        FileMod << "s.t. cc18: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( "
                   "(g[coordphase2,2]+t[coordphase2,2])-M*(1-gamma2) ) <= gammatu; \n";
        FileMod << "s.t. cc19: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatu <=  ( "
                   "(g[coordphase2,2]+t[coordphase2,2]) + M*(1-gamma2) ) ; \n";
        FileMod << "s.t. cc20: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatu <= "
                   "M*gamma2; \n";
    }
    if (m_codeUsage == ADAPTIVE_PRIORITY)
        FileMod << "s.t. TotRegVehDelay: TotRegVehDel=(sum{i in I} "
                   "active_arr[i]*rd[i])/SumOfActiveArr;  \n";
    FileMod << "  \n";
    FileMod << "s.t. PhaseLen{p in P, k in K}:  v[p,k]=(g[p,k]+y[p]+red[p])*coef[p,k];  \n";
    FileMod << "  \n";
    FileMod << "s.t. PhaseLen2{p in P, k in K}:  v[p,k]*coef[p,k]>=g[p,k]; \n";
    FileMod << "  \n";
    //    FileMod << "s.t. GrnMax{p in P ,k in K:  ( (p!= coordphase1 and p!=coordphase2 and "
    //               "isItCoordinated=1)         or isItCoordinated=0 ) }:  "
    //               "g[p,k]<=(gmax[p]-MinGrn1[p,k]-MinGrn2[p,k])*coef[p,k];    \n";
    //    FileMod << "s.t. GrnMax2:  "
    //               "(1-earlyReturn)*g[2,1]<=(1-earlyReturn)*(gmax[2]-((inSplit+afterSplit)*(current)+("
    //               "1-afterSplit-inSplit)*(MinGrn1[2,1]+MinGrn2[2,1])));   \n";
    //    FileMod << "s.t. GrnMax3:  "
    //               "(1-earlyReturn)*g[6,1]<=(1-earlyReturn)*(gmax[6]-((inSplit+afterSplit)*(current)+("
    //               "1-afterSplit-inSplit)*(MinGrn1[6,1]+MinGrn2[6,1])));   \n";
    //    FileMod << "s.t. GrnMax4{p in P: (p=coordphase1 or p=coordphase2)}:  g[p,2]<=gmax[p]; \n";
    //    FileMod << "s.t. GrnMin{p in P,k in K}:  "
    //               "g[p,k]>=(gmin[p]-MinGrn1[p,k]-MinGrn2[p,k])*coef[p,k];   \n";
    FileMod << "s.t. GrnMax{p in P ,k in K}:  "
               "g[p,k]<=(gmax_revised[p,k]-PassedGrn1[p,k]-PassedGrn2[p,k])*coef[p,k]; \n";
    FileMod << "s.t. GrnMin{p in P ,k in K}:  "
               "g[p,k]>=(gmin[p]-PassedGrn1[p,k]-PassedGrn2[p,k])*coef[p,k]; \n";

    FileMod << "  \n";
    FileMod << "s.t. PrioDelay1{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>=t[p,1]-Rl[p,j];  \n";
    FileMod << "s.t. PrioDelay2{p in P,j in J: active_pj[p,j]>0 }:    "
               "M*theta[p,j]>=Ru[p,j]-(t[p,1]+g[p,1]); \n";
    FileMod << "s.t. PrioDelay3{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>= "
               "ttheta[p,j]-Rl[p,j]*theta[p,j]; \n";
    FileMod << "s.t. PrioDelay4{p in P,j in J: active_pj[p,j]>0 }:    g[p,1]>= "
               "(Ru[p,j]-Rl[p,j])*(1-theta[p,j]); \n";
    FileMod
        << "s.t. PrioDelay5{p in P, j in J: active_pj[p,j]>0}:    ttheta[p,j]<=M*theta[p,j]; \n";
    FileMod << "s.t. PrioDelay6{p in P, j in J: active_pj[p,j]>0}:    "
               "t[p,2]-M*(1-theta[p,j])<=ttheta[p,j]; \n";
    FileMod << "s.t. PrioDelay7{p in P, j in J: active_pj[p,j]>0}:    "
               "t[p,2]+M*(1-theta[p,j])>=ttheta[p,j]; \n";
    FileMod << "s.t. PrioDelay8{p in P, j in J: active_pj[p,j]>0}:   "
               "g[p,2]>=(Ru[p,j]-Rl[p,j])*theta[p,j]; \n ";
    FileMod << "s.t. PrioDelay9 {p in P, j in J: active_pj[p,j]>0}:   Ru[p,j]*theta[p,j] <= ( "
               "t[p,2]+g[p,2]) ; \n ";
    FileMod << "  \n";
    if (m_codeUsage == ADAPTIVE_PRIORITY)
    {
        // FileMod << "s.t. RVehD1{i in I: (active_arr[i]>0)}:                                 rd[i]
        // >= t[Ph[i],1]     +q[i]/s-(dif1[i])/Ve[i]+Tq[i];  \n ";
        // FileMod << "s.t. RVehD2{i in I: (active_arr[i]>0)}:
        // M*miu[i] >= q[i]/s + Ar[i]/Ve[i] -(g[Ph[i],1]);  \n ";
        // FileMod << "s.t. RVehD3{i in I: (active_arr[i]>0 )}:                               rd[i]
        // >= tmiu[i]    + (dif2[i])/s - (dif3[i])/Ve[i] + Tq[i]; \n ";

        FileMod << "s.t. RVehD1{i in I: (active_arr[i]>0)}: rd[i] >= "
                   "t[Ph[i],1]+q[i]/s-(Ar[i]-q[i])/Ve[i]+Tq[i];  \n ";
        FileMod << "s.t. RVehD2{i in I: (active_arr[i]>0)} : M*miu[i] >= q[i]/s + Ar[i]/Ve[i] "
                   "-g[Ph[i],1];  \n ";
        FileMod << "s.t. RVehD3{i in I: (active_arr[i]>0 )} : rd[i] >= tmiu[i] + qmiu[i]/s - "
                   "gmiu[i] - Ar[i]*miu[i]/Ve[i] + qmiu[i]/Ve[i] + Tq[i]; \n ";
        // FileMod << "s.t. RVehD4{p in P,i in I: (active_arr[i]>0 and p=Ph[i])} : M*(1-miu[i]) >=
        // t[p,2]+g[p,2]-((q[i]-s*g[p,1])/s +Ar[i]/Ve[i]); \n ";
        FileMod << "s.t. RVehD5{i in I,ii in I: (active_arr[i]>0 and active_arr[ii]>0  and "
                   "Ln[i]=Ln[ii] and Ar[i]<Ar[ii])} : miu[i] <= miu[ii] ; \n ";

        FileMod << "s.t. RVehD6{i in I: (active_arr[i]>0)}: tmiu[i]<=M*miu[i];   \n ";
        FileMod << "s.t. RVehD7{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: "
                   "t[p,2]-M*(1-miu[i])<=tmiu[i];   \n ";
        FileMod << "s.t. RVehD8{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: "
                   "t[p,2]+M*(1-miu[i])>=tmiu[i]; \n ";

        FileMod << "s.t. RVehD9{i in I: active_arr[i]>0 }: gmiu[i]<=M*miu[i];   \n ";
        FileMod << "s.t. RVehD10{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: "
                   "g[p,1]-M*(1-miu[i])<=gmiu[i];   \n ";
        FileMod << "s.t. RVehD11{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: "
                   "g[p,1]+M*(1-miu[i])>=gmiu[i]; \n ";

        FileMod << "s.t. RVehD12{i in I: (active_arr[i]>0)}: qmiu[i]<=M*miu[i];   \n ";
        FileMod << "s.t. RVehD13{i in I: (active_arr[i]>0)}: q[i]-M*(1-miu[i])<=qmiu[i];   \n ";
        FileMod << "s.t. RVehD14{i in I: (active_arr[i]>0)}: q[i]+M*(1-miu[i])>=qmiu[i]; \n ";

        FileMod << "s.t. RVehD15{i in I,k in K: (active_arr[i]>0)}: "
                   "qq[i]=((LaPhSt[Ln[i]])*(L0[Ln[i]]+Ratio[i]*qSp-g[Ph[i],1]*s)*indic[i])+((1-"
                   "LaPhSt[Ln[i]])*(L0[Ln[i]]*del[i]+Ratio[i]*qSp*del[i]-s*Ratio[i]*del[i]+s*((1-"
                   "coef[Ph[i],1])*delT2[i]+coef[Ph[i],1]*delT1[i])-s*Ratio[i]*delZe[i]-s*((1-coef["
                   "Ph[i],1])*delZeT2[i]+coef[Ph[i],1]*delZeT1[i]))); \n ";

        FileMod << "s.t. RVehD16{i in I: (active_arr[i]>0)}: "
                   "coef[Ph[i],1]*(t[Ph[i],1]-M*(1-del[i]))<=coef[Ph[i],1]*delT1[i];  \n ";
        FileMod << "s.t. RVehD17{i in I: (active_arr[i]>0)}: "
                   "coef[Ph[i],1]*(t[Ph[i],1]+M*(1-del[i]))>=coef[Ph[i],1]*delT1[i]; \n ";
        FileMod << "s.t. RVehD18{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*delT1[i]<=M*del[i]; \n ";

        FileMod << "s.t. RVehD19{i in I: (active_arr[i]>0)}: "
                   "(1-coef[Ph[i],1])*t[Ph[i],2]-M*(1-del[i])<=(1-coef[Ph[i],1])*delT2[i];  \n ";
        FileMod << "s.t. RVehD20{i in I: (active_arr[i]>0)}: "
                   "(1-coef[Ph[i],1])*t[Ph[i],2]+M*(1-del[i])>=(1-coef[Ph[i],1])*delT2[i];  \n ";
        FileMod << "s.t. RVehD21{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*delT2[i]<=M*del[i]; "
                   " \n ";

        FileMod << " s.t. RVehD22{i in I: (active_arr[i]>0)}: "
                   "coef[Ph[i],1]*(t[Ph[i],1]-M*(1-delZe[i]))<=delZeT1[i];  \n ";
        FileMod << "s.t. RVehD23{i in I: (active_arr[i]>0)}: "
                   "t[Ph[i],1]+M*(1-delZe[i])>=coef[Ph[i],1]*delZeT1[i];  \n ";
        FileMod << "s.t. RVehD24{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*delZeT1[i]<=M*delZe[i]; "
                   " \n ";

        FileMod << "s.t. RVehD25{i in I: (active_arr[i]>0)}: "
                   "(1-coef[Ph[i],1])*(t[Ph[i],2]-M*(1-delZe[i]))<=delZeT2[i];  \n ";
        FileMod << "s.t. RVehD26{i in I: (active_arr[i]>0)}: "
                   "t[Ph[i],2]+M*(1-delZe[i])>=(1-coef[Ph[i],1])*delZeT2[i];  \n ";
        FileMod << "s.t. RVehD27{i in I: (active_arr[i]>0)}: "
                   "(1-coef[Ph[i],1])*delZeT2[i]<=M*delZe[i];  \n ";

        FileMod << "s.t. RVehD28{i in I: (active_arr[i]>0)}: delZe[i]<=del[i];  \n ";
        FileMod << "s.t. RVehD29{i in I: (active_arr[i]>0)}: delZe[i]<=ze[i];  \n ";
        FileMod << "s.t. RVehD30{i in I: (active_arr[i]>0)}: delZe[i]>=del[i]+ze[i]-1;  \n ";

        FileMod << "s.t. RVehD31{i in I: (active_arr[i]>0)}: ((1-coef[Ph[i],1])*t[Ph[i],2] + "
                   "coef[Ph[i],1]*t[Ph[i],1] - Ratio[i])<=M*ze[i];  \n ";
        FileMod << "s.t. RVehD32{i in I: (active_arr[i]>0)}: ((1-coef[Ph[i],1])*t[Ph[i],2] + "
                   "coef[Ph[i],1]*t[Ph[i],1] - Ratio[i])>=M*(ze[i]-1);  \n ";

        FileMod << "s.t. RVehD33{i in I: (active_arr[i]>0)}: s*(((1-coef[Ph[i],1])*t[Ph[i],2] + "
                   "coef[Ph[i],1]*t[Ph[i],1]) + L0[Ln[i]])/(s-qSp)- Ratio[i]<=M*del[i];  \n ";
        FileMod << "s.t. RVehD34{i in I: (active_arr[i]>0)}: s*(((1-coef[Ph[i],1])*t[Ph[i],2] + "
                   "coef[Ph[i],1]*t[Ph[i],1]) + L0[Ln[i]])/(s-qSp)- Ratio[i]>=M*(del[i]-1);  \n ";

        FileMod << "s.t. RVehD35{i in I: (active_arr[i]>0)}: qq[i]<=M*qIndic[i];  \n ";
        FileMod << "s.t. RVehD36{i in I: (active_arr[i]>0)}: qq[i]>=M*(qIndic[i]-1);  \n ";

        FileMod << "s.t. RVehD37{i in I: (active_arr[i]>0)}: qq[i]-M*(1-qIndic[i])<=q[i];  \n ";
        FileMod << "s.t. RVehD38{i in I: (active_arr[i]>0)}: qq[i]+M*(1-qIndic[i])>=q[i];  \n ";
        FileMod << "s.t. RVehD39{i in I: (active_arr[i]>0)}: q[i]<=M*qIndic[i];  \n ";

        // FileMod << "s.t. RVehD40{i in I: (active_arr[i]>0)}: Ar[i]-q[i]<=M*dif1Bina[i];  \n ";
        // FileMod << "s.t. RVehD41{i in I: (active_arr[i]>0)}: Ar[i]-q[i]>=M*(dif1Bina[i]-1);  \n
        // ";

        // FileMod << "s.t. RVehD42{i in I: (active_arr[i]>0)}:
        // Ar[i]-q[i]-M*(1-dif1Bina[i])<=dif1[i];  \n ";
        // FileMod << "s.t. RVehD43{i in I: (active_arr[i]>0)}:
        // Ar[i]-q[i]+M*(1-dif1Bina[i])>=dif1[i];  \n ";
        // FileMod << "s.t. RVehD44{i in I: (active_arr[i]>0)}: dif1[i]<=M*dif1Bina[i];  \n ";

        // FileMod << "s.t. RVehD45{i in I: (active_arr[i]>0)}: (qmiu[i] -
        // s*gmiu[i])<=M*dif2Bina[i];  \n ";
        // FileMod << "s.t. RVehD46{i in I: (active_arr[i]>0)}: (qmiu[i] -
        // s*gmiu[i])>=M*(dif2Bina[i]-1);  \n ";

        // FileMod << "s.t. RVehD47{i in I: (active_arr[i]>0)}: (qmiu[i] -
        // s*gmiu[i])-M*(1-dif2Bina[i])<=dif2[i];  \n ";
        // FileMod << "s.t. RVehD48{i in I: (active_arr[i]>0)}: (qmiu[i] -
        // s*gmiu[i])+M*(1-dif2Bina[i])>=dif2[i];  \n ";
        // FileMod << "s.t. RVehD49{i in I: (active_arr[i]>0)}: dif2[i]<=M*dif2Bina[i];  \n ";

        // FileMod << "s.t. RVehD50{i in I: (active_arr[i]>0)}: (Ar[i]*miu[i] -
        // qmiu[i])<=M*dif3Bina[i];  \n ";
        // FileMod << "s.t. RVehD51{i in I: (active_arr[i]>0)}: (Ar[i]*miu[i] -
        // qmiu[i])>=M*(dif3Bina[i]-1);  \n ";

        // FileMod << "s.t. RVehD52{i in I: (active_arr[i]>0)}: (Ar[i]*miu[i] -
        // qmiu[i])-M*(1-dif3Bina[i])<=dif3[i];  \n ";
        // FileMod << "s.t. RVehD53{i in I: (active_arr[i]>0)}: (Ar[i]*miu[i] -
        // qmiu[i])+M*(1-dif3Bina[i])>=dif3[i];  \n ";
        // FileMod << "s.t. RVehD54{i in I: (active_arr[i]>0)}: dif3[i]<=M*dif3Bina[i];   \n ";
    }

    if (m_codeUsage !=
        ADAPTIVE_PRIORITY) // in case we do not consider egular vehicles (no arrival table)
        FileMod << "minimize delay: PriorityDelay  ;\n";
    else
        FileMod << "  minimize delay:  TotRegVehDel+  PriorityDelay;     \n";

    const std::string resultsFile = SieMmitss::getGlpkResultsFilePath();

    FileMod << "  \n";
    FileMod << "solve;  \n";
    FileMod << "  \n";
    FileMod << "printf \" \" > \"" << resultsFile << "\";  \n";
    FileMod << "printf \"%3d  %3d \\n \",SP1, SP2 >>\"" << resultsFile << "\";  \n";
    FileMod << "printf \"%5.2f  %5.2f %5.2f  %5.2f \\n \",init1, init2,Grn1,Grn2 >>\""
            << resultsFile << "\";  \n";
    FileMod << "for {k in K}   \n";
    FileMod << " { \n";
    FileMod << "     for {p in P2} \n";
    FileMod << "        { \n";
    FileMod << "           printf \"%5.2f  \", if(p in P)  then v[p,k] else 0  >>\"" << resultsFile
            << "\";   \n";
    FileMod << "        } \n";
    FileMod << "        printf \" \\n \">>\"" << resultsFile << "\";\n";
    FileMod << " } \n";
    FileMod << "  \n";
    // Added on 2012.4.4: the g[p,k] is added for TimeStamp[2][3].
    FileMod << "for {k in K}   \n";
    FileMod << " { \n";
    FileMod << "     for {p in P2} \n";
    FileMod << "        { \n";
    FileMod << "           printf \"%5.2f  \", if(p in P)  then g[p,k] else 0  >>\"" << resultsFile
            << "\";   \n";
    FileMod << "        } \n";
    FileMod << "        printf \" \\n \">>\"" << resultsFile << "\";\n";
    FileMod << " } \n";
    FileMod << "  \n";
    FileMod << "printf \"%3d \\n \", ReqNo >>\"" << resultsFile << "\";  \n";
    FileMod << "  \n";
    FileMod << "for {p in P,j in J : Rl[p,j]>0}  \n";
    FileMod << " {  \n";
    FileMod << "   printf \"%d  %5.2f  %5.2f  %5.2f %d \\n \", (p+ 10*(theta[p,j])), "
               "Rl[p,j],Ru[p,j], d[p,j] , priorityType[j] >>\""
            << resultsFile << "\";\n";
    FileMod << " } \n";
    if (dCoordinationWeight > 0)
    {
        FileMod << "   printf \"%d  %5.2f  %5.2f  %5.2f %d \\n \", (2+ 10*(1-coef[2,1])), "
                   "Cl[2,1],Cu[2,1], coord1delay , 6 >>\""
                << resultsFile << "\";\n";
        FileMod << "   printf \"%d  %5.2f  %5.2f  %5.2f %d \\n \", (6+ 10*(1-coef[6,1])), "
                   "Cl[6,2],Cu[6,2], coord2delay , 6 >>\""
                << resultsFile << "\";\n";
    }
    FileMod << "printf \"%5.2f \\n \", PriorityDelay >>\"" << resultsFile << "\"; \n";
    if (m_codeUsage == ADAPTIVE_PRIORITY) // If arrival table is considered ( in Integrated Priority
                                          // and Adaptive Control)
        FileMod << "printf \"%5.2f \\n \", TotRegVehDel >>\"" << resultsFile << "\";  \n ";
    FileMod << "printf \" \\n \">>\"" << resultsFile << "\";\n";
    //------------- End of Print the Main body of mode----------------
    FileMod << "end;\n";
    FileMod.close();
    return true;
}

void MmitssPrioritySolver::GLPKSolver()
{
    double ttt2;
    double t1 = GetSeconds();
    // The argument should be the real phase 1-8.
    struct timeval start;
    gettimeofday(&start, NULL);
    std::string modFile =
        m_haveEVInList ? SieMmitss::getModEVFilePath() : SieMmitss::getModFilePath();
    ITSAPP_LOG("Modfile: %s", modFile.c_str());

    glp_prob* mip = glp_create_prob();
    glp_tran* tran = glp_mpl_alloc_wksp();

    // ret = glp_mpl_read_model(tran, "./Mod/PriReq_26.mod", 1);
    if (glp_mpl_read_model(tran, modFile.c_str(), 1) != 0)
    {
        ITSAPP_WRN("Error on translating model!");
        goto skip;
    }
    if (glp_mpl_read_data(tran, SieMmitss::getPriorityDataFilePath().c_str()) != 0)
    {
        ITSAPP_WRN("Error on translating data");
        goto skip;
    }
    if (glp_mpl_generate(tran, NULL) != 0)
    {
        ITSAPP_WRN("Error on generating model");
        goto skip;
    }

    ttt2 = GetSeconds();
    ITSAPP_LOG("TIME TO GENERATE THE OPTIMIZATION MODEL: %lfs", ttt2 - t1);

    glp_mpl_build_prob(tran, mip);
    glp_simplex(mip, NULL);
    glp_iocp iocp;
    glp_init_iocp(&iocp);
    iocp.tm_lim = 4000; // if after 4 seconds the optimizer can not find the best MIP solution, skip
                        // this time of solving

    glp_intopt(mip, &iocp);

    if (glp_mpl_postsolve(tran, mip, GLP_MIP) != 0)
    {
        ITSAPP_WRN("Error on post solving model: [%.2f].", GetSeconds());
    }
skip:
    glp_mpl_free_wksp(tran);
    glp_delete_prob(mip);
}

int MmitssPrioritySolver::GLPKSolutionValidation()
{
    fstream r_f;
    r_f.open(SieMmitss::getGlpkResultsFilePath(), fstream::in);
    int i, j;
    int IsValid = 1;
    int flag = 0;
    if (!r_f)
    {
        ITSAPP_WRN("***********Error opening the result file!");
        exit(1);
    }

    string lineread;
    double V[2][8];
    getline(r_f, lineread);
    getline(r_f, lineread);
    for (i = 0; i < 2; i++)
    {
        getline(r_f, lineread);
        sscanf(lineread.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &V[i][0], &V[i][1], &V[i][2],
            &V[i][3], &V[i][4], &V[i][5], &V[i][6], &V[i][7]);
        ITSAPP_LOG("%lf %lf %lf %lf %lf %lf %lf %lf ", V[i][0], V[i][1], V[i][2], V[i][3], V[i][4],
            V[i][5], V[i][6], V[i][7]);
    }
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 8; j++)
        {

            if (V[i][j] > 0.0)
            {
                IsValid = 0; // have valid solution
                flag = 1;
                break;
            }
        }
        if (flag == 1)
            break;
    }
    return IsValid;
}

void MmitssPrioritySolver::tick()
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
                    sigc::mem_fun(*this, &MmitssPrioritySolver::onTimer));
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

bool MmitssPrioritySolver::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..%d", m_started);
    // Get number of lanes each phase
    if (!get_lane_no())
        return false;
    // Get phase of each lane
    if (!get_lane_phase_mapping())
        return false;

    // cout << "Number of lanes of each phase is:";
    // cout << LaneNo[0] << " " << LaneNo[1] << " " << LaneNo[2] << " " << LaneNo[3] << " " <<
    // LaneNo[4] << " " << LaneNo[5] << " " << LaneNo[6] << " " << LaneNo[7] << endl;

    //-----------------Read in the ConfigIS Every time in case of changing
    // plan-----------------------//
    int curPlan = m_mib.CurTimingPlanRead();
    ITSAPP_LOG("Current timing plan is: %d", curPlan);

    // Generate: configinfo_XX.txt
    m_mib.IntersectionConfigRead(curPlan, SieMmitss::getConfigInfoFilePath());
    PrintPlan2Log(SieMmitss::getConfigInfoFilePath());

    m_cfg.ReadInConfig(
        SieMmitss::getConfigInfoFilePath(), SieMmitss::getPriorityConfigurationFilePath());
    m_cfg.PrintRSUConfig();

    if (!GenerateMod(SieMmitss::getConfigInfoFilePath(), SieMmitss::getModFilePath(), 0))
        return false;

    m_initTime[0] = 0;
    m_initTime[1] = 0;
    m_grnElapse[0] = 0;
    m_grnElapse[1] = 0;

    m_timeRequestTraj = 0.0;

    // warm up to see if we are connected to controller
    for (int i = 3; i > 0; i--)
    {
        m_mib.PhaseTimingStatusRead();
        m_phases.UpdatePhaseMPR(m_mib.getPhaseRead(), m_mib.getPhaseStatus(), m_globalGmax);
        m_phases.Display();
        // Phases.RecordPhase(signal_plan_file);
        std::this_thread::sleep_for(200ms);
    }

    // initialize red start time
    for (int i = 0; i < 8; i++)
    {
        m_previousSignalColor[i] = m_mib.getPhaseRead().phaseColor[i];
        if (m_mib.getPhaseRead().phaseColor[i] == Red)
            m_redStartTime[i] = GetSeconds();
        else
            m_redStartTime[i] = 0;
    }
    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssPrioritySolver::extractOptModInputFromTraj()
{
    int VehCnt = 0;
    m_currentTotalVeh = 0;
    for (int it = 0; it < 130; it++)
    {
        m_modInput[it].vehDistToStpBar = 0;
        m_modInput[it].vehSpd = 0;
        m_modInput[it].vehPhase = 0;
        m_modInput[it].ratio = 0;
        m_modInput[it].indic = 0;
    }
    for (int it = 0; it < m_totalLanes; it++)
        m_laneSignalState[it] = 0;

    for (const auto& veh : m_trackedVeh)
    {
        if (VehCnt >= 130)
            break;
        auto& mod = m_modInput[VehCnt];
        if (veh.req_phase > 0)
        {
            if (veh.time_stop > 0)
                mod.stopTime = GetSeconds() - veh.time_stop;
            else
                mod.stopTime = 0.0;
            mod.vehDistToStpBar = veh.stopBarDistance;
            if (veh.Speed > 1)
                mod.vehSpd = veh.Speed;
            else
                mod.vehSpd = 1;
            mod.vehPhase = veh.req_phase;
            // obtaining the lane
            int temp = (veh.approach - 1) / 2 - 1;
            int lane = 0;
            for (int j = temp; j > 0; j--)
            {

                lane = lane + m_laneNo[m_phaseOfEachApproach[j][0]];
                lane = lane + m_laneNo[m_phaseOfEachApproach[j][1]];
            }
            mod.laneNumber = lane + veh.lane;
            if ((0 < mod.laneNumber) && (mod.laneNumber <= m_totalLanes))
            {

                mod.ratio = max(0.0, (mod.vehDistToStpBar - m_qSizeOfEachLane[mod.laneNumber]) /
                                         (m_queuingSpd + mod.vehSpd));
                mod.indic = mod.ratio -
                            m_qSizeOfEachLane[mod.laneNumber] / (m_qDischargingSpd - m_queuingSpd);
            }
            VehCnt++;
            m_currentTotalVeh++;
        }
    }

    if (m_grnElapse[0] > 0)
    {
        for (int i = 0; i < m_totalLanes; i++)
        {
            if (m_phaseOfEachLane[i] == m_initPhase[0])
                m_laneSignalState[i] = 1;
        }
    }

    if (m_grnElapse[1] > 0)
    {
        for (int i = 0; i < m_totalLanes; i++)
        {
            if (m_phaseOfEachLane[i] == m_initPhase[1])
                m_laneSignalState[i] = 1;
        }
    }
}

void MmitssPrioritySolver::creatCPexactlyAsOptSol(double* EndOfPhaseRing1,
    const std::vector<int>& phaseSeqRing1,
    int t1,
    double* EndOfPhaseRing2,
    const std::vector<int>& phaseSeqRing2,
    int t2)
{
    auto& CP = m_criticalPoints;
    const auto& rsuCfg = m_cfg.getConfig();

    int iTemp = 0;
    int iTotalPlanningPhase = 0;
    iTotalPlanningPhase = min(t1, t2); // at most we plan 5 phase ahead in each ring (this is to
                                       // pass to COP). also, the number of the planned phase in
                                       // first and second ring should be the same.
    iTotalPlanningPhase = min(iTotalPlanningPhase, 5);

    ITSAPP_LOG(
        "iTotalPlanningPhase=%d, cycle length=%d", iTotalPlanningPhase, rsuCfg.iCoordinationCycle);
    // Ring 1
    for (int j = 0; j < iTotalPlanningPhase; j++)
    {
        if ((rsuCfg.iCoordinationCycle > 0) &&
            (EndOfPhaseRing1[j] >
                rsuCfg
                    .iCoordinationCycle)) // In the case of coordination priority, if there is early
                                          // return to green for coordinated phase, we should
                                          // release the max green for coordinated phase. One of the
                                          // possible extrem points may be Big M. In order to avoid
                                          // endofphasering1 gets a large number, we need this
                                          // checking. 150 is the largest assumed cycle lenght
            EndOfPhaseRing1[j] = rsuCfg.iCoordinationCycle;
        if ((rsuCfg.iCoordinationCycle == 0) && (EndOfPhaseRing1[j] > 150))
            EndOfPhaseRing1[j] = 150;

        CP[0][0][j] = EndOfPhaseRing1[j];
        CP[0][1][j] = CP[0][0][j];
        iTemp = phaseSeqRing1[j];
        CP[0][2][j] = iTemp % 10;
    }
    // Ring 2
    for (int j = 0; j < iTotalPlanningPhase; j++)
    {
        if ((rsuCfg.iCoordinationCycle > 0) &&
            (EndOfPhaseRing2[j] >
                rsuCfg
                    .iCoordinationCycle)) // In the case of coordination priority, if there is early
                                          // return to green for coordinated phase, we should
                                          // release the max green for coordinated phase. One of the
                                          // possible extrem points may be Big M. In order to avoid
                                          // endofphasering1 gets a large number, we need this
                                          // checking. 150 is the largest assumed cycle lenght
            EndOfPhaseRing2[j] = rsuCfg.iCoordinationCycle;
        if ((rsuCfg.iCoordinationCycle == 0) && (EndOfPhaseRing2[j] > 150))
            EndOfPhaseRing2[j] = 150;
        CP[1][0][j] = EndOfPhaseRing2[j];
        CP[1][1][j] = CP[1][0][j];
        iTemp = phaseSeqRing2[j];
        CP[1][2][j] = iTemp % 10;
    }
}

VEH_PRIORITY
MmitssPrioritySolver::getType(PriorityRequestList PrioReqListOfRing, int currentPosition)
{
    if (currentPosition < 0 || (unsigned)currentPosition >= PrioReqListOfRing.size())
    {
        ITSAPP_ERROR("Invalid currentPosition.");
        return VEH_PRIORITY::NORMAL;
    }
    auto prioReq = PrioReqListOfRing.begin() + currentPosition;
    return prioReq->iType;
}
void MmitssPrioritySolver::creatFeasibleRegionRing(int ring,
    PriorityRequestList PrioReqListOfRing,
    int currentPosition,
    double* EndOfPhase,
    const std::vector<int>& phaseSeq,
    int t,
    double initialGreen,
    double ElapsedGreen,
    int isThereReqInOtherRing)
{
    // this part just check whether there is another request in same phase
    // as the ERP(Earliest Request Respect to Ring and Phase) !!
    // this is helpful when the requested phase in the current phase and
    // we have several other request in that phase after the ERP

    if (ring > 1)
    {
        ITSAPP_ERROR("Invalid ring number passed.");
        return;
    }

    if (currentPosition < 0 || (unsigned)currentPosition >= PrioReqListOfRing.size())
    {
        ITSAPP_ERROR("Invalid currentPosition.");
        return;
    }

    auto& CP = m_criticalPoints[ring];
    auto& cfg = m_cfg.getConfig();

    auto prioReq = PrioReqListOfRing.begin() + currentPosition;
    auto phaseInRing = prioReq->iPhaseCycle;
    auto rl = prioReq->dRl;
    auto ru = prioReq->dRu;
    auto delay = prioReq->dReqDelay;
    auto type = prioReq->iType;

    auto& phaseSeqR = (ring == 0) ? cfg.Phase_Seq_R1 : cfg.Phase_Seq_R2;
    auto& ringNo = (ring == 0) ? cfg.Ring1No : cfg.Ring2No;

    for (const auto& req : PrioReqListOfRing)
    {
        if (req.iPhaseCycle == phaseInRing)
        {
            rl = req.dRl;
            ru = req.dRu;
        }
    }

    int iRequestedPhaseInSeq = -1;
    int iTotalPlanningPhase = 0;
    int idx0 = 0;
    int idx1 = 0;
    int idx2 = 0;
    int idx3 = 0;
    int idx4 = 0;
    for (int i = 0; i < t; i++)
    {
        if (phaseSeq[i] == phaseInRing)
        {
            iRequestedPhaseInSeq = i;
        }
    }

    // current phase in formate of 0 to 3
    const int phase0 = phaseSeq[0] % 10;
    const int phase1 = phaseSeq[1] % 10;
    const int phase2 = phaseSeq[2] % 10;
    const int phase3 = phaseSeq[3] % 10;
    const int phase4 = phaseSeq[4] % 10;

    ////////DEBATE HERE!!!!! about elaps time!!!!

    // --- Adjusting the elpase green time of coordinated phase for coordination request in order to
    // to consider early return to green and its consequesces.
    if ((type == VEH_PRIORITY::COORDINATION) && (rl == 1))
    { // during the split time of coordination
        ElapsedGreen = max(cfg.dCoordinationSplit[0], cfg.dCoordinationSplit[1]) - ru;
    }
    else if ((type == VEH_PRIORITY::COORDINATION) && (rl > 1) &&
             (ru < (cfg.iCoordinationCycle -
                       max(cfg.dCoordinationSplit[0], cfg.dCoordinationSplit[1]) - 10)) &&
             (phaseSeq[0] == phaseInRing))
    { // during other time of the cycle (not split or yield ) , I assumed 10 is the difference
        // between max green and split of the coordinated phase   .   Also, "
        // phaseSeq[0]==phaseInRing  "
        // means the requested coordination phase is the same as current phase. in fact we are in
        // the
        // early return to green

        ElapsedGreen = 0;
    }

    iTotalPlanningPhase = iRequestedPhaseInSeq + 1;
    ITSAPP_LOG("Number of Planned Phases %d", iTotalPlanningPhase);
    double backwRight[iTotalPlanningPhase];
    double backwLeft[iTotalPlanningPhase];
    double fwdRight[iTotalPlanningPhase];
    double fwdLeft[iTotalPlanningPhase];
    if (iTotalPlanningPhase == 1) // if the request is in  the current phase
    {
        idx0 = FindIndexArray<int>(phaseSeqR, ringNo, phase0);
        // dBackwardRightPoints[0]=max(EndOfPhase[0],ru); //999.0;
        backwLeft[0] = ru;
        // fwdRight[0]=cfg.Gmax[idx0]-ElapsedGreen+initialGreen;
        if (type == VEH_PRIORITY::COORDINATION) // for early return to green
        {
            fwdRight[0] = m_globalGmax[idx0];
            backwRight[0] = max(EndOfPhase[0], ru); // 99.0;
        }
        else
        {
            if (isThereReqInOtherRing > -1)
                // if there is request in other ring, we
                // should follow the solution and forceoff
                backwRight[0] = EndOfPhase[0];

            else
                // should not force off immidiately, bc there maybe
                // regulare vehicle behind the priority vehicle
                backwRight[0] = 99.0;

            // cfg.Gmax[cfgPhaseIdxIn]-ElapsedGreen+initialGreen;
            fwdRight[0] = m_globalGmax[idx0] - ElapsedGreen + initialGreen;
        }
        if (ring == 1)
            idx0 += 4; // TODO: this is how mmitss did it, but could be a bug as well..
        fwdLeft[0] = max(0.0, cfg.Gmin[idx0] + initialGreen - ElapsedGreen);

        CP[2][0] = phase0;
        CP[0][0] = max(backwLeft[0], fwdLeft[0]);
        if (backwRight[0] < 0)
            backwRight[0] = fwdRight[0];
        CP[1][0] = min(backwRight[0], fwdRight[0]);
        if (CP[1][0] < CP[0][0])
        { // Avoid the case than left point is greater than right points ,
            // this happens in case that elapse time of the current phase is
            // more than maximum time, then the right point would be negative.
            // so we should make it at lease thesame as left point
            CP[1][0] = CP[0][0];
        }
    }
    else if (iTotalPlanningPhase == 2) // if the request is in the next phase
    {
        // do forward points
        idx0 = FindIndexArray<int>(phaseSeqR, ringNo, phase0) + (ring * 4);
        fwdRight[0] = m_globalGmax[idx0] - ElapsedGreen + initialGreen;
        fwdLeft[0] = max(0.0, cfg.Gmin[idx0] + initialGreen - ElapsedGreen);
        idx1 = FindIndexArray<int>(phaseSeqR, ringNo, phase1) + (ring * 4);
        fwdRight[1] = fwdRight[0] + m_globalGmax[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        fwdLeft[1] = fwdLeft[0] + cfg.Gmin[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];

        // do backward points
        backwRight[1] = max(EndOfPhase[1], ru); // 999.0;
        if (delay != 0)
            backwLeft[1] = EndOfPhase[1];
        else
            backwLeft[1] = ru;
        if (delay == 0)
            backwRight[0] = rl - (cfg.Red[idx0] + cfg.Yellow[idx0]);
        else
            backwRight[0] = EndOfPhase[0];

        if (type == VEH_PRIORITY::COORDINATION) // if the type of request is coordinatuion (type=6)
                                                // we should let early
                                                // return to green
            backwLeft[0] = -1;
        else
        {
            if (delay != 0)
                backwLeft[0] = EndOfPhase[0];
            else
            {
                if (ring == 1)
                { // TODO: again not sure if it is a bug, that mmitss only does that in this branch
                    idx0 -= 4;
                    idx1 -= 4;
                }
                backwLeft[0] =
                    backwLeft[1] - (m_globalGmax[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0]);
            }
        }

        // Do phases
        CP[2][0] = phase0;
        CP[2][1] = phase1;

        // Do CPs
        CP[0][0] = max(backwLeft[0], fwdLeft[0]); // dBackwardLeftPoints[0];
        if (backwRight[0] < 0)
            backwRight[0] = fwdRight[0];
        CP[1][0] = min(backwRight[0], fwdRight[0]);

        if (CP[1][0] < CP[0][0])
        { // in case that elapse time of the current phase is more than maximum time,
            // then the right point would be negative. so we should make it at lease
            // the same as left point
            double iTemp = CP[0][0] - fwdRight[0];
            CP[1][0] = CP[0][0];
            fwdRight[0] = CP[0][0];
            fwdRight[1] = fwdRight[1] + iTemp;
        }

        CP[0][1] = max(backwLeft[1], fwdLeft[1]);
        if (backwRight[1] < 0)
            backwRight[1] = fwdRight[1];
        CP[1][1] = min(backwRight[1], fwdRight[1]);
    }
    else if (iTotalPlanningPhase == 3) // if the request is in the next next phase
    {
        // do forward points

        idx0 = FindIndexArray<int>(phaseSeqR, ringNo, phase0) + (ring * 4);
        fwdRight[0] = m_globalGmax[idx0] - ElapsedGreen + initialGreen;
        fwdLeft[0] = max(0.0, cfg.Gmin[idx0] + initialGreen - ElapsedGreen);
        idx1 = FindIndexArray<int>(phaseSeqR, ringNo, phase1) + (ring * 4);
        // gmax_{P-1} +Cl_{P-2}
        fwdRight[1] = fwdRight[0] + m_globalGmax[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        fwdLeft[1] = fwdLeft[0] + cfg.Gmin[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        idx2 = FindIndexArray<int>(phaseSeqR, ringNo, phase2) + (ring * 4);
        // gmax_{P} +Cl_{P-1}
        fwdRight[2] = fwdRight[1] + m_globalGmax[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        fwdLeft[2] = fwdLeft[1] + cfg.Gmin[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        // do backward points
        backwRight[2] = max(EndOfPhase[2], ru); // 999.0;

        if (delay != 0)
            backwLeft[2] = EndOfPhase[2];
        else
            backwLeft[2] = ru;

        // dBackwardLeftPoints[2]=EndOfPhase[2];
        if (delay == 0)
            backwRight[1] = rl - (cfg.Red[idx1] + cfg.Yellow[idx1]); // rl-cl_{p-1}
        else
            backwRight[1] = EndOfPhase[1];
        //-cl_{p-2}-gmin_{p-1}
        backwRight[0] = backwRight[1] - (cfg.Red[idx0] + cfg.Yellow[idx0]) - cfg.Gmin[idx1];
        backwLeft[0] = backwRight[1] - m_globalGmax[idx1] - (cfg.Red[idx0] + cfg.Yellow[idx0]);
        // Do phases
        CP[2][0] = phase0;
        CP[2][1] = phase1;
        CP[2][2] = phase2;
        // Do CPs
        // Do CPs
        CP[0][0] = max(backwLeft[0], fwdLeft[0]);
        if (type == VEH_PRIORITY::COORDINATION) // if the type of request is coordinatuion (type=6)
                                                // we should let early
                                                // return to green
            backwLeft[1] = CP[0][0] + cfg.Red[idx2] + cfg.Yellow[idx2] + cfg.Gmin[idx1];
        else
        {
            if (delay != 0)
                backwLeft[1] = EndOfPhase[1];
            else
                backwLeft[1] =
                    backwLeft[2] - m_globalGmax[idx2] - (cfg.Red[idx1] + cfg.Yellow[idx1]);
        }

        if (backwRight[0] < 0)
            backwRight[0] = fwdRight[0];
        CP[1][0] = min(backwRight[0], fwdRight[0]);
        if (CP[1][0] < CP[0][0])
        { // in case that elapse time of the current phase is more than maximum time,
            // then the right point would be negative. so we should make it at lease
            // the same as left point
            double iTemp = CP[0][0] - fwdRight[0];
            CP[1][0] = CP[0][0];
            fwdRight[0] = CP[0][0];
            fwdRight[1] = fwdRight[1] + iTemp;
            fwdRight[2] = fwdRight[2] + iTemp;
        }
        CP[0][1] = max(backwLeft[1], fwdLeft[1]);
        if (backwRight[1] < 0)
            backwRight[1] = fwdRight[1];
        CP[1][1] = min(backwRight[1], fwdRight[1]);
        CP[0][2] = max(backwLeft[2], fwdLeft[2]);
        if (backwRight[2] < 0)
            backwRight[2] = fwdRight[2];
        CP[1][2] = min(backwRight[2], fwdRight[2]);
    }
    else if (iTotalPlanningPhase == 4) // if the request is in the next next next phase
    {
        // do forward points

        idx0 = FindIndexArray<int>(phaseSeqR, ringNo, phase0) + (ring * 4);
        fwdRight[0] = m_globalGmax[idx0] - ElapsedGreen + initialGreen;
        fwdLeft[0] = max(0.0, cfg.Gmin[idx0] + initialGreen - ElapsedGreen);
        idx1 = FindIndexArray<int>(phaseSeqR, ringNo, phase1) + (ring * 4);
        // gmax_{P-2} +Cl_{P-3}
        fwdRight[1] = fwdRight[0] + m_globalGmax[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        fwdLeft[1] = fwdLeft[0] + cfg.Gmin[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        idx2 = FindIndexArray<int>(phaseSeqR, ringNo, phase2) + (ring * 4);
        // gmax_{P-1} +Cl_{P-2}
        fwdRight[2] = fwdRight[1] + m_globalGmax[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        fwdLeft[2] = fwdLeft[1] + cfg.Gmin[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        idx3 = FindIndexArray<int>(phaseSeqR, ringNo, phase3) + (ring * 4);
        // gmax_{P} +Cl_{P-1}
        fwdRight[3] = fwdRight[2] + m_globalGmax[idx3] + cfg.Red[idx2] + cfg.Yellow[idx2];
        fwdLeft[3] = fwdLeft[2] + cfg.Gmin[idx3] + cfg.Red[idx2] + cfg.Yellow[idx2];

        // do backward points
        backwRight[3] = max(EndOfPhase[3], ru); // 999.0;
        backwLeft[3] = EndOfPhase[3];
        if (delay == 0)
            // rl-cl_{p-1}
            backwRight[2] = rl - (cfg.Red[idx2] + cfg.Yellow[idx2]);
        else
            backwRight[2] = EndOfPhase[2];
        //~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early
        // return to green
        //~ dBackwardLeftPoints[2]=-1;
        //~ else
        //~
        // dBackwardLeftPoints[2]=dBackwardLeftPoints[3]-cfg.Gmax[idx3]-(cfg.Red[idx2]+cfg.Yellow[idx2]);
        //-cl_{p-2}-gmin_{p-1}
        backwRight[1] = backwRight[2] - (cfg.Red[idx1] + cfg.Yellow[idx1]) - cfg.Gmin[idx2];
        backwLeft[1] = backwRight[2] - m_globalGmax[idx2] - (cfg.Red[idx1] + cfg.Yellow[idx1]);
        //-cl_{p-2}-gmin_{p-1}
        backwRight[0] = backwRight[1] - (cfg.Red[idx0] + cfg.Yellow[idx0]) - cfg.Gmin[idx1];
        backwLeft[0] = backwRight[1] - m_globalGmax[idx1] - (cfg.Red[idx0] + cfg.Yellow[idx0]);
        // Do phases
        CP[2][0] = phase0;
        CP[2][1] = phase1;
        CP[2][2] = phase2;
        CP[2][3] = phase3;
        // Do CPs
        CP[0][0] = max(backwLeft[0], fwdLeft[0]);
        if (backwRight[0] < 0)
            backwRight[0] = fwdRight[0];
        CP[1][0] = min(backwRight[0], fwdRight[0]);
        if (CP[1][0] < CP[0][0])
        { // in case that elapse time of the current phase is more
            // than maximum time, then the right point would be negative.
            // so we should make it at lease thesame as left point
            double iTemp = CP[0][0] - fwdRight[0];
            CP[1][0] = CP[0][0];
            fwdRight[0] = CP[0][0];
            fwdRight[1] = fwdRight[1] + iTemp;
            fwdRight[2] = fwdRight[2] + iTemp;
            fwdRight[3] = fwdRight[3] + iTemp;
        }

        CP[0][1] = max(backwLeft[1], fwdLeft[1]);
        if (type == VEH_PRIORITY::COORDINATION) // if the type of request is coordinatuion (type=6)
                                                // we should let early
                                                // return to green
            backwLeft[2] = CP[0][1] + cfg.Red[idx2] + cfg.Yellow[idx2] + cfg.Gmin[idx1];
        else
            backwLeft[2] = backwLeft[3] - m_globalGmax[idx3] - (cfg.Red[idx2] + cfg.Yellow[idx2]);

        if (backwRight[1] < 0)
            backwRight[1] = fwdRight[1];
        CP[1][1] = min(backwRight[1], fwdRight[1]);
        CP[0][2] = max(backwLeft[2], fwdLeft[2]);
        if (backwRight[2] < 0)
            backwRight[2] = fwdRight[2];
        CP[1][2] = min(backwRight[2], fwdRight[2]);
        CP[0][3] = max(backwLeft[3], fwdLeft[3]);
        if (backwRight[3] < 0)
            backwRight[3] = fwdRight[3];
        CP[1][3] = min(backwRight[3], fwdRight[3]);
    }
    else if (iTotalPlanningPhase == 5) // if the request is in the next 4 phase
    {
        // do forward points

        idx0 = FindIndexArray<int>(phaseSeqR, ringNo, phase0) + (ring * 4);
        fwdRight[0] = m_globalGmax[idx0] - ElapsedGreen + initialGreen;
        fwdLeft[0] = max(0.0, cfg.Gmin[idx0] + initialGreen - ElapsedGreen);
        idx1 = FindIndexArray<int>(phaseSeqR, ringNo, phase1) + (ring * 4);
        // gmax_{P-2} +Cl_{P-3}
        fwdRight[1] = fwdRight[0] + m_globalGmax[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        fwdLeft[1] = fwdLeft[0] + cfg.Gmin[idx1] + cfg.Red[idx0] + cfg.Yellow[idx0];
        idx2 = FindIndexArray<int>(phaseSeqR, ringNo, phase2) + (ring * 4);
        // gmax_{P-1} +Cl_{P-2}
        fwdRight[2] = fwdRight[1] + m_globalGmax[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        fwdLeft[2] = fwdLeft[1] + cfg.Gmin[idx2] + cfg.Red[idx1] + cfg.Yellow[idx1];
        idx3 = FindIndexArray<int>(phaseSeqR, ringNo, phase3) + (ring * 4);
        // gmax_{P} +Cl_{P-1}
        fwdRight[3] = fwdRight[2] + m_globalGmax[idx3] + cfg.Red[idx2] + cfg.Yellow[idx2];
        fwdLeft[3] = fwdLeft[2] + cfg.Gmin[idx3] + cfg.Red[idx2] + cfg.Yellow[idx2];
        idx4 = FindIndexArray<int>(phaseSeqR, ringNo, phase4) + (ring * 4);
        // gmax_{P} +Cl_{P-1}
        fwdRight[4] = fwdRight[3] + m_globalGmax[idx4] + cfg.Red[idx3] + cfg.Yellow[idx3];
        fwdLeft[4] = fwdLeft[3] + cfg.Gmin[idx4] + cfg.Red[idx3] + cfg.Yellow[idx3];

        // do backward points
        backwRight[4] = max(EndOfPhase[4], ru); // 999.0;
        backwLeft[4] = EndOfPhase[4];
        if (delay == 0)
            backwRight[3] = rl - (cfg.Red[idx3] + cfg.Yellow[idx3]); // rl-cl_{p-1}
        else
            backwRight[3] = EndOfPhase[3];
        //~ if (type==6) // if the type of request is coordinatuion (type=6) we should let early
        // return to green
        //~ dBackwardLeftPoints[3]=-1;
        //~ else
        //~
        // dBackwardLeftPoints[3]=dBackwardLeftPoints[4]-cfg.Gmax[idx4]-(cfg.Red[idx3]+cfg.Yellow[idx3]);
        //-cl_{p-2}-gmin_{p-1}
        backwRight[2] = backwRight[3] - (cfg.Red[idx2] + cfg.Yellow[idx2]) - cfg.Gmin[idx3];
        backwLeft[2] = backwRight[3] - m_globalGmax[idx3] - (cfg.Red[idx2] + cfg.Yellow[idx2]);
        //-cl_{p-2}-gmin_{p-1}
        backwRight[1] = backwRight[2] - (cfg.Red[idx1] + cfg.Yellow[idx1]) - cfg.Gmin[idx2];
        backwLeft[1] = backwRight[2] - m_globalGmax[idx2] - (cfg.Red[idx1] + cfg.Yellow[idx1]);
        //-cl_{p-2}-gmin_{p-1}
        backwRight[0] = backwRight[1] - (cfg.Red[idx0] + cfg.Yellow[idx0]) - cfg.Gmin[idx1];
        backwLeft[0] = backwRight[1] - m_globalGmax[idx1] - (cfg.Red[idx0] + cfg.Yellow[idx0]);

        // Do phases
        CP[2][0] = phase0;
        CP[2][1] = phase1;
        CP[2][2] = phase2;
        CP[2][3] = phase3;
        CP[2][4] = phase4;
        // Do CPs
        CP[0][0] = max(backwLeft[0], fwdLeft[0]);
        //~ cout<<"EndOfPhase[4]"<<EndOfPhase[0]<<endl;
        //~ cout<<"dBackwardLeftPoints[0]"<<dBackwardLeftPoints[0]<<endl;
        //~ cout<<"dBackwardRightPoints[1]   "<< dBackwardRightPoints[1]<<endl;
        //~ cout<<"cfg.Gmax[idx1]
        //"<<cfg.Gmax[idx1]<<endl;
        //~ cout<<"fwdLeft[0] "<<fwdLeft[0]<<endl;
        //~ cout<<"fwdRight[0] "<<fwdRight[0]<<endl;
        //~ cout<<"fwdRight[1] "<<fwdRight[1]<<endl;
        //~ cout<<"dBackwardLeftPoints[4]"<<dBackwardLeftPoints[4]<<endl;
        //~ cout<<"fwdLeft[4] "<<fwdLeft[4]<<endl;

        if (backwRight[0] < 0)
            backwRight[0] = fwdRight[0];
        CP[1][0] = min(backwRight[0], fwdRight[0]);
        if (CP[1][0] < CP[0][0])
        { // in case that elapse time of the current phase is more than maximum time,
            // then the right point would be negative. so we should make it at lease thesame as left
            // point
            double iTemp = CP[0][0] - fwdRight[0];
            CP[1][0] = CP[0][0];
            fwdRight[0] = CP[0][0];
            fwdRight[1] = fwdRight[1] + iTemp;
            fwdRight[2] = fwdRight[2] + iTemp;
            fwdRight[3] = fwdRight[3] + iTemp;
            fwdRight[4] = fwdRight[4] + iTemp;
        }
        CP[0][1] = max(backwLeft[1], fwdLeft[1]);
        if (backwRight[1] < 0)
            backwRight[1] = fwdRight[1];
        CP[1][1] = min(backwRight[1], fwdRight[1]);
        CP[0][2] = max(backwLeft[2], fwdLeft[2]);
        if (type == VEH_PRIORITY::COORDINATION) // if the type of request is coordinatuion (type=6)
                                                // we should let early
                                                // return to green
            backwLeft[3] = CP[0][2] + cfg.Red[idx2] + cfg.Yellow[idx2] + cfg.Gmin[idx1];
        else
            backwLeft[3] = backwLeft[4] - m_globalGmax[idx4] - (cfg.Red[idx3] + cfg.Yellow[idx3]);

        if (backwRight[2] < 0)
            backwRight[2] = fwdRight[2];
        CP[1][2] = min(backwRight[2], fwdRight[2]);
        CP[0][3] = max(backwLeft[3], fwdLeft[3]);
        if (backwRight[3] < 0)
            backwRight[3] = fwdRight[3];
        CP[1][3] = min(backwRight[3], fwdRight[3]);
        CP[0][4] = max(backwLeft[4], fwdLeft[4]);
        if (backwRight[4] < 0)
            backwRight[4] = fwdRight[4];
        CP[1][4] = min(backwRight[4], fwdRight[4]);
    }
    ITSAPP_LOG("Current phase: %d", phase0);
    for (int i = 0; i < iTotalPlanningPhase; i++)
    {
        ITSAPP_LOG("fwdLeft[%d]=%lf", i, fwdLeft[i]);
        ITSAPP_LOG("bwdLeft[%d]=%lf", i, backwLeft[i]);
        ITSAPP_LOG("CP left[%d]=%lf", i, CP[0][i]);
        ITSAPP_LOG("fwdRight[%d]=%lf", i, fwdRight[i]);
        ITSAPP_LOG("bwdRight[%d]=%lf", i, backwRight[i]);
        ITSAPP_LOG("CP right[%d]=%lf", i, CP[1][i]);
        ITSAPP_LOG("phase=%lf", CP[2][i]);
    }
}

void MmitssPrioritySolver::creatFeasibleRegionRing_EV(double CP[3][5],
    PriorityRequestList PrioReqListOfRing,
    int currentPosition,
    double* EndOfPhase,
    int* phaseSeq,
    int t)
{
    //    NOTUSED(delay);

    if (currentPosition < 0 || (unsigned)currentPosition >= PrioReqListOfRing.size())
    {
        ITSAPP_ERROR("Invalid currentPosition.");
        return;
    }

    auto prioReq = PrioReqListOfRing.begin() + currentPosition;

    auto phaseInRing = prioReq->iPhaseCycle;
    auto ru = prioReq->dRu;
    //    auto delay = prioReq->dReqDelay;

    int iTemp = 0;
    int iPhase = 0;
    iTemp = phaseSeq[0];
    iPhase = iTemp % 10;
    int iTotalPlanningPhase = 0;
    int iRequestedPhaseInSeq = -1;
    if (t == 1) // in case we have only one phase in this ring, the request can not be passed to the
                // next cycle ( multiple optimal solution in solver may choose the request to be
                // passed to next cycle
        phaseInRing = phaseInRing % 10;

    for (int i = 0; i < t; i++)
    {
        if (phaseSeq[i] == phaseInRing)
        {
            iRequestedPhaseInSeq = i;
        }
    }

    iTotalPlanningPhase = iRequestedPhaseInSeq + 1;
    MMITSS_LOG("iTotalPlanningPhase=%d", iTotalPlanningPhase)
    if (iTotalPlanningPhase == 1) // if we have just one phase in this ring!
    {
        CP[0][0] = ru;
        CP[1][0] = CP[0][0];
        iTemp = phaseSeq[0];
        iPhase = iTemp % 10;
        CP[2][0] = iPhase;
    }
    else
    {
        for (int j = 0; j < iTotalPlanningPhase; j++)
        {
            CP[0][j] = EndOfPhase[j];
            CP[1][j] = CP[0][j];
            iTemp = phaseSeq[j];
            iPhase = iTemp % 10;
            CP[2][j] = iPhase;
        }
    }
}

void MmitssPrioritySolver::matchTheBarrierInCPs()
{
    auto& cp = m_criticalPoints;
    int temp1 = numberOfPlannedPhase(cp[0], 1);
    int temp2 = numberOfPlannedPhase(cp[1], 2);

    if (temp1 == 0)
    {
        for (int it = 0; it < 3; it++)
            for (int it2 = 0; it2 < 5; it2++)
                cp[0][it][it2] = cp[1][it][it2];
    }
    else if (temp2 == 0)
    {
        for (int it = 0; it < 3; it++)
            for (int it2 = 0; it2 < 5; it2++)
                cp[1][it][it2] = cp[0][it][it2];
    }
}

void MmitssPrioritySolver::matchTheBarrierInCPs_EV()
{
    auto& cp = m_criticalPoints;
    int temp1 = numberOfPlannedPhase(cp[0], 1);
    int temp2 = numberOfPlannedPhase(cp[1], 2);
    double dtemp1 = 0.0;
    double dtemp2 = 0.0;
    double dtemp3 = 0.0;
    double dtemp4 = 0.0;
    if (temp1 == 0)
    {
        for (int it = 0; it < 3; it++)
            for (int it2 = 0; it2 < 5; it2++)
                cp[0][it][it2] = cp[1][it][it2];
    }
    else if (temp2 == 0)
    {
        for (int it = 0; it < 3; it++)
            for (int it2 = 0; it2 < 5; it2++)
                cp[1][it][it2] = cp[0][it][it2];
    }
    else if (temp1 > 0 && temp2 > 0 && temp2 < 5 && temp1 < 5) // match the barrier
    {
        for (int it = 0; it < 5; it++)
        {
            for (int it2 = 0; it2 < 5; it2++)
                if ((((int)cp[0][2][it]) % 2 == 1) && (cp[0][2][it] == cp[1][2][it2]))
                {
                    dtemp1 = cp[0][0][it];
                    dtemp2 = cp[1][0][it2];
                    dtemp1 = max(dtemp1, dtemp2);
                    dtemp3 = cp[0][1][it];
                    dtemp4 = cp[1][1][it2];
                    dtemp3 = min(dtemp3, dtemp4); // temp 3 should be larger than temp 1
                    if (dtemp3 < dtemp1)
                        dtemp3 = dtemp1;
                    cp[0][0][it] = dtemp1;
                    cp[1][0][it2] = dtemp1;
                    cp[0][1][it] = dtemp3;
                    cp[1][1][it2] = dtemp3;
                }
        }
    }
}

void MmitssPrioritySolver::Construct_eventlist_EV()
{
    auto& cp = m_criticalPoints;

    int tempOmitPhases[8];
    int iNoOfOmit = 0;
    Schedule Temp_event;
    double iLargestCPinRing1 = 0.0;
    double iLargestCPinRing2 = 0.0;
    double dOmitThereshold = 0.0;
    int iNoPlannedPhaseInRing1 = 0;
    int iNoPlannedPhaseInRing2 = 0;
    iNoPlannedPhaseInRing1 = numberOfPlannedPhase(cp[0], 1);
    iNoPlannedPhaseInRing2 = numberOfPlannedPhase(cp[1], 2);
    iLargestCPinRing1 = findLargestCP(cp[0], 1, iNoPlannedPhaseInRing1);
    iLargestCPinRing2 = findLargestCP(cp[1], 2, iNoPlannedPhaseInRing2);
    ITSAPP_LOG("No of Planned Phase In Ring1 %d", iNoPlannedPhaseInRing1);
    ITSAPP_LOG("No of Planned Phase In Ring2 %d", iNoPlannedPhaseInRing2);
    ITSAPP_LOG("Longest CP in Ring1 %lf", iLargestCPinRing1);
    ITSAPP_LOG("Longest CP in Ring2 %lf", iLargestCPinRing2);
    if (iLargestCPinRing1 > 0 && iLargestCPinRing2 > 0)
        dOmitThereshold = min(iLargestCPinRing1, iLargestCPinRing2);
    else if (iLargestCPinRing1 == 0)
        dOmitThereshold = iLargestCPinRing2;
    else if (iLargestCPinRing2 == 0)
        dOmitThereshold = iLargestCPinRing1;
    for (int i = 0; i < 8; i++)
    {
        if (m_omitPhase[i] > 0)
        {
            tempOmitPhases[iNoOfOmit] = m_omitPhase[i];
            iNoOfOmit++;
        }
    }
    // cout << "No Of Omit" << iNoOfOmit << endl;
    for (int i = 0; i < iNoOfOmit; i++)
    {
        Temp_event.time = dOmitThereshold;
        Temp_event.action = Omit;
        Temp_event.phase = tempOmitPhases[i]; // converting phase number from 0-3 to  1-4
        if (tempOmitPhases[i] < 5)
            m_eventListR1.emplace_back(Temp_event);
        else
            m_eventListR2.emplace_back(Temp_event);
    }
    for (int i = 0; i < iNoPlannedPhaseInRing1; i++)
    {
        // hold
        Temp_event.time = cp[0][0][i];
        Temp_event.action = Hold;
        Temp_event.phase = ((int)(cp[0][2][i])) + 1; // converting phase number from 0-3 to  1-4
        m_eventListR1.emplace_back(Temp_event);
        // call , the call is neccessary before force off. The controller should know where to go (
        // which phase will come up after force off )
        if (i < iNoPlannedPhaseInRing1 - 1)
        {

            Temp_event.time = cp[0][1][i];
            if ((((int)(cp[0][2][i + 1])) + 1) == 5)
                Temp_event.phase = 1;
            else
                Temp_event.phase =
                    ((int)(cp[0][2][i + 1])) + 1; // converting phase number from 0-3 to  1-4
            Temp_event.action = VehCall;
            m_eventListR1.emplace_back(Temp_event);
        }
        // force off
        Temp_event.time = cp[0][1][i];
        Temp_event.action = ForceOff;
        Temp_event.phase = ((int)(cp[0][2][i])) + 1; // converting phase number from 0-3 to  1-4
        m_eventListR1.emplace_back(Temp_event);
    }
    for (int i = 0; i < iNoPlannedPhaseInRing2; i++)
    {
        // hold
        Temp_event.time = cp[1][0][i];
        Temp_event.action = Hold;
        Temp_event.phase = ((int)(cp[1][2][i])) + 5; // converting phase number from 0-3 to  5-8
        m_eventListR2.emplace_back(Temp_event);
        // call
        if (i < iNoPlannedPhaseInRing2 - 1)
        {
            Temp_event.time = cp[1][1][i];
            Temp_event.action = VehCall;
            if (((int)(cp[1][2][i + 1])) + 5 == 9)
                Temp_event.phase = 5;
            else
                Temp_event.phase =
                    ((int)(cp[1][2][i + 1])) + 5; // converting phase number from 0-3 to  5-8
            m_eventListR2.emplace_back(Temp_event);
        }
        // force off
        Temp_event.time = cp[1][1][i];
        Temp_event.action = ForceOff;
        Temp_event.phase = ((int)(cp[1][2][i])) + 5; // converting phase number from 0-3 to  5-8
        m_eventListR2.emplace_back(Temp_event);
    }
    for (const auto& event : m_eventListR1)
    {
        ITSAPP_LOG("Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
    }
    for (const auto& event : m_eventListR2)
    {
        ITSAPP_LOG("Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
    }
    for (int i = 0; i < 8; i++)
        ITSAPP_LOG("OmitPhase[%d]=%d", i, m_omitPhase[i]);
}

void MmitssPrioritySolver::Construct_eventlist()
{
    auto& cp = m_criticalPoints;
    int iNoPlannedPhaseInRing1 = 0;
    int iNoPlannedPhaseInRing2 = 0;
    iNoPlannedPhaseInRing1 = numberOfPlannedPhase(cp[0], 1);
    iNoPlannedPhaseInRing2 = numberOfPlannedPhase(cp[1], 2);
    ITSAPP_LOG("No of Planned Phase In Ring1 %d", iNoPlannedPhaseInRing1);
    ITSAPP_LOG("No of Planned Phase In Ring2 %d", iNoPlannedPhaseInRing2);
    Schedule Temp_event;

    const auto& rsuCfg = m_cfg.getConfig();

    for (int i = 0; i < iNoPlannedPhaseInRing1; i++)
    {
        // call
        Temp_event.time = cp[0][0][i];
        Temp_event.action = Hold;
        Temp_event.phase = (int)(cp[0][2][i] + 1); // converting phase number from 0-3 to  1-4
        m_eventListR1.emplace_back(Temp_event);
        // call , the call is neccessary before force off. The controller should know where to go (
        // which phase will come up after force off )
        if (i < iNoPlannedPhaseInRing1 - 1)
        {
            if ((((int)(cp[0][2][i + 1])) + 1) % 2 ==
                0) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  THIS IS BC IN THE VERY LOW DEMAND,
                   // WE DONT SERVE LEFT TURN!! IF THERE IS NO CAR ON THE DETECTORS
            {
                Temp_event.time = cp[0][1][i] - 0.2; // 0.2 seconds before force off we put a call
                Temp_event.action = VehCall;
                Temp_event.phase =
                    ((int)(cp[0][2][i + 1])) + 1; // converting phase number from 0-3 to  1-4
                m_eventListR1.emplace_back(Temp_event);
            }
        }

        if ((((int)(cp[0][2][i])) + 1) == 2) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  If it is
                                             // neccesary to force off 2, we should call 4 before
                                             // that. Also if it is necessary to forec off 4, we
                                             // should call 2 before that.
        {
            Temp_event.time = cp[0][1][i] - 0.2; // 0.2 seconds before force off we put a call
            Temp_event.action = VehCall;
            Temp_event.phase = 4;
            m_eventListR1.emplace_back(Temp_event);
        }
        if ((((int)(cp[0][2][i])) + 1) == 4) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  If it is
                                             // neccesary to force off 2, we should call 4 before
                                             // that. Also if it is necessary to forec off 4, we
                                             // should call 2 before that.
        {
            Temp_event.time = cp[0][1][i] - 0.2; // 0.2 seconds before force off we put a call
            Temp_event.action = VehCall;
            Temp_event.phase = 2;
            m_eventListR1.emplace_back(Temp_event);
        }

        //// call, this call is neccesary when we run on Coordination. In coordination mode,
        /// coordinated phase should alway have a call. so we put call for coordinated phased at
        /// both
        /// left and right CP
        if (rsuCfg.dCoordinationWeight > 0)
        {
            double ii = 0.0;
            if (cp[0][2][i] + 1 == rsuCfg.iCoordinatedPhase[0])
            {
                // Call in every 1.8 seconds (should be less than backup time, which for now it is
                // assume to be 2sec) for all of the times in the feasible set
                if (i > 0) // in case the coordinated phase is not the first phase ahead
                {
                    while (ii < cp[0][1][i] - cp[0][0][i - 1])
                    {
                        Temp_event.time = (cp[0][0][i - 1] + ii);
                        Temp_event.action = VehCall;
                        Temp_event.phase = rsuCfg.iCoordinatedPhase[0];
                        m_eventListR1.emplace_back(Temp_event);
                        ii = ii + 1.9;
                    }
                    Temp_event.time = (cp[0][0][i - 1]) - 0.2;
                    Temp_event.action = VehCall;
                    Temp_event.phase = rsuCfg.iCoordinatedPhase[0];
                    m_eventListR1.emplace_back(Temp_event);
                }
                if (i == 0)
                {
                    while (ii < cp[0][0][i])
                    {
                        Temp_event.time = ii;
                        Temp_event.action = VehCall;
                        Temp_event.phase = rsuCfg.iCoordinatedPhase[0];
                        m_eventListR1.emplace_back(Temp_event);
                        ii = ii + 1.9;
                    }
                    Temp_event.time = (cp[0][0][i]) - 0.2;
                    Temp_event.action = VehCall;
                    Temp_event.phase = rsuCfg.iCoordinatedPhase[0];
                    m_eventListR1.emplace_back(Temp_event);
                }
            }
        }
        // force off
        Temp_event.time = cp[0][1][i];
        Temp_event.action = ForceOff;
        Temp_event.phase = (int)(cp[0][2][i] + 1); // converting phase number from 0-3 to  1-4
        m_eventListR1.emplace_back(Temp_event);
    }

    for (int i = 0; i < iNoPlannedPhaseInRing2; i++)
    {
        // hold
        Temp_event.time = cp[1][0][i];
        Temp_event.action = Hold;
        Temp_event.phase = (int)(cp[1][2][i] + 5); // converting phase number from 0-3 to  5-8
        m_eventListR2.emplace_back(Temp_event);
        // call
        if (i < iNoPlannedPhaseInRing2 - 1)
        {
            if ((((int)(cp[1][2][i + 1])) + 5) % 2 ==
                0) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  THIS IS BC IN THE VERY LOW DEMAND,
                   // WE DONT SERVE LEFT TURN!! IF THERE IS NO CAR ON THE DETECTORS
            {
                Temp_event.time = cp[1][1][i] - 0.2; // 0.2 seconds before force off we put a call
                Temp_event.action = VehCall;
                Temp_event.phase =
                    ((int)(cp[1][2][i + 1])) + 5; // converting phase number from 0-3 to  5-8
                m_eventListR2.emplace_back(Temp_event);
            }
        }

        if ((((int)(cp[1][2][i])) + 5) == 6) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  If it is
                                             // neccessary to force off 6, we should call 8 before
                                             // that. Also if it is necessary to forec off 8, we
                                             // should call 6 before that.
        {
            Temp_event.time = cp[1][1][i] - 0.2; // 0.2 seconds before force off we put a call
            Temp_event.action = VehCall;
            Temp_event.phase = 8; // converting phase number from 0-3 to  5-8
            m_eventListR2.emplace_back(Temp_event);
        }
        if ((((int)(cp[1][2][i])) + 5) == 8) // JUST PUT CALL FOR MAJOR MOVEMENT!!!!!!!!!  If it is
                                             // neccessary to force off 6, we should call 8 before
                                             // that. Also if it is necessary to forec off 8, we
                                             // should call 6 before that.
        {
            Temp_event.time = cp[1][1][i] - 0.2; // 0.2 seconds before force off we put a call
            Temp_event.action = VehCall;
            Temp_event.phase = 6; // converting phase number from 0-3 to  5-8
            m_eventListR2.emplace_back(Temp_event);
        }
        // call, this call is neccesary when we run on Coordination. In coordination mode,
        // coordinated phase should alway have a call
        if (rsuCfg.dCoordinationWeight > 0)
        {
            // Call in every 1.8 seconds (should be less than backup time, which for now it is
            // assume to be 2sec) for all of the times in the feasible set
            double iii = 0.0;
            if ((cp[1][2][i] + 5) == rsuCfg.iCoordinatedPhase[1])
            {
                if (i > 0)
                {
                    while (iii < (cp[1][1][i] - cp[1][0][i - 1]))
                    {
                        Temp_event.time = (cp[1][0][i - 1] + iii);
                        Temp_event.action = VehCall;
                        Temp_event.phase = rsuCfg.iCoordinatedPhase[1];
                        m_eventListR2.emplace_back(Temp_event);
                        iii = iii + 1.9;
                    }
                    Temp_event.time = (cp[1][0][i - 1]) - 0.2;
                    Temp_event.action = VehCall;
                    Temp_event.phase = rsuCfg.iCoordinatedPhase[1];
                    m_eventListR2.emplace_back(Temp_event);
                }
                if (i == 0)
                {
                    while (iii < cp[1][0][i])
                    {
                        Temp_event.time = iii;
                        Temp_event.action = VehCall;
                        Temp_event.phase = rsuCfg.iCoordinatedPhase[1];
                        m_eventListR2.emplace_back(Temp_event);
                        iii = iii + 1.9;
                    }
                    Temp_event.time = (cp[1][0][i]) - 0.2;
                    Temp_event.action = VehCall;
                    Temp_event.phase = rsuCfg.iCoordinatedPhase[1];
                    m_eventListR2.emplace_back(Temp_event);
                }
            }
        }
        // force off
        Temp_event.time = cp[1][1][i];
        Temp_event.action = ForceOff;
        Temp_event.phase = (int)(cp[1][2][i] + 5); // converting phase number from 0-3 to  5-8
        m_eventListR2.emplace_back(Temp_event);
    }
    for (const auto& event : m_eventListR1)
    {
        ITSAPP_LOG("Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
    }
    for (const auto& event : m_eventListR2)
    {
        ITSAPP_LOG("Time: %lf, Phase: %d, Action: %d", event.time, event.phase, event.action);
    }
}

void MmitssPrioritySolver::readOptPlanFromFile()
{
    const RsuConfig::RSU_Config& rsuCfg = m_cfg.getConfig();

    PriorityRequestList PrioReqList[2];
    for (int k = 0; k < 2; k++)
        for (int kk = 0; kk < 3; kk++)
            for (int kkk = 0; kkk < 5; kkk++)
                m_criticalPoints[k][kk][kkk] = 0.0;
    fstream fss;
    fss.open(SieMmitss::getGlpkResultsFilePath(), fstream::in);
    if (!fss)
    {
        ITSAPP_WRN("***********Error opening the plan file!");
        return;
        ;
    }
    string lineread;
    int SP[2];
    double InitTime1[2], InitGrn[2];
    double V[2][8], g[2][8];
    int ReqNo = 0;
    int ReqPhaseNo = 0;
    double ReqRl = 0.0;
    double ReqRu = 0.0;
    double ReqDelay = 0.0;
    double dTotalDelay = 0.0;
    VEH_PRIORITY ReqType = VEH_PRIORITY::NORMAL;
    //-------------------------------Begin of Read Plan----------------------------//
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%d %d", &SP[0], &SP[1]);
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%lf %lf %lf %lf", &InitTime1[0], &InitTime1[1], &InitGrn[0],
        &InitGrn[1]);
    for (int i = 0; i < 2; i++)
    {
        getline(fss, lineread);
        sscanf(lineread.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &V[i][0], &V[i][1], &V[i][2],
            &V[i][3], &V[i][4], &V[i][5], &V[i][6], &V[i][7]);
    }
    for (int i = 0; i < 2; i++)
    {
        getline(fss, lineread);
        sscanf(lineread.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &g[i][0], &g[i][1], &g[i][2],
            &g[i][3], &g[i][4], &g[i][5], &g[i][6], &g[i][7]);
    }
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%d", &ReqNo);
    ITSAPP_LOG("Number of Requests=%d", ReqNo);
    for (int i = 0; i < ReqNo; i++)
    {
        getline(fss, lineread);
        int type = 0;
        sscanf(
            lineread.c_str(), "%d %lf %lf %lf %d ", &ReqPhaseNo, &ReqRl, &ReqRu, &ReqDelay, &type);
        ReqType = static_cast<VEH_PRIORITY>(type);
        ReqPhaseNo = ReqPhaseNo - 1;
        int CurRing = FindRingNo(ReqPhaseNo);
        if (ReqPhaseNo <=
            10) // Double check if the req in next cycle is not being process in current cycl!!!
        {
            if (ReqPhaseNo < SP[CurRing] - 1)
                ReqPhaseNo = ReqPhaseNo * 10;
            // check if the coordination request for next cycle is not being processed in the
            // current cycle!!!
            if (ReqType == VEH_PRIORITY::COORDINATION)
            {
                if ((g[0][1] == 0) || (g[0][5] == 0))
                    ReqPhaseNo = ReqPhaseNo * 10;
            }
        }
        int phase_in_ring = ReqPhaseNo - 4 * CurRing; // If in ring 2, phase should -4.
        ITSAPP_LOG("ReqPhaseNo=%d, phase_in_ring=%d", ReqNo, phase_in_ring);
        PriorityRequest PriorityRequest_t =
            PriorityRequest(phase_in_ring, ReqRl, ReqRu, ReqDelay, ReqType);
        PrioReqList[CurRing].emplace_back(PriorityRequest_t);
    }
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%lf ", &dTotalDelay);
    ITSAPP_LOG("Total Delay of Requests: %lf", dTotalDelay);
    fss.close();

    int R1No = rsuCfg.Ring1No;
    int R2No = rsuCfg.Ring2No;
    SP[0] = SP[0] - 1;
    SP[1] = SP[1] - 5;
    int SPIdx1 = FindIndexArray(rsuCfg.Phase_Seq_R1, R1No, SP[0]);
    int SPIdx2 = FindIndexArray(rsuCfg.Phase_Seq_R2, R2No, SP[1]);
    ITSAPP_LOG("SP index1=%d, index2=%d", (SPIdx1 + 1), (SPIdx2 + 1));
    int t1 = R1No * 2 - SPIdx1;
    int t2 = R2No * 2 - SPIdx2;
    double* Split1 = new double[t1];
    double* Split2 = new double[t2];
    double* mustChangeTime1 = new double[t1];
    double* mustChangeTime2 = new double[t2];
    std::vector<int> Phase1 =
        GeneratePhaseArray(SP[0], rsuCfg.Phase_Seq_R1, R1No, t1, 1); // TOTAL t1 & t2 "Array.h"
    std::vector<int> Phase2 = GeneratePhaseArray(
        SP[1], rsuCfg.Phase_Seq_R2, R2No, t2, 1); // Phase1 including the cycle information
    int ii = 0;
    for (int i = 0; i < 2; i++) // Cycle
    {
        for (int k = 0; k < R1No; k++) // Phase
        {
            int RP = rsuCfg.Phase_Seq_R1[k];
            if (V[i][RP] != 0)
            {
                Split1[ii] = V[i][RP];
                ii++;
            }
        }
    }
    for (int i = 0; i < t1; i++)
    {
        int phase_no = Phase1[i];
        int ring_no = phase_no / 10;
        int real_phase = phase_no % 10;
        mustChangeTime1[i] = SumArray<double>(Split1, t1, 0, i) + g[ring_no][real_phase];
        mustChangeTime1[i] = mustChangeTime1[i] + InitTime1[0];
    }
    PrintArray(Phase1, t1);
    PrintArray(Split1, t1);
    PrintArray(mustChangeTime1, t1);

    ii = 0;
    for (int i = 0; i < 2; i++) // Cycle
    {
        for (int k = 0; k < R2No; k++) // Phase
        {
            int RP = rsuCfg.Phase_Seq_R2[k] + 4; // Ring 2 shuold +4; (should be real phase -1)
            if (V[i][RP] != 0)
            {
                Split2[ii] = V[i][RP];
                ii++;
            }
        }
    }
    for (int i = 0; i < t2; i++)
    {
        int phase_no = Phase2[i];
        int ring_no = phase_no / 10;
        int real_phase = phase_no % 10 + 4; // Ring 2 shuold +4; (should be real phase -1)
        mustChangeTime2[i] = SumArray<double>(Split2, t2, 0, i) + g[ring_no][real_phase];
        mustChangeTime2[i] = mustChangeTime2[i] + InitTime1[1];
    }

    PrintArray(Phase2, t2);
    PrintArray(Split2, t2);
    PrintArray(mustChangeTime2, t2);

    int iPosOfEarliestReqInPrioReqList1 = findTheEarliestReqInERP(PrioReqList[0]); // in ring 1
    int iPosOfEarliestReqInPrioReqList2 = findTheEarliestReqInERP(PrioReqList[1]); // in ring 2

    // if we face very congested situation or when we run adaptive control, follow the optimal
    // solution from solver
    if (m_congestion == CONGESTED || m_codeUsage == ADAPTIVE_PRIORITY)
        creatCPexactlyAsOptSol(mustChangeTime1, Phase1, t1, mustChangeTime2, Phase2, t2);
    else
    {
        if (m_codeUsage == PRIORITY) //  if priority and actuation logic is applied (Send optimal
                                     //  schedule to Interface )
        {
            if (dTotalDelay > 0) // if the total delay of requests is positive, we have to follow
                                 // exactly the solution of the optimizer and therefore there is no
                                 // flexibility
                creatCPexactlyAsOptSol(mustChangeTime1, Phase1, t1, mustChangeTime2, Phase2, t2);
            else
            {
                if (iPosOfEarliestReqInPrioReqList1 > -1)
                {
                    creatFeasibleRegionRing(0, PrioReqList[0], iPosOfEarliestReqInPrioReqList1,
                        mustChangeTime1, Phase1, t1, InitTime1[0], InitGrn[0],
                        iPosOfEarliestReqInPrioReqList2);
                }
                if (iPosOfEarliestReqInPrioReqList2 > -1)
                {
                    creatFeasibleRegionRing(1, PrioReqList[1], iPosOfEarliestReqInPrioReqList2,
                        mustChangeTime2, Phase2, t2, InitTime1[1], InitGrn[1],
                        iPosOfEarliestReqInPrioReqList1);
                }
            }
        }
        else //  if priority + intelligent phase allocation alg applied (Send optimal schedule to
             //  COP ) // in this case, if we have request in one ring, and coordination in the
             //  second ring, we focuse on priority request !!!
        {
            VEH_PRIORITY tempTypeR1 = VEH_PRIORITY::NORMAL;
            VEH_PRIORITY tempTypeR2 = VEH_PRIORITY::NORMAL;
            if (iPosOfEarliestReqInPrioReqList1 > -1)
            {
                tempTypeR1 = getType(PrioReqList[0], iPosOfEarliestReqInPrioReqList1);
            }
            if (iPosOfEarliestReqInPrioReqList2 > -1)
            {
                tempTypeR2 = getType(PrioReqList[1], iPosOfEarliestReqInPrioReqList2);
            }
            if (tempTypeR1 == VEH_PRIORITY::COORDINATION &&
                tempTypeR2 != VEH_PRIORITY::COORDINATION)
                creatFeasibleRegionRing(1, PrioReqList[1], iPosOfEarliestReqInPrioReqList2,
                    mustChangeTime2, Phase2, t2, InitTime1[1], InitGrn[1],
                    iPosOfEarliestReqInPrioReqList1);
            else if (tempTypeR2 == VEH_PRIORITY::COORDINATION &&
                     tempTypeR1 != VEH_PRIORITY::COORDINATION)
                creatFeasibleRegionRing(0, PrioReqList[0], iPosOfEarliestReqInPrioReqList1,
                    mustChangeTime1, Phase1, t1, InitTime1[0], InitGrn[0],
                    iPosOfEarliestReqInPrioReqList2);
            else if ((tempTypeR2 != VEH_PRIORITY::COORDINATION &&
                         tempTypeR1 != VEH_PRIORITY::COORDINATION) ||
                     (tempTypeR2 == VEH_PRIORITY::COORDINATION &&
                         tempTypeR1 == VEH_PRIORITY::COORDINATION))
            {
                if (iPosOfEarliestReqInPrioReqList1 > -1)
                {
                    creatFeasibleRegionRing(0, PrioReqList[0], iPosOfEarliestReqInPrioReqList1,
                        mustChangeTime1, Phase1, t1, InitTime1[0], InitGrn[0],
                        iPosOfEarliestReqInPrioReqList2);
                }
                if (iPosOfEarliestReqInPrioReqList2 > -1)
                {
                    creatFeasibleRegionRing(1, PrioReqList[1], iPosOfEarliestReqInPrioReqList2,
                        mustChangeTime2, Phase2, t2, InitTime1[1], InitGrn[1],
                        iPosOfEarliestReqInPrioReqList1);
                }
            }
        }
    }
    delete[] mustChangeTime1;
    delete[] mustChangeTime2;
    delete[] Split1;
    delete[] Split2;
}

void MmitssPrioritySolver::readOptPlanFromFileForEV(const RsuConfig& configEV)
{
    const auto& rsuCfg = m_cfg.getConfig();
    const auto& rsuCfgEV = configEV.getConfig();

    PriorityRequestList PrioReqList[2];
    // Figuring out which phases are missing
    for (int j = 0; j < 8; j++)
        m_omitPhase[j] = 0;
    for (int j = 0; j < 8; j++)
        if (rsuCfgEV.Gmax[j] == 0 && rsuCfg.Gmax[j] > 0)
            m_omitPhase[j] = j + 1;

    for (int k = 0; k < 2; k++)
        for (int kk = 0; kk < 3; kk++)
            for (int kkk = 0; kkk < 5; kkk++)
                m_criticalPoints[k][kk][kkk] = 0.0;

    fstream fss;
    fss.open(SieMmitss::getGlpkResultsFilePath(), fstream::in);
    if (!fss)
    {
        ITSAPP_WRN("***********Error opening the plan file!");
        return;
    }
    string lineread;
    int SP[2];
    double InitTime1[2], InitGrn[2];
    double V[2][8], g[2][8];
    int ReqNo = 0;
    int ReqPhaseNo = 0;
    double ReqRl = 0.0;
    double ReqRu = 0.0;
    double ReqDelay = 0.0;
    VEH_PRIORITY ReqType = VEH_PRIORITY::NORMAL;
    //-------------------------------Begin of Read Plan----------------------------//
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%d %d", &SP[0], &SP[1]);
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%lf %lf %lf %lf", &InitTime1[0], &InitTime1[1], &InitGrn[0],
        &InitGrn[1]);
    for (int i = 0; i < 2; i++)
    {
        getline(fss, lineread);
        sscanf(lineread.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &V[i][0], &V[i][1], &V[i][2],
            &V[i][3], &V[i][4], &V[i][5], &V[i][6], &V[i][7]);
    }
    for (int i = 0; i < 2; i++)
    {
        getline(fss, lineread);
        sscanf(lineread.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &g[i][0], &g[i][1], &g[i][2],
            &g[i][3], &g[i][4], &g[i][5], &g[i][6], &g[i][7]);
    }
    getline(fss, lineread);
    sscanf(lineread.c_str(), "%d", &ReqNo);
    for (int i = 0; i < ReqNo; i++)
    {
        getline(fss, lineread);
        int type = 0;
        sscanf(
            lineread.c_str(), "%d %lf %lf %lf %d ", &ReqPhaseNo, &ReqRl, &ReqRu, &ReqDelay, &type);
        ReqType = static_cast<VEH_PRIORITY>(type);
        ReqPhaseNo = ReqPhaseNo - 1;
        int CurRing = FindRingNo(ReqPhaseNo);
        if (ReqPhaseNo <=
            10) // Double check if the req in next cycle is not being process in current cycl!!!
        {
            if (ReqPhaseNo < SP[CurRing] - 1)
                ReqPhaseNo = ReqPhaseNo * 10;
        }
        int phase_in_ring = ReqPhaseNo - 4 * CurRing; // If in ring 2, phase should -4.
        PriorityRequest PriorityRequest_t =
            PriorityRequest(phase_in_ring, ReqRl, ReqRu, ReqDelay, ReqType);
        PrioReqList[CurRing].emplace_back(PriorityRequest_t);
    }
    fss.close();

    int R1No = rsuCfgEV.Ring1No;
    int R2No = rsuCfgEV.Ring2No;
    SP[0] = SP[0] - 1;
    SP[1] = SP[1] - 5;
    int SPIdx1 = FindIndexArray(rsuCfgEV.Phase_Seq_R1, R1No, SP[0]);
    int SPIdx2 = FindIndexArray(rsuCfgEV.Phase_Seq_R2, R2No, SP[1]);
    ITSAPP_LOG("SP index1=%d, index2+%d", (SPIdx1 + 1), (SPIdx2 + 1));

    int t1 = -1; // R1No*2-SPIdx1;
    int t2 = -1; // R2No*2-SPIdx2;
    double* Split1;
    double* Split2;
    double* mustChangeTime1;
    double* mustChangeTime2;
    int* Phase1; //=GeneratePhaseArray(SP[0],ConfigIS_EV.Phase_Seq_R1,R1No,t1,1); // TOTAL t1 & t2
                 //"Array.h"
    int* Phase2; //=GeneratePhaseArray(SP[1],ConfigIS_EV.Phase_Seq_R2,R2No,t2,1); // Phase1
                 // including the cycle information

    ITSAPP_LOG("R1No=%d, R2No=%d", R1No, R2No);
    int ii = 0;
    if (R1No == 1)
    {
        t1 = 1; // only one phase
        Phase1 = new int[t1];
        Split1 = new double[t1];
        mustChangeTime1 = new double[t1];
        Split1[0] = 0.0;
        std::vector<int> Phase11 =
            GeneratePhaseArray(SP[0], rsuCfgEV.Phase_Seq_R1, R1No, t1, 1); // TOTAL t1=1
        Phase1[0] = Phase11[0];
        //------ IF only have one phase in a ring, we need to add 2 cycles' result up.
        ii = 0;
        int RP1 = rsuCfgEV.Phase_Seq_R1[0] % 10; // IS (Real phase -1)
        for (int i = 0; i < 2; i++)              // Cycle
        {
            Split1[ii] += V[i][RP1];
        }
        int CurPhase = Phase1[ii] % 10; // Ring 2 shuold +4;
        mustChangeTime1[ii] = Split1[ii] - rsuCfgEV.Yellow[CurPhase] - rsuCfgEV.Red[CurPhase];
        PrintArray<int>(Phase1, t1);
        PrintArray<double>(Split1, t1);
        PrintArray<double>(mustChangeTime1, t1);
    }
    else
    {
        t1 = R1No * 2 - SPIdx1;
        Phase1 = new int[t1];
        Split1 = new double[t1];
        mustChangeTime1 = new double[t1];
        std::vector<int> Phase11 = GeneratePhaseArray(SP[0], rsuCfgEV.Phase_Seq_R1, R1No, t1, 1);
        //~ cout << Phase11[0] << endl;
        //~ cout << Phase11[1] << endl;
        //~
        for (int jj = 0; jj < t1; jj++)
        {
            Phase1[jj] = Phase11[jj];
        }
        ii = 0;
        for (int i = 0; i < 2; i++) // Cycle
        {
            for (int k = 0; k < R1No; k++) // Phase
            {
                int RP = rsuCfgEV.Phase_Seq_R1[k];
                if (V[i][RP] != 0)
                {
                    Split1[ii] = V[i][RP];
                    ii++;
                }
            }
        }
        for (int i = 0; i < t1; i++)
        {
            int phase_no = Phase1[i];
            int ring_no = phase_no / 10; // Cycle
            int real_phase = phase_no % 10;
            mustChangeTime1[i] = SumArray<double>(Split1, t1, 0, i) + g[ring_no][real_phase];
            mustChangeTime1[i] = mustChangeTime1[i] + InitTime1[0];
        }
        PrintArray<int>(Phase1, t1);
        PrintArray<double>(Split1, t1);
        PrintArray<double>(mustChangeTime1, t1);
    }

    if (R2No == 1)
    {
        if (SPIdx2 < 0)
            ITSAPP_LOG("***********Starting Phase is not in Ring 2 Sequence.***********");
        t2 = 1; // only one phase
        Phase2 = new int[t2];
        Split2 = new double[t2];
        mustChangeTime2 = new double[t2];
        Split2[0] = 0.0;
        std::vector<int> Phase22 =
            GeneratePhaseArray(SP[1], rsuCfgEV.Phase_Seq_R2, R2No, t2, 1); // TOTAL t1=1
        Phase2[0] = Phase22[0];
        //------ IF only have one phase in a ring, we need to add 2 cycles' result up.
        ii = 0;
        int RP2 = rsuCfgEV.Phase_Seq_R2[0] % 10 + 4; // IS (Real phase -1)
        for (int i = 0; i < 2; i++)                  // Cycle
        {
            Split2[ii] += V[i][RP2];
        }
        int CurPhase = Phase2[ii] % 10 + 4; // Ring 2 shuold +4;
        mustChangeTime2[ii] = Split2[ii] - rsuCfgEV.Yellow[CurPhase] - rsuCfgEV.Red[CurPhase];
        PrintArray<int>(Phase2, t2);
        PrintArray<double>(Split2, t2);
        PrintArray<double>(mustChangeTime2, t2);
    }
    else
    {

        t2 = R2No * 2 - SPIdx2;
        Phase2 = new int[t2];
        Split2 = new double[t2];
        mustChangeTime2 = new double[t2];
        std::vector<int> Phase22 = GeneratePhaseArray(SP[1], rsuCfgEV.Phase_Seq_R2, R2No, t2, 1);
        for (int jj = 0; jj < t2; jj++)
            Phase2[jj] = Phase22[jj];
        ii = 0;
        for (int i = 0; i < 2; i++) // Cycle
        {
            for (int k = 0; k < R2No; k++) // Phase
            {
                int RP = rsuCfgEV.Phase_Seq_R2[k] % 10 + 4;
                if (V[i][RP] != 0)
                {
                    Split2[ii] = V[i][RP];
                    //  cout << " Split2[ii]" <<  Split2[ii] << endl;
                    ii++;
                }
            }
        }
        for (int i = 0; i < t2; i++)
        {
            int phase_no = Phase2[i];
            int ring_no = phase_no / 10; // Cycle
            int real_phase = phase_no % 10 + 4;
            mustChangeTime2[i] = SumArray<double>(Split2, t2, 0, i) + g[ring_no][real_phase];
            mustChangeTime2[i] = mustChangeTime2[i] + InitTime1[1];
        }
        PrintArray<int>(Phase2, t2);
        PrintArray<double>(Split2, t2);
        PrintArray<double>(mustChangeTime2, t2);
    }
    int iPosOfEarliestReqInPrioReqList1 = findTheEarliestReqInERP(PrioReqList[0]); // in ring 1
    int iPosOfEarliestReqInPrioReqList2 = findTheEarliestReqInERP(PrioReqList[1]); // in ring 2
    if (iPosOfEarliestReqInPrioReqList1 > -1)
    {
        creatFeasibleRegionRing_EV(m_criticalPoints[0], PrioReqList[0],
            iPosOfEarliestReqInPrioReqList1, mustChangeTime1, Phase1, t1);
    }
    if (iPosOfEarliestReqInPrioReqList2 > -1)
    {
        creatFeasibleRegionRing_EV(m_criticalPoints[1], PrioReqList[1],
            iPosOfEarliestReqInPrioReqList2, mustChangeTime2, Phase2, t2);
    }

    delete[] mustChangeTime1;
    delete[] mustChangeTime2;
    delete[] Split1;
    delete[] Split2;
}

void MmitssPrioritySolver::LinkList2DatFile(const RsuConfig& cfg)
{
    //-- Convert request linkedlist to the NewModel.dat file for GLPK solver.
    //-- Init1 and Init2 are the initial time for the two rings while current phases are in R or Y.
    //-- MinGrn1 and MinGrn2 are the elapsed time when current phase is in Green

    ofstream fs;
    fs.open(SieMmitss::getPriorityDataFilePath(), ios::out);

    const RsuConfig::RSU_Config& ConfigIS = cfg.getConfig();
    int R1No = ConfigIS.Ring1No;
    int R2No = ConfigIS.Ring2No;

    fs << "data;\n";
    if (m_haveCoordInList) // in order to know in which
    {
        fs << m_reqListCombined.getParamFromConfig(cfg);
    }
    else
        fs << "param current:=0;\n";

    fs << "param SP1:=" << m_initPhase[0] << ";\n"; // This is the real phase [1-4]
    fs << "param SP2:=" << m_initPhase[1] << ";\n"; // This is the real phase [5-8]

    for (int i = 0; i < 2; i++)
    {
        if (m_initTime[i] < 0)
        {
            m_initTime[i] = 0;
        }
    }

    fs << "param init1:=" << m_initTime[0] << ";\n";
    fs << "param init2:=" << m_initTime[1] << ";\n";
    fs << "param Grn1 :=" << m_grnElapse[0] << ";\n";
    fs << "param Grn2 :=" << m_grnElapse[1] << ";\n";

    //=================Add the information for Yellow, Red======//
    fs << "param y       \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = ConfigIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << ConfigIS.Yellow[k];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = ConfigIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << ConfigIS.Yellow[k + 4];
    }
    fs << ";\n";

    fs << "param red       \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = ConfigIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << ConfigIS.Red[k];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = ConfigIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << ConfigIS.Red[k + 4];
    }
    fs << ";\n";

    fs << "param gmin      \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = ConfigIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << ConfigIS.Gmin[k];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = ConfigIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << ConfigIS.Gmin[k + 4];
    }
    fs << ";\n";

    /*
    fs << "param gmax      \t:=";
    for(int i=0;i<R1No;i++)
    {
        int k=ConfigIS.Phase_Seq_R1[i];

        fs << "\t" << (k+1) << "  " << ConfigIS.Gmax[k];
    }
    for(int i=0;i<R2No;i++)
    {
        int k=ConfigIS.Phase_Seq_R2[i];

        fs << "\t" << (k+5) << "  " << ConfigIS.Gmax[k+4];
    }
    fs << ";\n\n";
    //*/

    // adding green extention portion to the max green time for the requested phases
    int iGmax[2][4] = {}; // max green time of each phase in ring 1
    for (int i = 0; i < R1No; i++)
    {
        int k = ConfigIS.Phase_Seq_R1[i];
        if (m_reqListCombined.FindRequestPhaseInList((k + 1)) > 0)
        {
            m_globalGmax[i] = (ConfigIS.Gmax[k] * (1 + m_maxGrnExtPortion));
            m_globalGmax[i + 4] = m_globalGmax[i];
            iGmax[0][i] = (int)m_globalGmax[i];
        }
        else
        {
            m_globalGmax[i] = ConfigIS.Gmax[k];
            iGmax[0][i] = (int)m_globalGmax[i];
        }
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = ConfigIS.Phase_Seq_R2[i];
        if (m_reqListCombined.FindRequestPhaseInList((k + 5)) > 0)
        {
            m_globalGmax[i + 4] = (ConfigIS.Gmax[k + 4] * (1 + m_maxGrnExtPortion));
            m_globalGmax[i] = m_globalGmax[i + 4];
            iGmax[1][i] = (int)m_globalGmax[i + 4];
        }
        else
        {
            m_globalGmax[i + 4] = max(ConfigIS.Gmax[k + 4], m_globalGmax[i + 4]);
            iGmax[1][i] = (int)m_globalGmax[i + 4];
        }
    }

    // in coordination case, the maximum green time for the two coordinated phase should be the same
    // because of MILP max constraints!!
    if (ConfigIS.dCoordinationWeight > 0)
    {
        int temp_indx1 = 0;
        double temp_val1 = 0;
        for (int i = 0; i < R1No; i++)
        {
            int k = ConfigIS.Phase_Seq_R1[i];
            if (k + 1 == ConfigIS.iCoordinatedPhase[0])
            {
                temp_val1 = m_globalGmax[i];
                temp_indx1 = i;
            }
        }
        for (int i = 0; i < R2No; i++)
        {
            int k = ConfigIS.Phase_Seq_R2[i];
            if (k + 5 == ConfigIS.iCoordinatedPhase[1])
            {
                m_globalGmax[i + 4] = max(m_globalGmax[i + 4], temp_val1);
                m_globalGmax[temp_indx1] = max(m_globalGmax[i + 4], temp_val1);
                iGmax[1][i] = (int)m_globalGmax[i + 4];
                iGmax[0][temp_indx1] = (int)m_globalGmax[i + 4];
            }
        }
    }

    fs << "param gmax      \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = ConfigIS.Phase_Seq_R1[i];
        // if ( ( (InitPhase[0]==k+1) && (HaveCoordInList!=1) ) || ((InitPhase[0]==k+1) &&
        // (HaveCoordInList==1) && (InitPhase[0]!=ConfigIS.iCoordinatedPhase[0]) )) // if the
        // current phase is not the coordinated phase. or if the coordiantion is not on
        //        m_globalGmax[i]=m_globalGmax[i]-GrnElapse[0];
        //    else if ((InitPhase[0]==k+1) && (HaveCoordInList==1) &&
        //    (InitPhase[0]==ConfigIS.iCoordinatedPhase[0]) && (icurrent<
        //    ConfigIS.iCoordinationSplit)) // if we are in the coordinated split time interval, we
        //    should not reduce the length of max green time
        //        m_globalGmax[i]=m_globalGmax[i];
        if (m_globalGmax[i] <= ConfigIS.Gmin[k])
            m_globalGmax[i] = ConfigIS.Gmin[k];
        fs << "\t" << (k + 1) << "  " << m_globalGmax[i];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = ConfigIS.Phase_Seq_R2[i];
        //  if ( ( (InitPhase[1]==k+5) && (HaveCoordInList!=1) ) || ((InitPhase[1]==k+5) &&
        //  (HaveCoordInList==1) && (InitPhase[1]!=ConfigIS.iCoordinatedPhase[1]) )) // if the
        //  current phase is not the coordinated phase. or if the coordiantion is not on
        //    m_globalGmax[i+4]=m_globalGmax[i+4]-GrnElapse[1];
        // else if ((InitPhase[1]==k+5) && (HaveCoordInList==1) &&
        // (InitPhase[1]==ConfigIS.iCoordinatedPhase[1]) && (icurrent< ConfigIS.iCoordinationSplit))
        // // if we are in the coordinated split time interval, we should not reduce the length of
        // max green time
        //    m_globalGmax[i+4]=m_globalGmax[i+4];
        if (m_globalGmax[i + 4] <= ConfigIS.Gmin[k + 4])
            m_globalGmax[i + 4] = ConfigIS.Gmin[k + 4];
        fs << "\t" << (k + 5) << "  " << m_globalGmax[i + 4];
    }

    fs << ";\n\n";

    int NumberofRequests = 0;
    int iNumberofTransitInList = 1;
    int iNumberofTruckInList = 1;
    fs << "param priorityType:= ";
    if (m_reqListCombined.empty() == false)
    {
        fs << m_reqListCombined.allNonCoordinationVehClassAsString(
            NumberofRequests, iNumberofTransitInList, iNumberofTruckInList);

        while (NumberofRequests < 10)
        {
            NumberofRequests++;
            fs << NumberofRequests;
            fs << " ";
            fs << 0;
            fs << " ";
        }
        fs << " ;  \n";
    }
    else
    {
        fs << " 1 0 2 0 3 5 4 0 5 0 6 0 7 0 8 0 9 0 10 0 ; \n";
    }

    if (iNumberofTransitInList > 1)
        iNumberofTransitInList = iNumberofTransitInList - 1;
    if (iNumberofTruckInList > 1)
        iNumberofTruckInList = iNumberofTruckInList - 1;

    fs << "param PrioWeigth:=  1 1 2 ";
    fs << cfg.getConfig().iTransitWeight / iNumberofTransitInList;
    fs << " 3 ";
    fs << cfg.getConfig().iTruckWeight / iNumberofTruckInList;
    fs << " 4 0 5 0 ";
    fs << " 6 ";
    fs << cfg.getConfig().dCoordinationWeight;
    fs << " 7 0 8 0 9 0 10 0 ; \n";

    //================End of the information for Yellow, Red======//

    //---------------According to priorityconfiguration file, some priority eligible vehicle may
    // have wight equal to zero. we shoudl remove them from the list
    RequestList Req_List_New;
    if (m_reqListCombined.empty() == false)
    {
        m_reqListCombined.getNonZeroWeightReq(
            cfg.getConfig().iTransitWeight, cfg.getConfig().iTruckWeight, Req_List_New);
    }

    fs << "param Rl (tr): 1 2 3 4 5 6 7 8:=\n";
    int ReqSeq = 1;
    for (auto& req : Req_List_New)
    {
        if (req.VehClass != VEH_PRIORITY::COORDINATION) // if it is a coordination request, we will
                                                        // check it in at it in the Cu and Cl
        {
            fs << ReqSeq << "  ";
            for (int j = 1; j <= 8; j++)
            {
                if (req.Phase == j)
                {
                    if (req.MinGreen > 0) // in this case the vhicle is in the queue and we should
                                          // set the Rl as less as possible!!!  // MZ Added to hedge
                                          // against the worst case that may happen when the vehicle
                                          // is in the queue
                    {
                        int iRingOfTheRequest = FindRingNo(j - 1);
                        fs << 1 << "  ";
                        if (iGmax[iRingOfTheRequest][(j - 1) % 4] <= 0)
                        {
                            if (j % 2 == 0)
                                iGmax[iRingOfTheRequest][(j - 1) % 4] =
                                    iGmax[iRingOfTheRequest][(j - 1) % 4 - 1];
                            else
                                iGmax[iRingOfTheRequest][(j - 1) % 4] =
                                    iGmax[iRingOfTheRequest][(j - 1) % 4 + 1];
                        }
                    }
                    else
                        fs << max(req.ETA - ROBUSTTIME_LOW, float(1.0)) << "  ";
                }
                else
                    fs << ".  ";
            }
            fs << "\n";
            ReqSeq = ReqSeq + 1;
        }
    }
    fs << ";\n";
    fs << "param Ru (tr): 1 2 3 4 5 6 7 8:=\n";
    ReqSeq = 1;
    for (auto& req : Req_List_New)
    {
        if (req.VehClass != VEH_PRIORITY::COORDINATION) // if it is a coordination request, we will
                                                        // check it in at it in the Cu and Cl
        {
            fs << ReqSeq << "  ";
            for (int j = 1; j <= 8; j++)
            {
                if (req.Phase == j)
                {
                    if (req.MinGreen > 0) // in this case the vhicle is in the queue and we should
                                          // set the Ru at most less than maximum remaining green
                                          // time!!  // MZ Added to hedge against the worst case
                                          // that may happen when the vehicle is in the queue
                    {
                        int iRingOfTheRequestt = FindRingNo(j - 1);
                        double dmaxGreenForThisPhase = 0.0;
                        if (iRingOfTheRequestt == 1)
                        {
                            for (int i = 0; i < R2No; i++)
                            {
                                int kkk = ConfigIS.Phase_Seq_R2[i];
                                if (kkk + 5 == j)
                                    dmaxGreenForThisPhase = m_globalGmax[i + 4];
                            }
                        }
                        if (iRingOfTheRequestt == 0)
                        {
                            for (int i = 0; i < R1No; i++)
                            {
                                int kkk = ConfigIS.Phase_Seq_R1[i];
                                if (kkk + 1 == j)
                                    dmaxGreenForThisPhase = m_globalGmax[i];
                            }
                        }

                        if (req.MinGreen + ROBUSTTIME_UP >= (float)dmaxGreenForThisPhase)
                            fs << dmaxGreenForThisPhase << "  ";
                        else
                            fs << req.MinGreen + ROBUSTTIME_UP << "  ";
                    }
                    else
                        fs << req.ETA + ROBUSTTIME_UP << "  ";
                }
                else
                    fs << ".  ";
            }
            fs << "\n";
            ReqSeq = ReqSeq + 1;
        }
    }
    fs << ";\n";
    if (m_haveCoordInList) // in order to know in which
    {
        fs << "param Cl (tr): 1 2 3 4 5 6 7 8:=\n";
        ReqSeq = 1;
        for (auto& req : Req_List_New)
        {
            if (req.VehClass == VEH_PRIORITY::COORDINATION)
            {
                fs << ReqSeq << "  ";
                for (int j = 1; j <= 8; j++)
                {
                    if (req.Phase == j)
                    {
                        if (req.MinGreen > 0) // in this case the vhicle is in the queue and we
                                              // should set the Ru at most less than maximum
                                              // remaining green time!!  // MZ Added to hedge
                                              // against the worst case that may happen when the
                                              // vehicle is in the queue
                            fs << 1 << "  ";
                        else
                            fs << req.ETA << "  ";
                    }
                    else
                        fs << ".  ";
                }
                fs << "\n";
                ReqSeq = ReqSeq + 1;
            }
        }
        fs << ";\n";

        fs << "param Cu (tr): 1 2 3 4 5 6 7 8:=\n";
        ReqSeq = 1;
        for (auto& req : Req_List_New)
        {
            if (req.VehClass == VEH_PRIORITY::COORDINATION)
            {
                fs << ReqSeq << "  ";
                for (int j = 1; j <= 8; j++)
                {
                    if (req.Phase == j)
                    {
                        if (req.MinGreen > 0) // in this case the vhicle is in the queue and we
                                              // should set the Ru at most less than maximum
                                              // remaining green time!!  // MZ Added to hedge
                                              // against the worst case that may happen when the
                                              // vehicle is in the queue
                            fs << req.MinGreen << "  ";
                        else
                        {
                            if (ConfigIS.dCoordinationSplit[0] == ConfigIS.dCoordinationSplit[1])
                                fs << req.ETA + (float)ConfigIS.dCoordinationSplit[0] << "  ";
                            else
                            {
                                if (req.Phase == ConfigIS.iCoordinatedPhase[0])
                                    fs << req.ETA + (float)ConfigIS.dCoordinationSplit[0] << "  ";
                                if (req.Phase == ConfigIS.iCoordinatedPhase[1])
                                    fs << req.ETA + (float)ConfigIS.dCoordinationSplit[1] << "  ";
                            }
                        }
                    }
                    else
                        fs << ".  ";
                }
                fs << "\n";
                ReqSeq = ReqSeq + 1;
            }
        }
        fs << ";\n";
    }
    // if we consider adaptive control and the number of vehicles in the trajectory is greater than
    // zero
    if ((m_codeUsage == ADAPTIVE_PRIORITY) && (m_currentTotalVeh > 0))
    {
        // if (currentTotalVeh>31) // !!!! increase solution time!!!!
        // currentTotalVeh=31;
        float t = 0.0;
        fs << "param s:=" << m_qDischargingSpd << "; \n";
        fs << "param qSp:=" << m_queuingSpd << "; \n";
        fs << "param Ar:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            t = roundf(m_modInput[i].vehDistToStpBar * 10) / 10;
            fs << i + 1 << " " << t << " ";
        }
        fs << "; \n";
        fs << "param Ve:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            t = roundf(m_modInput[i].vehSpd * 10) / 10;
            fs << i + 1 << " " << t << " ";
        }
        fs << "; \n";
        fs << "param Ln:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            fs << i + 1 << " " << m_modInput[i].laneNumber << " ";
        }
        fs << "; \n";
        fs << "param Ratio:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            t = roundf(m_modInput[i].ratio * 10) / 10;
            fs << i + 1 << " " << t << " ";
        }
        fs << "; \n";

        fs << "param Ph:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            fs << i + 1 << " " << m_modInput[i].vehPhase << " ";
        }
        fs << "; \n";

        fs << "param Tq:=";
        for (int i = 0; i < m_currentTotalVeh; i++)
        {
            t = roundf(m_modInput[i].stopTime * 10) / 10;
            fs << i + 1 << " " << t << " ";
        }
        fs << "; \n";

        fs << "param LaPhSt:=";
        for (int i = 0; i < m_totalLanes; i++)
        {
            fs << i + 1 << " " << m_laneSignalState[i] << " ";
        }
        fs << "; \n";

        fs << "param L0:=";
        for (int i = 0; i < m_totalLanes; i++)
        {
            t = roundf(m_qSizeOfEachLane[i] * 10) / 10;
            fs << i + 1 << " " << t << " ";
        }
        fs << "; \n";
    }

    fs << "end;";
    fs.close();
}

void MmitssPrioritySolver::packOPT(char* buffer)
{
    auto& cp = m_criticalPoints;
    int temp;
    int iNoPlannedPhaseInRing1 = 0;
    int iNoPlannedPhaseInRing2 = 0;
    iNoPlannedPhaseInRing1 = numberOfPlannedPhase(cp[0], 1);
    iNoPlannedPhaseInRing2 = numberOfPlannedPhase(cp[1], 2);
    cout << "iNoPlannedPhaseInRing1 " << iNoPlannedPhaseInRing1 << endl;
    cout << "iNoPlannedPhaseInRing2 " << iNoPlannedPhaseInRing2 << endl;
    int offset = 0;
    char* pByte; // pointer used (by cast)to get at each byte
    unsigned short tempUShort;
    long templong;

    tempUShort = (unsigned short)m_optMsgID;
    pByte = (char*)&tempUShort;
    buffer[offset + 0] = *(pByte + 1);
    buffer[offset + 1] = *(pByte + 0);
    offset = offset + 2;

    tempUShort = (unsigned short)m_optMsgCnt;
    pByte = (char*)&tempUShort;
    buffer[offset + 0] = *(pByte + 1);
    buffer[offset + 1] = *(pByte + 0);
    offset = offset + 2;

    buffer[offset + 0] = 0X01;
    offset++;

    tempUShort = (unsigned short)iNoPlannedPhaseInRing1;
    pByte = (char*)&tempUShort;
    buffer[offset + 0] = *(pByte + 1);
    buffer[offset + 1] = *(pByte + 0);
    offset = offset + 2;

    for (int ii = 0; ii < 5; ii++)
    {
        templong = (long)cp[0][0][ii] * 1000000;
        pByte = (char*)&templong;
        buffer[offset + 0] = *(pByte + 3);
        buffer[offset + 1] = *(pByte + 2);
        buffer[offset + 2] = *(pByte + 1);
        buffer[offset + 3] = *(pByte + 0);
        offset = offset + 4;

        templong = (long)cp[0][1][ii] * 1000000;
        pByte = (char*)&templong;
        buffer[offset + 0] = *(pByte + 3);
        buffer[offset + 1] = *(pByte + 2);
        buffer[offset + 2] = *(pByte + 1);
        buffer[offset + 3] = *(pByte + 0);
        offset = offset + 4;

        temp = (int)(cp[0][2][ii] + 1); // by adding 1 to cp[0][2][ii]  ,  the sent phase number is
                                        // 1 2 3 or 4
        tempUShort = (unsigned short)temp;
        pByte = (char*)&tempUShort;
        buffer[offset + 0] = *(pByte + 1);
        buffer[offset + 1] = *(pByte + 0);
        offset = offset + 2;
    }
    buffer[offset + 0] = 0X02;
    offset++;

    tempUShort = (unsigned short)iNoPlannedPhaseInRing2;
    pByte = (char*)&tempUShort;
    buffer[offset + 0] = *(pByte + 1);
    buffer[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    for (int i = 0; i < 5; i++)
    {
        templong = (long)cp[1][0][i] * 1000000;
        pByte = (char*)&templong;
        buffer[offset + 0] = *(pByte + 3);
        buffer[offset + 1] = *(pByte + 2);
        buffer[offset + 2] = *(pByte + 1);
        buffer[offset + 3] = *(pByte + 0);
        offset = offset + 4;

        templong = (long)cp[1][1][i] * 1000000;
        pByte = (char*)&templong;
        buffer[offset + 0] = *(pByte + 3);
        buffer[offset + 1] = *(pByte + 2);
        buffer[offset + 2] = *(pByte + 1);
        buffer[offset + 3] = *(pByte + 0);
        offset = offset + 4;

        temp = (int)(cp[1][2][i] + 5); // by adding 1 to cp[0][2][ii]  ,  the sent phase number is 5
                                       // 6 7 or 8
        tempUShort = (unsigned short)temp;
        pByte = (char*)&tempUShort;
        buffer[offset + 0] = *(pByte + 1);
        buffer[offset + 1] = *(pByte + 0);
        offset = offset + 2;
    }
}

// With EV case: will change the MaxGrn to a very large number for EV requested phases
void MmitssPrioritySolver::LinkList2DatFileForEV(const RsuConfig& cfg, int ChangeMaxGrn)
{
    //-- Convert request linkedlist to the NewModel.dat file for GLPK solver.
    //-- Init1 and Init2 are the initial time for the two rings while current phases are in R or Y.
    //-- MinGrn1 and MinGrn2 are the elapsed time when current phase is in Green
    //-- Add new dumy argument: ChangeMaxGrn: default value=0, no need to change the green time;
    //---otherwise, will change the max green time to a big number MaxGrnTime
    const auto& configIS = cfg.getConfig();

    ofstream fs;

    fs.open(SieMmitss::getPriorityDataFilePath(), ios::out);

    int R1No = configIS.Ring1No;
    int R2No = configIS.Ring2No;

    fs << "data;\n";
    fs << "param current:=0;\n";
    fs << "param SP1:=" << m_initPhase[0] << ";\n"; // This is the real phase [1-4]
    fs << "param SP2:=" << m_initPhase[1] << ";\n"; // This is the real phase [5-8]

    for (int i = 0; i < 2; i++)
    {
        if (m_initTime[i] < 0)
        {
            m_initTime[i] = 0;
        }
    }

    fs << "param init1:=" << m_initTime[0] << ";\n";
    fs << "param init2:=" << m_initTime[1] << ";\n";
    fs << "param Grn1 :=" << m_grnElapse[0] << ";\n";
    fs << "param Grn2 :=" << m_grnElapse[1] << ";\n";

    int MP[2];             //=ConfigIS.MissPhase[i];// Missing phase
    int RlP[2] = {-1, -1}; //=ConfigIS.MP_Relate[i];// Missing Phase related

    MP[0] = configIS.MissPhase[0];
    if (MP[0] >= 0)
    {
        if (FindIndexArray(configIS.Phase_Seq_R1, configIS.Ring1No, MP[0]) < 0)
            RlP[0] = configIS.MP_Relate[0];
    }

    MP[1] = configIS.MissPhase[1];
    if (MP[1] >= 0)
    {
        if (FindIndexArray(configIS.Phase_Seq_R2, configIS.Ring2No, MP[1] - 4) < 0)
            RlP[1] = configIS.MP_Relate[1];
    }

    //=================Add the information for Yellow, Red, GrnMin======//
    fs << "param y       \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = configIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << configIS.Yellow[k];
    }

    for (int i = 0; i < R2No; i++)
    {
        int k = configIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << configIS.Yellow[k + 4];
    }
    //========================================================//
    for (int i = 0; i < 2; i++)
    {
        if (RlP[i] >= 0)
        {
            int MP1 = configIS.MissPhase[i];  // Missing phase
            int RlP1 = configIS.MP_Relate[i]; // Missing Phase related
            fs << "\t" << (MP1 + 1) << "  " << configIS.Yellow[RlP1];
        }
    }
    //========================================================//
    fs << ";\n";

    fs << "param red       \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = configIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << configIS.Red[k];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = configIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << configIS.Red[k + 4];
    }
    //========================================================//
    for (int i = 0; i < 2; i++)
    {
        if (RlP[i] >= 0)
        {
            int MP1 = configIS.MissPhase[i];  // Missing phase
            int RlP1 = configIS.MP_Relate[i]; // Missing Phase related
            fs << "\t" << (MP1 + 1) << "  " << configIS.Red[RlP1];
        }
    }
    //========================================================//
    fs << ";\n";

    fs << "param gmin      \t:=";
    for (int i = 0; i < R1No; i++)
    {
        int k = configIS.Phase_Seq_R1[i];
        fs << "\t" << (k + 1) << "  " << configIS.Gmin[k];
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = configIS.Phase_Seq_R2[i];
        fs << "\t" << (k + 5) << "  " << configIS.Gmin[k + 4];
    }
    //========================================================//
    for (int i = 0; i < 2; i++)
    {
        if (RlP[i] >= 0)
        {
            int MP1 = configIS.MissPhase[i];  // Missing phase
            int RlP1 = configIS.MP_Relate[i]; // Missing Phase related
            fs << "\t" << (MP1 + 1) << "  " << configIS.Gmin[RlP1];
        }
    }
    //========================================================//
    fs << ";\n";

    //================End of the information for Yellow, Red, GrnMin======//

    // NEED to change the green max of the requested phases to a large number
    fs << "param gmax      \t:=";

    for (int i = 0; i < R1No; i++)
    {
        int k = configIS.Phase_Seq_R1[i];

        double temp_grnMax = configIS.Gmax[k]; // RequestPhaseInList
        if (temp_grnMax > 0)                   // Phase
        {
            if (ChangeMaxGrn != 0) // EV: not only change phase
            {
                fs << "\t" << (k + 1) << "  " << MaxGrnTime;
            }
            else
            {
                if (m_reqListCombined.FindRequestPhaseInList((k + 1)))
                {
                    fs << "\t" << (k + 1) << "  "
                       << int(configIS.Gmax[k] * (1 + m_maxGrnExtPortion));
                }
                else
                {
                    fs << "\t" << (k + 1) << "  " << configIS.Gmax[k];
                }
            }
        }
    }
    for (int i = 0; i < R2No; i++)
    {
        int k = configIS.Phase_Seq_R2[i]; // k belongs to {0~3}
        double temp_grnMax = configIS.Gmax[k + 4];

        if (temp_grnMax > 0) // Phase is used.
        {
            if (ChangeMaxGrn != 0) // EV: not only change phase
            {
                fs << "\t" << (k + 5) << "  " << MaxGrnTime;
            }
            else
            {
                if (m_reqListCombined.FindRequestPhaseInList((k + 5)))
                {
                    fs << "\t" << (k + 5) << "  "
                       << int(configIS.Gmax[k + 4] * (1 + m_maxGrnExtPortion));
                }
                else
                {
                    fs << "\t" << (k + 5) << "  " << configIS.Gmax[k + 4];
                }
            }
        }
    }
    //====================MaxGreen=240======================//
    for (int i = 0; i < 2; i++)
    {
        if (RlP[i] >= 0)
        {
            int MP1 = configIS.MissPhase[i]; // Missing phase
            // int RlP1=ConfigIS.MP_Relate[i];// Missing Phase related
            fs << "\t" << (MP1 + 1) << "  " << MaxGrnTime;
        }
    }
    //========================================================//

    fs << ";\n\n";

    int NumberofRequests = 0;
    fs << "param priorityType:= ";
    if (m_reqListCombined.empty() == false)
    {
        fs << m_reqListCombined.allVehClassAsString(NumberofRequests);
        while (NumberofRequests < 10)
        {
            NumberofRequests++;
            fs << NumberofRequests;
            fs << " ";
            fs << 0;
            fs << " ";
        }
        fs << " ;  \n";
    }
    fs << "param PrioWeigth:=  1 1 2 0 3 0 4 0 5 0 6 0 7 0 8 0 9 0 10 0 ; \n";
    RequestList Req_List_New;
    VEH_PRIORITY priority;
    if (m_reqListCombined.empty() == false)
    {
        //------Here EV=1, TRANSIT=2. EV>TRANSIT-----//
        priority =
            m_reqListCombined.FindListHighestPriority(); //*** Req list should be not empty.***//
        // cout << "priority" << priority << endl;
        m_reqListCombined.FindReqWithSamePriority(priority, Req_List_New);
    }

    fs << "param Rl (tr): 1 2 3 4 5 6 7 8:=\n";
    unsigned int ReqSeq = 1;
    for (auto& req : Req_List_New)
    {
        fs << ReqSeq << "  ";
        for (int j = 1; j <= 8; j++)
        {
            if (req.Phase == j)
                fs << max(req.ETA - ROBUSTTIME_LOW_EV, float(1.0)) << "  ";
            else
                fs << ".  ";
        }
        if (ReqSeq < Req_List_New.size())
            fs << "\n";
        ReqSeq = ReqSeq + 1;
    }

    fs << ";\n";

    fs << "param Ru (tr): 1 2 3 4 5 6 7 8:=\n";
    ReqSeq = 1;
    for (auto& req : Req_List_New)
    {
        fs << ReqSeq << "  ";
        for (int j = 1; j <= 8; j++)
        {
            if (req.Phase == j)
                if (req.MinGreen > 0)
                {
                    fs << ROBUSTTIME_UP_EV + req.MinGreen << "  ";
                }
                else
                    fs << req.ETA + ROBUSTTIME_UP_EV << "  ";
            else
                fs << ".  ";
        }
        if (ReqSeq < Req_List_New.size())
            fs << "\n";
        ReqSeq = ReqSeq + 1;
    }
    fs << ";\n";

    fs << "end;";
    fs.close();
}

void MmitssPrioritySolver::calculateQ()
{
    ConnectedVehicle temp_CV;
    int templane = 0;
    int iMaxVehInQ = 0;
    double dSizeOfQ = 0.0;
    double dEndofQ = 0.0;
    double dBeginingofQ = 10000.0;
    int templ = 0;
    ConnectedVehicleList vehlistPhase;
    ConnectedVehicleList vehlistLane;

    //    double dRecTime=GetSeconds();
    for (int e; e < 8; e++)
    {
        for (int ee; ee < 6; ee++)
        {
            m_noVehInQ[e][ee] = 0;
            m_qSize[e][ee] = 0;
        }
    }
    for (int p = 0; p < m_totalLanes; p++)
        m_qSizeOfEachLane[p] = 0;

    for (int e = 0; e < 8; e++)
    {
        // Assign vehicles to the list of each lane of each phase. First of all, vehicles clusters
        // by their phases and then cluster by each lane in the phase.
        vehlistPhase.clear();
        for (const auto& veh : m_trackedVeh)
        {
            if (veh.req_phase == e + 1)
            {
                vehlistPhase.emplace_back(veh);
            }
        }

        // TEMPORARY!!!!!!!!!!!!!!!!!!!!!!!!!! // !!! check the total lines if we want to use the
        // code for other intersections
        templ = 0;
        if ((e + 1) == 1)
            templ = m_laneNo[5];
        if ((e + 1) == 3)
            templ = m_laneNo[7];
        if ((e + 1) == 5)
            templ = m_laneNo[1];
        if ((e + 1) == 7)
            templ = m_laneNo[3];
        // if (((e+1)==8)||((e+1)==2)||((e+1)==6))
        // templ=1;

        for (int ee = templ; ee < m_laneNo[e] + templ; ee++)
        {
            vehlistLane.clear();
            for (const auto& veh : vehlistPhase)
            {
                if (veh.lane == ee + 1)
                {
                    vehlistLane.emplace_back(veh);
                }
            }
            templane = 0;
            iMaxVehInQ = 0;
            dSizeOfQ = 0.0;
            dEndofQ = 0.0;
            dBeginingofQ = 10000.0;

            for (const auto& veh : vehlistLane)
            {
                if (veh.Speed < 1)
                {
                    iMaxVehInQ++;
                    dEndofQ = max(dEndofQ, veh.stopBarDistance);
                    dBeginingofQ = min(dBeginingofQ, veh.stopBarDistance);
                    templane = veh.lane;
                }
            }
            if (iMaxVehInQ == 0) // no vehicle is in q
                dSizeOfQ = 0.0;
            // else if (dEndofQ==dBeginingofQ) // only one vehicle in Q
            // dSizeOfQ=dEndofQ;
            else
            {
                if ((dBeginingofQ == dEndofQ) &&
                    (dEndofQ <
                        3)) // only one car in the queue and the car is the first car in the queue
                    dSizeOfQ = 3.5;
                else if (dBeginingofQ == dEndofQ) // only one connected car in the queue and there
                                                  // is a queue of non-connected cars infront of it
                    dSizeOfQ = dEndofQ + 3.5;
                else
                    dSizeOfQ = dEndofQ - dBeginingofQ;
            }

            m_noVehInQ[e][ee] = iMaxVehInQ;
            m_qSize[e][ee] = dSizeOfQ;
            //        cout << "Qsize phase " << e+1 << "lane " << ee+1 <<   "  size " <<
            //        QSize[e][ee] << endl;
            if (templane >
                1) // !!! check the total lines if we want to use the code for other intersections
            {
                if ((e + 1 == 4) || (e + 1 == 7))
                    m_qSizeOfEachLane[templane - 1] = dSizeOfQ;
                else if ((e + 1 == 1) || (e + 1 == 6))
                    m_qSizeOfEachLane[templane + 3 - 1] = dSizeOfQ;
                else if ((e + 1 == 3) || (e + 1 == 8))
                    m_qSizeOfEachLane[templane + 3 + 5 - 1] = dSizeOfQ;
                else if ((e + 1 == 2) || (e + 1 == 5))
                    m_qSizeOfEachLane[templane + 3 + 5 + 4 - 1] = dSizeOfQ;
            }

            // cout << "Qsize" << QSize[e][ee] << endl;

            // cout <<  "TEMPLATE LANE" << templane << endl;
            // if ((templane>0) && (e+1==PhaseOfEachApproach[(int)(tempapproach-1)/2][0]))
            //{
            // for (int cn=(int)(tempapproach-1)/2 ;cn>0;cn--)
            //{
            // if (PhaseOfEachApproach[cn][0]!=PhaseOfEachApproach[cn][1])
            //{
            // templane=LaneNo[PhaseOfEachApproach[cn][0]]+LaneNo[PhaseOfEachApproach[cn][1]]+templane;
            //}else
            // templane=LaneNo[PhaseOfEachApproach[cn][0]]+templane;
            //}
            // dQsizeOfEachLane[templane]=dSizeOfQ;
            //}
            // cout << " LANE NUBMER"  << templane <<  " SIZE "  << dQsizeOfEachLane[templane] <<
            // endl;
        }
    }
    // for (int g=0;g<TotalLanes;g++) // !!! check the total lines if we want to use the code for
    // other intersections
    //    cout << "line" << g+1 << " size " << dQsizeOfEachLane[g] << endl;
}

// Without EV case: will extend the MaxGrn to some portion for Transit requested phases
void MmitssPrioritySolver::deductSolvingTimeFromCPs(double tt2, double tt1)
{
    for (int k = 0; k < 2; k++)
    {
        for (int ii = 0; ii < 5; ii++)
        {
            m_criticalPoints[k][0][ii] = max(0.0, m_criticalPoints[k][0][ii] - (tt2 - tt1));
            m_criticalPoints[k][1][ii] = max(0.0, m_criticalPoints[k][1][ii] - (tt2 - tt1));
        }
    }
}

void MmitssPrioritySolver::Pack_Event_List(
    char* tmp_event_data, int& size) // This function is written by YF
{
    int offset = 0;
    char* pByte; // pointer used (by cast)to get at each byte
                 // of the shorts, longs, and blobs

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
    pByte = (char*)&tempUShort;
    tmp_event_data[offset + 0] = *(pByte + 1);
    tmp_event_data[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    // Events in R1
    for (const auto& event : m_eventListR1)
    {
        // Time
        tempLong = (long)(event.time * DEG2ASNunits);
        pByte = (char*)&tempLong;
        tmp_event_data[offset + 0] = *(pByte + 3);
        tmp_event_data[offset + 1] = *(pByte + 2);
        tmp_event_data[offset + 2] = *(pByte + 1);
        tmp_event_data[offset + 3] = *(pByte + 0);
        offset = offset + 4;
        // phase
        tempUShort = (unsigned short)event.phase;
        pByte = (char*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        // action
        tempUShort = (unsigned short)event.action;
        pByte = (char*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
    }

    // No. events in R2
    int No_Event_R2 = m_eventListR2.size();
    tempUShort = (unsigned short)No_Event_R2;
    pByte = (char*)&tempUShort;
    tmp_event_data[offset + 0] = *(pByte + 1);
    tmp_event_data[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    // Events in R2
    for (const auto& event : m_eventListR2)
    {
        // Time
        tempLong = (long)(event.time * DEG2ASNunits);
        pByte = (char*)&tempLong;
        tmp_event_data[offset + 0] = *(pByte + 3);
        tmp_event_data[offset + 1] = *(pByte + 2);
        tmp_event_data[offset + 2] = *(pByte + 1);
        tmp_event_data[offset + 3] = *(pByte + 0);
        offset = offset + 4;
        // phase
        tempUShort = (unsigned short)event.phase;
        pByte = (char*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
        // action
        tempUShort = (unsigned short)event.action;
        pByte = (char*)&tempUShort;
        tmp_event_data[offset + 0] = *(pByte + 1);
        tmp_event_data[offset + 1] = *(pByte + 0);
        offset = offset + 2;
    }
    size = offset;
}

void MmitssPrioritySolver::UnpackTrajData1(const char* ablob)
{
    int No_Veh;

    int offset;
    offset = 0;
    long tempLong;
    unsigned char byteA; // force to unsigned this time,
    unsigned char byteB; // we do not want a bunch of sign extension
    unsigned char byteC; // math mucking up our combine logic
    unsigned char byteD;

    // Header
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    offset = offset + 2;

    // cout << temp << endl;

    // id
    offset = offset + 1; // move past to next item
    // cout << temp << endl;

    // Do vehicle number
    byteA = ablob[offset + 0];
    byteB = ablob[offset + 1];
    No_Veh = (byteA << 8) + byteB; // in fact unsigned
    offset = offset + 2;

    // cout << No_Veh << endl;

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

        // cout << "E_Offset[" << j << "] is:" << TempVeh.E_Offset[j] << endl;
        // cout << "Done with one vehicle" << endl;

        // Here reflects the penetration rate!!!!!!!!!!!!!!!!!!!!!!!!!!! Add only half of the
        // vehicles
        //    if (penetration>0.99)  //%100 penetration rate
        //    {
        m_trackedVeh.emplace_back(TempVeh);
        //    }
        //    if (penetration>0.74 && penetration<0.76) //75% penetration rate
        //    {
        //        if (TempVeh.TempID%4==3 || TempVeh.TempID%4==1 ||TempVeh.TempID%4==2)
        //            m_trackedVeh.InsertRear(TempVeh);
        //    }
        //    if (penetration>0.49 && penetration<0.51)    //50% penetration rate
        //    {
        //        if (TempVeh.TempID%2==1)
        //            m_trackedVeh.InsertRear(TempVeh);
        //    }
        //    if (penetration>0.24 && penetration<0.26)    //25% penetration rate
        //    {
        //        if (TempVeh.TempID%4==1)
        //            m_trackedVeh.InsertRear(TempVeh);
        //    }
    }
}

void MmitssPrioritySolver::printCriticalPoints()
{
    ITSAPP_LOG(" RING 1 CRITICAL POINTS ");
    for (int i = 0; i < 5; i++)
    {
        ITSAPP_LOG("CP left               %d %lf", i, m_criticalPoints[0][0][i]);
        ITSAPP_LOG("CP right              %d %lf", i, m_criticalPoints[0][1][i]);
        ITSAPP_LOG("phase                 %d %lf", i, m_criticalPoints[0][2][i] + 1);
    }
    ITSAPP_LOG(" RING 2 CRITICAL POINTS ");
    for (int i = 0; i < 5; i++)
    {
        ITSAPP_LOG("CP left               %d %lf", i, m_criticalPoints[1][0][i]);
        ITSAPP_LOG("CP right              %d %lf", i, m_criticalPoints[1][1][i]);
        ITSAPP_LOG("phase                 %d %lf", i, m_criticalPoints[1][2][i] + 5);
    }
    // sprintf(temp_log,".......... The new OPT set sent  At time: %.2f.......... \n",
    // GetSeconds()); outputlog(temp_log); cout <<  temp_log <<  endl;
}

bool MmitssPrioritySolver::get_lane_no()
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

    for (int e = 0; e < 8; e++)
    {
        m_totalLanes += m_laneNo[e];
        ITSAPP_LOG("Phase %d number of lane %d", e + 1, m_laneNo[e]);
    }

    fs.close();
    return true;
}

bool MmitssPrioritySolver::get_lane_phase_mapping()
{
    fstream fs(SieMmitss::getLanePhaseMapFilePath(), fstream::in);

    string temp_string;
    getline(fs, temp_string); // First line is comment
    getline(fs, temp_string); // Second line contains information
    if (temp_string.size() == 0)
    {
        ITSAPP_WRN("Reading Lane_Phase_Mapping_File problem");
        fs.close();
        return false;
    }
    sscanf(temp_string.c_str(), "%d %d %d %d %d %d %d %d", &m_phaseOfEachApproach[0][0],
        &m_phaseOfEachApproach[0][1], &m_phaseOfEachApproach[1][0], &m_phaseOfEachApproach[1][1],
        &m_phaseOfEachApproach[2][0], &m_phaseOfEachApproach[2][1], &m_phaseOfEachApproach[3][0],
        &m_phaseOfEachApproach[3][1]);

    int lane = 0;
    for (int j = 0; j < 4; j++)
    {
        // populating phase of each lane data structre
        if (m_phaseOfEachApproach[j][0] != m_phaseOfEachApproach[j][1])
        { // in the case that left turn and through are not the same phase
            if (m_phaseOfEachApproach[j][0] > 0)
                for (int i = 0; i < m_laneNo[m_phaseOfEachApproach[j][0] - 1]; i++)
                {
                    m_phaseOfEachLane[lane] = m_phaseOfEachApproach[j][0];
                    lane++;
                }
            if (m_phaseOfEachApproach[j][1] > 0)
                for (int i = 0; i < m_laneNo[m_phaseOfEachApproach[j][1] - 1]; i++)
                {
                    m_phaseOfEachLane[lane] = m_phaseOfEachApproach[j][1];
                    lane++;
                }
        }
        else
        {
            if (m_phaseOfEachApproach[j][0] > 0)
                for (int i = 0; i < m_laneNo[m_phaseOfEachApproach[j][0] - 1]; i++)
                {
                    m_phaseOfEachLane[lane] = m_phaseOfEachApproach[j][0];
                    lane++;
                }
        }
    }
    for (int j = 0; j < 4; j++)
    {
        lane = 0;
        if (m_phaseOfEachApproach[j][0] > 0)
        {
            // populating LaneNo2 data structure to be used in calculateQ function
            if (m_phaseOfEachApproach[j][0] != m_phaseOfEachApproach[j][1])
            { // in the case that left turn and through are not the same phase
                // LaneNo2 will record the total number of lanes per approach
                m_laneNo2[m_phaseOfEachApproach[j][0] - 1] =
                    m_laneNo[m_phaseOfEachApproach[j][0] - 1];
                if (m_phaseOfEachApproach[j][1] > 0)
                {
                    m_laneNo2[m_phaseOfEachApproach[j][1] - 1] =
                        m_laneNo[m_phaseOfEachApproach[j][0] - 1] +
                        m_laneNo[m_phaseOfEachApproach[j][1] - 1];
                }
            }
            else
            {
                // LaneNo2 will record the total number of lanes per approach
                m_laneNo2[m_phaseOfEachApproach[j][0] - 1] =
                    m_laneNo[m_phaseOfEachApproach[j][0] - 1];
            }
        }
    }
    for (int jj = 0; jj < m_totalLanes; jj++)
        ITSAPP_TRACE("lane %d phase %d", jj + 1, m_phaseOfEachLane[jj]);
    for (int jj = 0; jj < 8; jj++)
        ITSAPP_TRACE("phase %d lane num per approach %d", jj + 1, m_laneNo2[jj]);
    fs.close();
    return true;
}

bool MmitssPrioritySolver::setupConnection()
{
    if ((m_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        ITSAPP_ERROR("sockfd error");
        return false;
    }
    int broadcast = 1;
    if ((setsockopt(m_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))) == -1)
    {
        ITSAPP_ERROR("setsockopt - SO_SOCKET ");
        return false;
    }

    m_recvAddr.sin_family = AF_INET;
    m_recvAddr.sin_port = htons(SieMmitss::getTrafficControllerInterfacePort());
    m_recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // INADDR_BROADCAST;
    memset(m_recvAddr.sin_zero, '\0', sizeof(m_recvAddr.sin_zero));
    return true;
}

int numberOfPlannedPhase(double cp[3][5], int rng)
{
    if (rng == 1)
    {
        int NoPhaseRng1 = 0;
        for (int i = 0; i < 4; i++)
        {
            if ((cp[0][i] > 0 && cp[0][i + 1] == 0) || (cp[1][i] > 0 && cp[1][i + 1] == 0))
            {
                NoPhaseRng1 = i + 1;
            }
        }
        if (cp[1][4] > 0)
            NoPhaseRng1 = 5;

        //~ else if (NoPhaseRng1==-1 && cp[][1]==0 && cp[1][1]==0)
        //~ NoPhaseRng1=1;
        return NoPhaseRng1;
    }
    else
    {
        int NoPhaseRng2 = 0;
        for (int i = 0; i < 4; i++)
        {
            if ((cp[0][i] > 0 && cp[0][i + 1] == 0) || (cp[1][i] > 0 && cp[1][i + 1] == 0))
            {
                NoPhaseRng2 = i + 1;
            }
        }
        if (cp[1][4] > 0)
            NoPhaseRng2 = 5;
        return NoPhaseRng2;
    }
}

double findLargestCP(double cp[3][5], int rng, int maxNumOfPlannedPhases)
{
    NOTUSED(rng);

    double temp = 0.0;
    for (int i = 0; i < maxNumOfPlannedPhases; i++)
    {
        if (cp[0][i] > temp)
            temp = cp[0][i];
    }
    return temp;
}

int FindRingNo(int phase)
{
    int RingNo;
    switch (phase % 10)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            RingNo = 0;
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            RingNo = 1;
            break;
        default:
            ITSAPP_WRN("******Error: No such phase!");
            RingNo = -1;
    }
    return RingNo;
}

// Return the earliest request in the ring with respect to both time and phase sequence
int MmitssPrioritySolver::findTheEarliestReqInERP(PriorityRequestList PR)
{
    int iTemp = -1;
    int iERP = 99;

    // first find the smallest iPhaseCycle
    auto prioReq = PR.begin();
    while (prioReq != PR.end())
    {
        if (prioReq->iPhaseCycle <= iERP)
        {
            iERP = prioReq->iPhaseCycle;
        }
        prioReq++;
    }

    double dEarliestArr = 999;
    prioReq = PR.begin();
    int i = 0;
    while (prioReq != PR.end())
    {
        if (prioReq->iPhaseCycle == iERP && prioReq->dRl <= dEarliestArr)
        {
            dEarliestArr = prioReq->dRl;
            iTemp = i;
        }
        prioReq++;
        i++;
    }

    return iTemp;
}

} // namespace Mmitss
} // namespace WaveApp
