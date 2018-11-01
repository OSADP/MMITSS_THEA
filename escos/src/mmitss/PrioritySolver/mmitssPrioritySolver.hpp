#pragma once

#include <siekdbus.hpp>
#include <stddef.h>
#include <array>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

#include "udpReceiver.hpp"
#include "ConnectedVehicle.h"
#include "Schedule.h"
#include "Config.h"
#include "Mib.h"
#include "ReqEntry.h"
#include "ReqEntryListHandle.h"
#include "Signal.h"
#include "PriorityRequest.h"

namespace WaveApp
{
namespace Mmitss
{

class MmitssPrioritySolverApp;

class MmitssPrioritySolver
{
  public:
    using PriorityRequestList = std::vector<PriorityRequest>;
    using ScheduleList = std::vector<Schedule>;
    using ConnectedVehicleList = std::vector<ConnectedVehicle>;

    MmitssPrioritySolver(MmitssPrioritySolverApp& parent);
    virtual ~MmitssPrioritySolver() = default;
    MmitssPrioritySolver(const MmitssPrioritySolver&) = delete;
    MmitssPrioritySolver& operator=(const MmitssPrioritySolver&) = delete;

    bool onStartup();
    void solve(std::string&& reqListJson);
    void onShutdown();
    virtual void onExit();
    void tick();

    bool mmitssInit();

  private:
    // This code can be used only for priority eligible vehicles
    // OR can be integrated into COP OR can be used in extended
    // formulation to consider regular vehicle as well as priority
    // vehicles in the mathematical formulation
    enum CodeUsage : int
    {
        // Code usage is for Prioirty Alg + COP
        COP_AND_PRIORITY = 0,
        // Code usage is for priority alg + Actuation
        PRIORITY = 1,
        // Code usage is for integrated priority alg and Adaptive Control
        ADAPTIVE_PRIORITY = 2
    };

    enum Congestion : int
    {
        NORMAL_TRAFFIC = 0,
        CONGESTED = 1
    };

    const uint32_t DEFAULT_TIMER_DELAY_MS = 1000;

    struct ModInputElement
    {
        double vehDistToStpBar;
        double vehSpd;
        int vehPhase;
        int laneNumber;  // lane number of each car
        double stopTime; // The amount of time each vehicle has stopped
        double ratio;    // one of the inputs of mathematical model
        double indic;    // one of the inputs of mathematical model
    };

    // at most 130 vehicles is considered in the adaptive control optimization model
    using ModInput = std::array<ModInputElement, 130>;

    /**
     * For receiving JSON SRM for testing
     */
    void onUdpReceive(const void* data, size_t len);

    bool loadMmitssConfig();
    void onTimer(siekdbus::timerid_t id);
    bool updatePhaseAndElapseTime();
    void requestTrajectory();

    // generate .mod file for glpk optimizer ,
    //------- If there is no EV, using ConfigIS; if there is EV, using ConfigIS_EV. --------//
    bool GenerateMod(const std::string& Filename, const std::string& OutFilename, int haveEVinList);

    void extractOptModInputFromTraj();
    void readOptPlanFromFile();
    void readOptPlanFromFileForEV(const RsuConfig& configEV);
    void packOPT(char* buffer);

    void calculateQ();

    void creatCPexactlyAsOptSol(double* EndOfPhaseRing1,
        const std::vector<int>& phaseSeqRing1,
        int t1,
        double* EndOfPhaseRing2,
        const std::vector<int>& phaseSeqRing2,
        int t2);
    void matchTheBarrierInCPs_EV();
    void matchTheBarrierInCPs();
    void deductSolvingTimeFromCPs(double tt2, double tt1);

    VEH_PRIORITY getType(PriorityRequestList PrioReqListOfRing, int currentPosition);
    void creatFeasibleRegionRing(int ringNo,
        PriorityRequestList PrioReqListOfRing,
        int currentPosition,
        double* EndOfPhase,
        const std::vector<int>& phaseSeq,
        int t,
        double initialGreen,
        double ElapsedGreen,
        int isThereReqInOtherRing);
    void creatFeasibleRegionRing_EV(double CP[3][5],
        PriorityRequestList PrioReqListOfRing,
        int currentPosition,
        double* EndOfPhase,
        int* phaseSeq,
        int t);

    void Construct_eventlist();
    void Construct_eventlist_EV();

    void LinkList2DatFile(const RsuConfig& cfg);
    void LinkList2DatFileForEV(const RsuConfig& cfg, int ChangeMaxGrn);

    void GLPKSolver();
    int GLPKSolutionValidation();

    void Pack_Event_List(char* tmp_event_data, int& size);
    void UnpackTrajData1(const char* ablob);

    void printCriticalPoints();

    bool get_lane_no();
    bool get_lane_phase_mapping();
    bool setupConnection();
    int findTheEarliestReqInERP(PriorityRequestList PR);

    MmitssPrioritySolverApp& m_parent;

    uint32_t m_timerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_timerId{0};
    siekdbus::TimerConn m_timerConn;

    bool m_started{false};

    std::future<bool> m_delayedInit;
    UdpReceiver m_udpReceiver;

    std::string m_testSystemAddr;

    // Former command line arguments:
    CodeUsage m_codeUsage{PRIORITY};
    Congestion m_congestion{NORMAL_TRAFFIC};

    // penetration rate
    // TODO: This needs to be adapted for field use!
    float m_penetrationRate{1.0};

    int m_currentTotalVeh{0};

    // Req list for requests in original requests_combined.txt
    ReqEntryListHandle m_reqListCombined;

    int m_addPhantom{0};

    bool m_haveEVInList{false};
    bool m_haveCoordInList{false};
    bool m_haveTruckInList{false};
    bool m_haveTransitInList{false};
    bool m_havePedCall{false};

    // Extended the MaxGrn of requested phases to this portion.
    // 1ant: keep the number with no 0 before .  ,
    // we need this format when we build the glpk mod file.
    // GenerateMod function
    double m_maxGrnExtPortion{0.15};
    // this array will record the number of lanes per each phase.
    int m_laneNo[8];
    // this array will record the number of lanes per each phase.
    // But if two phases belong to one approach, the LaneNo2 value
    // for the second phase of the approach include the Laneno2 valuse
    // of the first phase as well. As example, if phase 4 has 3 lanes
    // and phase 7 (same approach) has 2 lanes, then LaneNo2[4-1]=3 and
    // LaneNo2[7-1]= 3+2. This array is used in calculateQ function.
    int m_laneNo2[8];

    int m_totalLanes = 0;
    // Use Lane_Phase_File_Name to populate this array
    int m_phaseOfEachApproach[4][2];
    // assume there are 48 approaching lanes at most
    int m_phaseOfEachLane[48];
    // assume there are 48 approaching lanes at most
    double m_qSizeOfEachLane[48];

    ScheduleList m_eventListR1;
    ScheduleList m_eventListR2;

    //--- When in Red or Yellow, init1 & 2 are non-zero, Grn1&Grn2 (GrnElapse) are zero
    //--- When in Green, init1 & 2 are zero, Grn1&Grn2 are non-zero
    double m_initTime[2];
    double m_grnElapse[2];
    int m_initPhase[2];

    // Record current red elapse time information
    // Notes: since COP always runs at the beginning of the green time (2 phases),
    // actually we should calculate the duration of previous red phase for the queue length
    // estimation
    // for the 2 phases just turned red, the red duration time could be 0
    // (or in actually calculation they are very small)
    // for other four phases, just in the middle of red duration
    double m_redStartTime[8];
    // Red elapse time of each phase to be used in EVLS
    double m_redElapseTime[8];

    int m_previousSignalColor[8];
    // time to request Arrival Table in case the code is used for adaptive control
    double m_timeRequestTraj{0.0};
    // this vector get the values of ConfigIS.Gmax an will keep those values
    // unless we need to change the max green time to ConfigIS.Gmax*(1+MaxGrnExtPortion)
    double m_globalGmax[8] = {0.0};
    // is 1 for the lanes that their signal is green, otherwise 0
    bool m_laneSignalState[48];
    // at most 130 vehicles is considered in the adaptive control optimization model
    ModInput m_modInput;
    // This array keeps the crtitical point of the optimal schedule .
    // The first dimention is Ring ( 2 rings [2] ) the send dimension is
    // the left or right side of the Critical zone ( [2] ) the third
    // dimension is the total number of phases we look ahead in the schedule
    // which is 5 phases.
    double m_criticalPoints[2][3][5];
    // The phases that should be omitted when we have EV among the priority vehicles.
    int m_omitPhase[8];

    int m_optMsgCnt{0};

    // TODO: check why this is statically defined and not read from cfg file
    float m_dsrcRange{300};

    // Parameters for trajectory control
    ConnectedVehicleList m_trackedVeh;
    ConnectedVehicleList m_vehListEachPhase;

    // There is a list for all vehicles in each lane of each phase.
    // It is assumed there are at most 8 phases and at most 6 lanes per phase.
    int m_noVehInQ[8][6];
    // length of the queue in each lane (in meter)
    double m_qSize[8][6];
    double m_qDischargingSpd{3.0};
    double m_queuingSpd{2.0};

    int m_optMsgID{55};

    std::string m_rsuId;
    RsuConfig m_cfg;
    RsuConfig m_cfg_EV;
    Phase m_phases;
    MmitssMib m_mib;

    // MPRsolver uses this port to send the optimal solution to ... COP or Controller
    int m_sockfd{-1};
    struct sockaddr_in m_recvAddr;
    uint32_t m_reqCount{0};
    uint32_t m_lastAckCount{0};
};

} // namespace Mmitss
} // namespace WaveApp
