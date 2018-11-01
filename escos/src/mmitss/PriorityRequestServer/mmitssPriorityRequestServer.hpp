#pragma once

#include <siekdbus.hpp>
#include <stddef.h>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "facilityWaveSae.hpp"
#include "udpReceiver.hpp"
#include "Config.h"
#include "Mib.h"
#include "ReqEntryListHandle.h"
#include "Signal.h"
#include "NMAP.h"
#include "IntLanePhase.h"

namespace WaveApp
{
namespace Mmitss
{

static constexpr const char* MMITSS_DB_PRIORITY_REQ_LIST = "/its/us/mmitss/priority/prio_req_list";
static constexpr const int TEMP_ID_LEN = 4;

class ReqEntryListHandle;
class MmitssPriorityRequestServerApp;

enum class GRANT_STATE
{
    Pending,
    Granted,
    Rejected
};

class MmitssPriorityRequestServer
{
  public:
    MmitssPriorityRequestServer(MmitssPriorityRequestServerApp& parent);
    virtual ~MmitssPriorityRequestServer() = default;
    MmitssPriorityRequestServer(const MmitssPriorityRequestServer&) = delete;
    MmitssPriorityRequestServer& operator=(const MmitssPriorityRequestServer&) = delete;

    bool onStartup();
    void onShutdown();
    virtual void onExit();
    void tick();

    bool mmitssInit();
    bool onRxSrm(const saeSignalRequestMessage& msg);
    void RecvSrmFromCommand(const void* data, size_t len) { onUdpReceive(data, len); }
    void updateGrantedStatus(std::string name, std::string granted);
    void enableGrantRequest(bool enable = false);
    std::string reqListAsString() const { return m_reqList.reqListAsString(); };
    std::string getRsuId() const { return m_rsuId; }
    int getIntersectionId() const { return m_lanePhase.iIntersectionID; }
    double getCurrentCycleTime() const { return m_currentCycleTime; }
    double getCoordCycle() const { return m_coordCycle; }
    double getCoordPhaseSplit() const { return m_coordPhaseSplit; }

  private:
    using SsmPtr = std::unique_ptr<saeSignalStatusMessage>;
    using GrantStatusTable = std::unordered_map<string, GRANT_STATE>; // veh-id <-> grant status
    using VehcileIdList = std::vector<string>;
    using NameIdMapping = std::map<string, string>; // name <-> veh-id

    struct SignalRequest
    {
        int seqNum{0};
        int requestId{-1};
        std::string vehId{"0"};
        std::string name{"0"};
        int vehRole{0};
        int intersecId{0};
        int inLaneId{-1};
        int outLaneId{-1};
        int minute{0};
        int second{0};
        int duration{0};
        int type{0};
        int speed{0};
    };

    const uint32_t DEFAULT_TIMER_DELAY_MS = 1000;
    const uint32_t SSM_BROADCAST_INTERVAL_MS = 1000;

    void processRequest(const SignalRequest& req);
    void requestForSchedule(string name);
    void requestForSolveAfterDeleteRequests();
    void requestForSolve();
    void resetPriorityRequestTableInDb();
    void removeFromNameTableById(std::string id);
    VehcileIdList getGrantedVehciles();

    void writeCombinedRequestListToDb(std::string listName);

    bool ReadInCoordinationConfig(const std::string& filename);

    void initSsm();
    void updateSsm();
    void clearSsm();

    /**
     * For receiving JSON SRM for testing
     */
    void onUdpReceive(const void* data, size_t len);
    bool loadMmitssConfig();

    void onTimer(siekdbus::timerid_t id);
    void onSsmTimer(siekdbus::timerid_t id);

    MmitssPriorityRequestServerApp& m_parent;

    uint32_t m_timerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_timerId{0};
    siekdbus::TimerConn m_timerConn;

    SsmPtr m_ssm;
    SsmXmitContext m_ssmContext;
    int m_msgCnt{0};
    std::unordered_map<std::string, SignalRequest> m_requests;

    uint32_t m_ssmTimerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_ssmTimerId{0};
    siekdbus::TimerConn m_ssmTimerConn;

    UdpReceiver m_udpReceiver;
    bool m_started{false};

    std::future<bool> m_delayedInit;

    std::string m_testSystemAddr;

    /*
     * MMITSS members
     */
    ReqEntryListHandle m_reqList;

    IntLanePhase m_lanePhase;

    std::vector<int> m_phaseNotEnabledList;

    /**
     * code usage, if the argument is -c 1 , the program will be used
     * with traffic interface (priority and actuation) .
     * if -c 2 as argument, the program work with ISIG
     */
    int m_codeUsage{1};

    /**
     * 0 = PRS works with NO coordination request
     * 1 = Coordination priority request is ON
     */
    int m_onCoordination{0};

    int m_coordPhase[2];
    // will be obtained from priorityConfiguration file
    double m_coordCycle{100.0};
    double m_currentCycleTime{100.0};
    // will be obtained from priorityConfiguration
    double m_coordPhaseSplit{30.0};
    // coordination parameters
    double m_coordSentTime{-100.0}; // - cycle time !!!
    int m_coordSplitTimeFlag{1};
    int m_coordMsgCount{0};
    double m_currentTime{0.0};
    double m_lastReqFileRevisedTime{0.0};

    std::string m_rsuId;

    MAP m_map;
    RsuConfig m_cfg;
    Phase m_phases;
    MmitssMib m_mib;
    GrantStatusTable m_grantStatusTable;
    NameIdMapping m_nameTable;
    bool m_enableGrantRequest{false};
    bool m_recvFromUdp{false};
};

} // namespace Mmitss
} // namespace WaveApp
