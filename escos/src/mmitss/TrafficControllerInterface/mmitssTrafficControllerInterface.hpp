#pragma once

#include <siekdbus.hpp>
#include <array>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

#include "Schedule.h"
#include "Config.h"
#include "Mib.h"
#include "Signal.h"

#include "udpReceiver.hpp"

namespace WaveApp
{
namespace Mmitss
{

class MmitssTrafficControllerInterfaceApp;

class MmitssTrafficControllerInterface
{
  public:
    using ScheduleList = std::vector<Schedule>;
    MmitssTrafficControllerInterface(MmitssTrafficControllerInterfaceApp& parent);
    virtual ~MmitssTrafficControllerInterface() = default;
    MmitssTrafficControllerInterface(const MmitssTrafficControllerInterface&) = delete;
    MmitssTrafficControllerInterface& operator=(const MmitssTrafficControllerInterface&) = delete;

    virtual bool onStartup();
    virtual void onShutdown();
    virtual void onExit();
    void tick();

    bool mmitssInit();

  private:
    static constexpr const uint32_t DEFAULT_TIMER_DELAY_MS = 500;

    bool loadMmitssConfig();
    void onTimer(siekdbus::timerid_t id);
    void resetEventTableInDb();
    void storeEventTimeTableInDb();
    void onSignalCtrlScheduleReceive(const void* data, size_t len);

    bool ReadSignalParameters();
    void ModifyCurPhase();
    int CheckConflict();
    void UnpackEventData(const char* ablob);

    MmitssTrafficControllerInterfaceApp& m_parent;

    uint32_t m_timerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_timerId{0};
    siekdbus::TimerConn m_timerConn;

    UdpReceiver m_udpReceiver;

    std::string m_testSystemAddr;

    // MMITSS members

    std::string m_rsuId;
    RsuConfig m_cfg;
    Phase m_phases;
    MmitssMib m_mib;

    bool m_started{false};

    std::future<bool> m_delayedInit;
    // timer to hold the phase;
    int m_holdTimer{0};

    ScheduleList m_eventListR1;
    ScheduleList m_eventListR2;
    std::array<std::array<int, 8>, 6> m_eventTimeTable; // 6 actions and 8 phases

    int m_phaseDisabled[8];
    int m_occupancy[8];
    int m_pedInfo[2];

    int m_currPhase[2];
    int m_phaseNum{0};
    int m_phaseSeq[8];
    // flag to find out whether a new signal timing schedule is received
    int m_newSchedule{0};

    double m_currentTime{0};
    // the current event list beginning time
    double m_beginTime{0};
    std::string m_beginTimeUTC{"N/A"};
    // Hold timer, control the No. hold command send to ASC controller
    double m_holdTimerR1{0.0};
    double m_holdTimerR2{0.0};

    int m_sockfd{-1};
    struct sockaddr_in m_sendAddr;
};

} // namespace Mmitss
} // namespace WaveApp
