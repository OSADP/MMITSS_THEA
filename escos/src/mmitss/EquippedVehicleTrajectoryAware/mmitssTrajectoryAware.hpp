#pragma once

#include <siekdbus.hpp>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

#include "ConnectedVehicle.h"
#include "geoCoord.h"
#include "BasicVehicle.h"
#include "NMAP.h"
#include "Poco/Net/DatagramSocket.h"

#include "udpReceiver.hpp"

class saeBasicSafetyMessage;

namespace WaveApp
{
namespace Mmitss
{

class MmitssTrajectoryAwareApp;

class MmitssTrajectoryAware
{
  public:
    MmitssTrajectoryAware(MmitssTrajectoryAwareApp& parent);
    virtual ~MmitssTrajectoryAware() = default;
    MmitssTrajectoryAware(const MmitssTrajectoryAware&) = delete;
    MmitssTrajectoryAware& operator=(const MmitssTrajectoryAware&) = delete;

    bool mmitssInit();

    bool onStartup();
    void onShutdown();
    virtual void onExit();
    void tick();

    void onRxBsm(const saeBasicSafetyMessage& msg);

  private:
    const uint32_t DEFAULT_TIMER_DELAY_MS = 500;

    bool loadMmitssConfig();

    void onTimer(siekdbus::timerid_t id);
    void onUdpReceive(const void* data, size_t len);

    void sendTrajectoryData(const Poco::Net::SocketAddress& socketAddr,
        bool toSignalControl = false,
        bool json = false);
    // Pack trajectory data if the request is from performance observer
    void packTrajData(char* tmp_traj_data, int& size);
    // Pack trajectory data if the request is from signal control
    void packTrajData1(char* tmp_traj_data, int& size);

    void FindVehInMap(double Speed,
        double Heading,
        int nFrame,
        double N_Offset1,
        double E_Offset1,
        double N_Offset2,
        double E_Offset2,
        MAP NewMap,
        int pre_app,
        double& Dis_curr,
        double& est_TT,
        int& request_phase,
        int& approach,
        int& lane);

    bool get_lane_phase_mapping();
    void get_appr_phase_mapping();
    bool get_DSRC_Range();

    MmitssTrajectoryAwareApp& m_parent;

    uint32_t m_timerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_timerId{0};
    siekdbus::TimerConn m_timerConn;

    /*
     * MMITSS members
     */
    std::vector<ConnectedVehicle> m_trackedVeh;

    // The number of seconds after which a vehcle is deleted
    // from the list after passing the stop bar or disappearing
    int m_vehDeleteTimeSec{30};

    // Determines how often the trajectory per vehicle is calculated
    // Increase this time to reduce processing load
    double m_processTrajTimeSec{0.5};

    // Note the rectangular coordinate system: local_x is N(+) S(-) side, and local_y is E(+) W(-)
    // side
    // This is relative distance to the ref point!!!!
    // local z can be considered as 0 because no elevation change.
    geoCoord m_refPoint;
    double m_localX{0};
    double m_localY{0};
    double m_localZ{0};
    double m_ecefX{0};
    double m_ecefY{0};
    double m_ecefZ{0};

    float m_speedCoif{1};
    vector<LaneNodes> m_mapNodes;
    MAP m_newMap;
    int m_beginTime{0};
    bool m_started{false};

    std::future<bool> m_delayedInit;

    std::string m_testSystemAddr;

    std::vector<BasicVehicle> m_rxVehicles;
    int m_sendingSocket{-1};
    UdpReceiver m_udpReceiver;

    // For sending the trajectory data
    int m_trajDataSize{0}; // in byte

    int m_sockfd{-1};
    struct sockaddr_in m_sendAddr;
    struct sockaddr_in m_recvAddr;
    // the corresponding phases sequence: approach 1 3 5 7 through left
    int m_phaseMapping[8];
    // Range of the DSRC radio on each leg of the intersection
    float m_dsrcRange[8];
    // 8 approaches, at most 8 lanes each approach, at most 20 lane nodes each lane
    // the value is just the requested phase of this lane node;
    int m_laneNodePhaseMapping[8][8][20];
};

} // namespace Mmitss
} // namespace WaveApp
