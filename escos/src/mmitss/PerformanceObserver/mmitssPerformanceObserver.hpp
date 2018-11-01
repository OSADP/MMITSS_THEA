#pragma once

#include <siekdbus.hpp>
#include <cstdint>
#include <future>
#include <map>
#include <string>
#include <vector>

#include "udpReceiver.hpp"
#include "ConnectedVehicle.h"
#include "Config.h"
#include "Mib.h"
#include "Signal.h"
#include "NMAP.h"
#include "Performance.h"

namespace WaveApp
{
namespace Mmitss
{

class MmitssPerformanceObserverApp;

/**
 *  MmitssTrajectoryAware class
 */
class MmitssPerformanceObserver
{
    // fixed configuration of radar in approach 1 and lane 1..MAX_LANES_RADAR_DETECTED
    static constexpr const int MAX_LANES_RADAR_DETECTED = 3;

  public:
    class UnequippedVeh
    {
      public:
        double time_join;
    };
    class DetectedVeh
    {
      public:
        DetectedVeh() = default;
        DetectedVeh(double _speed, double _distance)
            : speed(_speed)
            , distance(_distance){};
        double speed{0};    // Vehicle current Speed
        double distance{0}; // distance to the detector
    };
    using ConnectedVehicleList = std::vector<ConnectedVehicle>;
    using UnequippedVehicleList = std::vector<UnequippedVeh>;
    using DetectedVehicleList = std::map<int, DetectedVeh>; // current only on approach 1, need to
                                                            // extend when this happens on more
                                                            // approaches
    MmitssPerformanceObserver(MmitssPerformanceObserverApp& parent);
    virtual ~MmitssPerformanceObserver() = default;
    MmitssPerformanceObserver(const MmitssPerformanceObserver&) = delete;
    MmitssPerformanceObserver& operator=(const MmitssPerformanceObserver&) = delete;

    virtual bool onStartup();
    virtual void onShutdown();
    virtual void onExit();
    void tick();
    void addDetectedVeh(int lane, double speed, double distance);
    void addIntoUnequippedVehList(int approach, int lane, double currentTime);
    void updateMaximumEstimation(int approach, int lane, double currentTime);
    bool mmitssInit();
    void setRadarReportedLaneOrder(bool increasingOrder)
    {
        m_radarReportLaneInSameOrder = increasingOrder;
    }

  private:
    /*std::string handle_status(int argc, char* argv[]);
    std::string handle_json(int argc, char* argv[]);
    std::string handle_start(int argc, char* argv[]);
    std::string handle_stop(int argc, char* argv[]);*/
    const uint32_t DEFAULT_TIMER_DELAY_MS = 1000;

    bool loadMmitssConfig();
    void onTimer(siekdbus::timerid_t id);
    void onTrajectoryReceive(const void* data, size_t len);
    void storeQueueData();
    void storeTravelTimeData();
    void resetQueueAndTravelTimeData();

    int getVehicleMovementOnEgress(const ConnectedVehicle& veh);
    int getMovementIdx(const ConnectedVehicle& veh);

    void UnpackTrajData(const uint8_t* ablob);
    bool getDetNumbers();
    bool getLaneMovementMapping();

    MmitssPerformanceObserverApp& m_parent;

    uint32_t m_timerInterval{DEFAULT_TIMER_DELAY_MS};
    siekdbus::timerid_t m_timerId{0};
    siekdbus::TimerConn m_timerConn;

    UdpReceiver m_udpReceiver;

    std::string m_testSystemAddr;

    MAP m_map;

    float m_penetrationRate{1.0};

    /*
     * MMITSS members
     */
    // Market Penetration Rate for each lane
    float m_PR[8][5];

    RsuConfig m_cfg;
    Phase m_phases;
    MmitssMib m_mib;
    bool m_started{false};

    std::future<bool> m_delayedInit;

    ConnectedVehicleList m_trackedVeh;
    // This is the list from BSM_reciever for updating the trajectory
    ConnectedVehicleList m_receivedVehList;

    // This is the list forwarded from NextConnect for updating the trajectory
    DetectedVehicleList m_forwardedVehList;
    bool m_radarReportLaneInSameOrder{true};

    UnequippedVehicleList
        m_unequippedVehList[20]; // A list of unequipped vehicle, useful for detectors data

    Performance m_perf;

    // Distance from the System Detector to the back of the Queue
    std::vector<double> m_disDetQ;
    // Time to join the back of the queue for non-CVs
    std::vector<double> m_timeToJoin;

    int m_previousSignalColor[8];
    double m_redStartTime[8];
    double m_greenStartTime[8];
    int m_arrivalOnGreen[8];
    int m_arrivalOnRed[8];
    // Shows how often we send request for the trajectory data
    int m_frequency{1};
    // Shows how often we calculate the performance measures
    int m_calcFrequency{15};
    // Shows how often we aggregate data
    int m_aggFrequency{300};

    double m_sendTimer{0};
    double m_calcTimer{0};
    double m_aggTimer{0};

    // Detectors
    // Total number of detectors
    int m_totNumDet{0};
    // Indicate the detector numbers
    std::vector<int> m_detNumbers;
    // Array of the distances from system detectors to the associated stopbar
    float m_detectorStopbarDist[20];
    // Array of the phases associated with each lane
    int m_lanePhase[20];
    // Array of the Turning Movement Proportions
    float m_turningProportion[20][3];

    // The corresponding lane used for movement identification
    int m_laneMapping[8];
    // The corresponding movements for TT&D purposes!
    int m_movementMapping[12];
    uint32_t m_reqCount{0};
    uint32_t m_lastAckCount{0};
    int m_sockfd{-1};
    struct sockaddr_in m_recvAddr;

    int m_sockMmitssCtl{-1};
    struct sockaddr_in m_mmitssCtlAddress;
};

} // namespace Mmitss
} // namespace WaveApp
