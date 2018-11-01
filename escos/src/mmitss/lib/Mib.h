//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once

#include "stdheader.h"
#include "Signal.h"
#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"

//------------------START OF DEFINITION--------------------------------//
// ASC INTERSECTION MIB :NTCIP 1202
// 1.3.6.1.4.1.1206.4.1.3.1.1.3
#define RED_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.2.1" // Object
#define YELLOW_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.3.1"
#define GREEN_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.4.1"
#define DONOTWALK_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.5.1"
#define PEDCLEAR_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.6.1"
#define WALK_GROUP "1.3.6.1.4.1.1206.4.2.1.1.4.1.7.1"
#define VEHICLE_CALL "1.3.6.1.4.1.1206.4.2.1.1.4.1.8.1"
#define PEDES_CALL "1.3.6.1.4.1.1206.4.2.1.1.4.1.8.1" // AT this time just use vehicle call
// Peds
#define PED_CALL "1.3.6.1.4.1.1206.4.2.1.1.4.1.9.1"

// Phase control
#define MIB_PHASE_HOLD "1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1"
#define MIB_PHASE_FORCEOFF "1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1"
#define MIB_PHASE_OMIT "1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1"
#define MIB_PHASE_VEH_CALL "1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1"
#define MIB_PHASE_PED_CALL "1.3.6.1.4.1.1206.4.2.1.1.5.1.7.1" // Add by YF 04/27/2015

// Controller configure information
#define MAX_PHASE_NO "1.3.6.1.4.1.1206.4.2.1.1.1"
#define PHASE_NUMBER \
    "1.3.6.1.4.1.1206.4.2.1.1.2.1.1.1" // phase number: last ".X" is phase number, the return value
                                       // is also X

#define CUR_TIMING_PLAN "1.3.6.1.4.1.1206.3.5.2.1.22.0" // return the current timing plan

#define PHASE_ENABLED \
    "1.3.6.1.4.1.1206.4.2.1.1.2.1.21." // Phase options: last ".X" is phase, the last bit of return
                                       // result is "0", the phase is not enabled.
//------------The following from standard: only read PLAN 1---------//
#define PHASE_MIN_GRN \
    "1.3.6.1.4.1.1206.4.2.1.1.2.1.4." // need last "x" is the phase number: return the minimun green
                                      // of phase x
#define PHASE_MAX_GRN "1.3.6.1.4.1.1206.4.2.1.1.2.1.6."
#define PHASE_RED_CLR "1.3.6.1.4.1.1206.4.2.1.1.2.1.9."
#define PHASE_YLW_XGE "1.3.6.1.4.1.1206.4.2.1.1.2.1.8."
#define PED_WALK "1.3.6.1.4.1.1206.4.2.1.1.2.1.2."
#define PED_CLEAR "1.3.6.1.4.1.1206.4.2.1.1.2.1.3."
//------------The following from ASC3: WILL BE USED---------//
#define PHASE_MIN_GRN_ASC \
    "1.3.6.1.4.1.1206.3.5.2.1.2.1.9." // need last "x.p" x is the timing plan number,p is the phase
                                      // number: x get from CUR_TIMING_PLAN
#define PHASE_MAX_GRN_ASC "1.3.6.1.4.1.1206.3.5.2.1.2.1.15."
#define PHASE_RED_CLR_ASC "1.3.6.1.4.1.1206.3.5.2.1.2.1.19."
#define PHASE_YLW_XGE_ASC "1.3.6.1.4.1.1206.3.5.2.1.2.1.18."

//**********asc3PhaseStatusTiming
// T (1):       Phase Timing
// N (2):       Phase Next
//- (3):       Phase Not Enabled
//(space) (4): Phase Not Timing or Next
#define PHASE_STA_TIME_ASC "1.3.6.1.4.1.1206.3.5.2.1.18.1.1." // NEED last "p"  for the phase
//**********asc3PhaseStatusTiming2
// (1) X: XPED timing
// (2) N: Phase Next
// (3) -: Phase Not enabled
// (4) .: Phase Not Timing
// (5) R: Phase Timing RED
// (6) Y: Phase Timing YEL
// (7) G: Phase Timing GREEN

// (8) D: Phase Timing DELAY GREEN
// (9) O: Phase Timing YEL & RED
//(10) g: Phase Timing FLASHING GREEN
//(11) y: Phase Timing FLASHING YELLOW "
#define PHASE_STA_TIME2_ASC "1.3.6.1.4.1.1206.3.5.2.1.18.1.6." // NEED last "p"  for the phase

// Added by Shayan 5.5.14
#define SYS_DET_VOL \
    "1.3.6.1.4.1.1206.4.2.1.2.5.4.1.1." // NEED last "d" for the detector number to collect Volume
#define SYS_DET_OCC \
    "1.3.6.1.4.1.1206.4.2.1.2.5.4.1.2." // NEED last "d" for the detector number to collect
                                        // Occupancy

// Added by Yiheng 5.5.2015  System Pattern Control for Coordination
#define SYS_PAT_CON "1.3.6.1.4.1.1206.4.2.1.4.14.1"

#define PORT 15020 // PREEMPTION port
#define BROADCAST_ADDR "192.168.255.255"
namespace WaveApp
{
namespace Mmitss
{

//------------------END OF DEFINITION--------------------------------//
// define the different phase control types
enum PhaseControlAction : int
{
    ForceOff = 0,
    Omit = 1,
    VehCall = 2,
    Hold = 3,
    PedCall = 4
};

using PedInfo = std::array<int, 3>;
using byte = uint8_t;

void PackEventList(std::vector<byte>& tmp_event_data);

class MmitssMib
{
  public:
    using SocketAddress = Poco::Net::SocketAddress;
    using PhaseState = std::array<int, 8>;

    MmitssMib(){};
    MmitssMib(const SocketAddress& controllerAddress);
    virtual ~MmitssMib() = default;
    MmitssMib(const MmitssMib&) = delete;
    MmitssMib& operator=(const MmitssMib&) = delete;

    void setControllerAddress(const SocketAddress& controllerAddress)
    {
        m_controllerAddress = controllerAddress;
    };

    int GetSignalColor(int PhaseStatusNo);
    void PhaseTimingStatusRead();
    int CurTimingPlanRead();
    void IntersectionConfigRead(int CurTimingPlanNo, const std::string& ConfigOutFile);
    void IntersectionPedConfigRead(int CurTimingPlanNo, const std::string& ConfigOutFile);
    void IntersectionPhaseControl(int phase_control, int Total, char YES);
    void DetReadVolume(const std::vector<int>& detNumber, int totNum);
    void DetReadOccupancy(std::vector<int>& occupancy);
    void PedStatusRead();

    // The output of this function shows there is either a ped call, ped is in walking state, or ped
    // is in clearing state
    int CheckPedPhaseInfo();

    // Used for savari SMARTCROSS project to handle the received SRM from phone!
    void SendPedCall(int pedReqPhase);
    // To clearing the commandds after the request table becomes empty
    void SendClearCommands();
    void SendClearPedCall(int ped_phase);

    const std::array<int, 20>& getOut() const { return m_out; };
    const PhaseState& getPhaseStatus() const { return m_curPhaseStatus; };
    const PedInfo& getPedInfo() const { return m_pedInfo; };
    const PhaseState& getPedPhaseConsidered() const { return m_pedPhaseConsidered; };
    const PhaseState& getPedReqFlagForClean() const { return m_pedReqFlagForClean; };
    const PhaseStatus& getPhaseRead() const { return m_phaseRead; };

  private:
    using UdpSocket = Poco::Net::DatagramSocket;

    SocketAddress m_controllerAddress;

    PhaseStatus m_phaseRead;
    PhaseState m_curPhaseStatus;
    std::array<int, 20> m_out;
    PedInfo m_pedInfo; //[0]: ped status, [1]: ped call in integer number, [2]: ped clearing
    PhaseState m_pedPhaseConsidered; // Which ped phase need to be considered, If it is 0, there is
                                     // no ped, if it is 1, there is call, if it is 2 , ped is in
                                     // walking state, if it is 3, ped in in clearance state
    PhaseState m_pedReqFlagForClean;
};

} // namespace Mmitss
} // namespace WaveApp
