#pragma once

#include "facilityLayer.hpp"
#include "facilityInfo.hpp"
#include "facilityWave.hpp"
#include "facilityWaveSae.hpp"
#include "spatMapProxy.hpp"
#include "mmitssServiceControl.hpp"
#include "mmitssConfig.hpp"
#include "mmitssPerfData.hpp"
#include "mmitssUtils.h"
#include "Poco/Net/DatagramSocket.h"

namespace WaveApp
{
namespace MmitssCtrl
{

/**
 *  MmitssControllerDaemon class
 *
 *  Controls MMITSS services:
 *   - start/stop
 *   - config file generation
 *   - monitoring
 */
class MmitssControllerDaemon : public ItsFacilityApplicationWaveSae
{
  public:
    MmitssControllerDaemon();
    virtual ~MmitssControllerDaemon() = default;
    MmitssControllerDaemon(const MmitssControllerDaemon&) = delete;
    MmitssControllerDaemon& operator=(const MmitssControllerDaemon&) = delete;

    /// Overrides from baseclass
    /// @{
    virtual int getFacilityVersion() const override { return FACILITY_INTERFACE_VERSION; }

    virtual bool onStartup() override;
    virtual void onShutdown() override;
    virtual void onExit() override;

    virtual void onSystemTick() override;

    virtual SERVICE_STATE onSystemCheck(const char* arg, std::string& result) override;
    virtual void onDatabaseChange(const char* key, const char* value) override;

    virtual void getCommandList(its::ItsFacilityCommandList& cmd) override;
    virtual void getDatabaseHooks(KeyList& keys) const override;

    /// retrieve PSIDs to listen
    /// keep empty to listen for ALL messages
    /// @return false (default) for NOT registering for upstream messages
    /*virtual bool getPsidList(its::PsidList& psidList) const
    {   // register for BSM
        //psidList.insert(its::DOT3_P_PSID_VVSA);
        // register for SRM
        //psidList.insert(PSID_THEA_SRM);
        return true;
    }

    virtual bool onNetRxBsm(const saeBasicSafetyMessage& msg,
    ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxSrm(const saeSignalRequestMessage& msg,
    ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);*/

    /// @}

    // Fitnesse
    virtual void onFitSyntax(std::string& msg) const override;
    virtual bool onFitReset() override;
    virtual bool onFitCommand(std::string& result, const char* command, const char* args) override;

  private:
    using SignalPtr = std::unique_ptr<siekdbus::Signal>;

    void readConfig();

    bool startMmitss();
    bool stopMmitss();

    void onSpatMapConfigChange(siekdbus::Message& in);

    void updateCurSignalPlan();
    bool isMmitssSignalPlanActive() const;

    void udpBsmXmit(const void* data, size_t size);
    void udpSrmXmit(const void* data, size_t size);
    void udpInit();
    void udpClose();

    std::string handle_status(int argc, char* argv[]);
    std::string handle_json(int argc, char* argv[]);
    std::string handle_start(int argc, char* argv[]);
    std::string handle_stop(int argc, char* argv[]);
    std::string handle_changeRadarLaneOrder(int argc, char* argv[]);
    std::string handle_enableGrantRequest(int argc, char* argv[]);
    std::string handle_requestPriority(int argc, char* argv[]);
    std::string handle_sendSRM(int argc, char* argv[]);
    std::string handle_respGrant(int argc, char* argv[]);
    std::string handle_getIdFromOctetStr(int argc, char* argv[]);
    std::string handle_dispReqList(int argc, char* argv[]);
    void requestPriority(std::string rsuID, std::string intersectionID, std::string vehicleID);

    bool m_configDirty{false};

    std::string m_status{"Not running"};

    MmitssOperatingMode m_opMode{Passive};
    MmitssServiceControl m_serviceCtrl;
    SpatMapProxy m_spatMap;
    MmitssConfig m_mmitssCfg;
    MmitssPerfData m_perfData;
    bool m_mmitssEnabled{false};
    bool m_autoGenerateMap{false};

    sigc::connection m_spatMapSignalConnection;
    SignalPtr m_spatMapSignal;

    Int32u m_bsmUdpXmit{0};
    Int32u m_bsmUdpError{0};
    Int32u m_srmUdpXmit{0};
    Int32u m_srmUdpError{0};
    Poco::Net::DatagramSocket m_bsmUdpSocket;
    Poco::Net::DatagramSocket m_srmUdpSocket;

    std::vector<int> m_sigPlanList;
    int m_curSignalPlan{-1};
};

} // namespace MmitssCtrl
} // namespace WaveApp
