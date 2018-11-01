#pragma once
#include "stdheader.h"

#include "facilityLayer.hpp"
#include "facilityInfo.hpp"
#include "facilityWave.hpp"
#include "facilityWaveSae.hpp"

#include "SieMmitssConfig.hpp"
#include "mmitssTrajectoryAware.hpp"

namespace WaveApp
{
namespace Mmitss
{

/**
 *  MmitssTrajectoryAware class
 */
class MmitssTrajectoryAwareApp : public ItsFacilityApplicationWaveSae
{
  public:
    MmitssTrajectoryAwareApp();
    virtual ~MmitssTrajectoryAwareApp() = default;
    MmitssTrajectoryAwareApp(const MmitssTrajectoryAwareApp&) = delete;
    MmitssTrajectoryAwareApp& operator=(const MmitssTrajectoryAwareApp&) = delete;

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
    virtual bool getPsidList(its::PsidList& psidList) const
    { // register for BSM
        psidList.insert(its::DOT3_P_PSID_VVSA);
        return true;
    }

    virtual bool onNetRxBsm(
        const saeBasicSafetyMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);

    /// @}

    // Fitnesse
    virtual void onFitSyntax(std::string& msg) const override;
    virtual bool onFitReset() override;
    virtual bool onFitCommand(std::string& result, const char* command, const char* args) override;

  private:
    /*std::string handle_status(int argc, char* argv[]);
    std::string handle_json(int argc, char* argv[]);
    std::string handle_start(int argc, char* argv[]);
    std::string handle_stop(int argc, char* argv[]);*/

    bool m_configDirty{false};
    std::string m_status{"Not running"};

    MmitssTrajectoryAware m_trajAware;
};

} // namespace Mmitss
} // namespace WaveApp
