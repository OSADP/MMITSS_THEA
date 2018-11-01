#pragma once

#include "facilityLayer.hpp"
#include "facilityInfo.hpp"
#include "facilityWave.hpp"
#include "facilityWaveSae.hpp"

#include "mmitssSignalControl.hpp"

namespace WaveApp
{
namespace Mmitss
{

/**
 *  MmitssTrajectoryAware class
 */
class MmitssSignalControlApp : public ItsFacilityApplicationWaveSae
{
  public:
    MmitssSignalControlApp();
    virtual ~MmitssSignalControlApp() = default;
    MmitssSignalControlApp(const MmitssSignalControlApp&) = delete;
    MmitssSignalControlApp& operator=(const MmitssSignalControlApp&) = delete;

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

    /// @}

    // Fitnesse
    virtual void onFitSyntax(std::string& msg) const override;
    virtual bool onFitReset() override;
    virtual bool onFitCommand(std::string& result, const char* command, const char* args) override;

  private:
    // std::string handle_status(int argc, char* argv[]);
    // std::string handle_json(int argc, char* argv[]);

    bool m_configDirty{false};
    std::string m_status{"Not running"};

    MmitssSignalControl m_signalControl;
};

} // namespace Mmitss
} // namespace WaveApp
