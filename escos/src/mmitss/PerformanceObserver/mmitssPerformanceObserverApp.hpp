#pragma once
#include "stdheader.h"

#include "mmitssPerformanceObserver.hpp"

#include "facilityLayer.hpp"
#include "facilityInfo.hpp"
#include "facilityWave.hpp"
#include "facilityWaveSae.hpp"
#include "Poco/Net/DatagramSocket.h"

#include "SieMmitssConfig.hpp"

namespace WaveApp
{
namespace Mmitss
{

/**
 *  MmitssTrajectoryAware class
 */
class MmitssPerformanceObserverApp : public ItsFacilityApplicationWaveSae
{
  public:
    MmitssPerformanceObserverApp();
    virtual ~MmitssPerformanceObserverApp() = default;
    MmitssPerformanceObserverApp(const MmitssPerformanceObserverApp&) = delete;
    MmitssPerformanceObserverApp& operator=(const MmitssPerformanceObserverApp&) = delete;

    /// Overrides from baseclass
    /// @{
    virtual int getFacilityVersion() const override { return FACILITY_INTERFACE_VERSION; }

    virtual bool onStartup() override;
    virtual void onShutdown() override;
    virtual void onExit() override;

    virtual void onSystemTick() override;

    virtual SERVICE_STATE onSystemCheck(const char* arg, std::string& result) override;
    virtual void onDatabaseChange(const char* key, const char* value) override;

    virtual void getCommandList(its::ItsFacilityCommandList& cmdList) override;
    virtual void getDatabaseHooks(KeyList& keys) const override;
    /// @}

    // Fitnesse
    virtual void onFitSyntax(std::string& msg) const override;
    virtual bool onFitReset() override;
    virtual bool onFitCommand(std::string& result, const char* command, const char* args) override;

  private:
    std::string handle_json(int argc, char* argv[]);

    bool m_configDirty{false};
    std::string m_status{"Not running"};

    MmitssPerformanceObserver m_perfObs;
};

} // namespace Mmitss
} // namespace WaveApp
