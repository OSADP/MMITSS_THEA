#pragma once

#include <stdtypes.h>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

#include "facilityBase.hpp"
#include "facilityWaveSae.hpp"
#include "ieee1609Packet.hpp"
#include "itsWave.hpp"
#include "mmitssPriorityRequestServer.hpp"

namespace WaveApp
{
namespace Mmitss
{

class ReqEntryListHandle;

class MmitssPriorityRequestServerApp : public ItsFacilityApplicationWaveSae
{
  public:
    MmitssPriorityRequestServerApp();
    virtual ~MmitssPriorityRequestServerApp() = default;
    MmitssPriorityRequestServerApp(const MmitssPriorityRequestServerApp&) = delete;
    MmitssPriorityRequestServerApp& operator=(const MmitssPriorityRequestServerApp&) = delete;

    /// Overrides from baseclass
    virtual int getFacilityVersion() const override { return FACILITY_INTERFACE_VERSION; }

    virtual bool onStartup() override;
    virtual void onShutdown() override;
    virtual void onExit() override;

    virtual void onSystemTick() override;

    virtual SERVICE_STATE onSystemCheck(const char* arg, std::string& result) override;
    virtual void onDatabaseChange(const char* key, const char* value) override;

    virtual void getCommandList(its::ItsFacilityCommandList& cmd) override;
    virtual void getDatabaseHooks(KeyList& keys) const override;

    virtual bool getPsidList(its::PsidList& psidList) const
    {
        // register for SRM
        psidList.insert(PSID_THEA_SRM);
        return true;
    }

    virtual bool onNetRxSrm(
        const saeSignalRequestMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);

    // Fitnesse
    virtual void onFitSyntax(std::string& msg) const override;
    virtual bool onFitReset() override;
    virtual bool onFitCommand(std::string& result, const char* command, const char* args) override;
    void requestPriority(std::string rsuID, int intersectionID, std::string vehicleID);

  private:
    std::string handle_json(int argc, char* argv[]);
    std::string getSRMJson(int intersection,
        std::string vecId,
        std::string name,
        std::string type,
        int inLane,
        int outLane,
        int64_t endTimeMin,
        int64_t endTimeSec);

    bool m_configDirty{false};
    std::string m_status{"Not running"};

    MmitssPriorityRequestServer m_prioReq;
};

} // namespace Mmitss
} // namespace WaveApp
