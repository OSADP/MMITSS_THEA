#include "stdheader.h"
#include "mmitssTrajectoryAwareApp.hpp"

namespace WaveApp
{
namespace Mmitss
{

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssTraj",
    .name = "mmitss_trajectory_aware",
    .description = "MMITSS Trajectory Aware",
    .uuid = "5b58b275-5ea4-4cac-a6f4-aa0e95934da6",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_trajectory_aware",
    .busPath = "/com/siemens/c2x/mmitss_trajectory_aware",
    .busInterface = "com.siemens.c2x.mmitss_trajectory_aware.Service",

    .busMethods = nullptr,
    .busProperties = nullptr,
    .busSignals = nullptr,

    .memoryType = CACHEMEM_SIZE_TYPE::SMALL,
    .pipeType = sieipc::PipeType::SMALL,
    .idleDelay = 5000,
    .gpsActivate = false,

    // TODO: fix we're not running as root
    //       should be APPLICATION_UID, APPLICATION_GID then
    .facilityUser = 0,
    .facilityGroup = 0,
    .facilityCapabilities = 0};

MmitssTrajectoryAwareApp::MmitssTrajectoryAwareApp()
    : m_trajAware(*this)
{
}

SERVICE_STATE MmitssTrajectoryAwareApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssTrajectoryAwareApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssTrajectoryAwareApp::getCommandList(its::ItsFacilityCommandList& cmd)
{
    NOTUSED(cmd);
}

void MmitssTrajectoryAwareApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssTrajectoryAwareApp::onNetRxBsm(
    const saeBasicSafetyMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(pkt);
    m_trajAware.onRxBsm(msg);
    return true;
}

bool MmitssTrajectoryAwareApp::onStartup()
{
    return m_trajAware.onStartup();
}

void MmitssTrajectoryAwareApp::onShutdown()
{
    return m_trajAware.onShutdown();
}

void MmitssTrajectoryAwareApp::onExit()
{
    m_trajAware.onExit();
}

void MmitssTrajectoryAwareApp::onSystemTick()
{
    m_trajAware.tick();
}

void MmitssTrajectoryAwareApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssTrajectoryAwareApp::onFitReset()
{
    return true;
}

bool MmitssTrajectoryAwareApp::onFitCommand(
    std::string& result, const char* command, const char* args)
{
    NOTUSED(result);
    NOTUSED(command);
    NOTUSED(args);
    return true;
}

} // namespace Mmitss
} // namespace WaveApp

static void showHelp()
{
    fprintf(stdout, "mmitss_trajectory_aware [-d]\n");
}

int main(int argc, char* argv[])
{
    int opt, verbose = 0;
    /* do normal option parsing */
    while ((opt = getopt(argc, argv, "d")) > 0)
    {
        switch (opt)
        {
            case 'd':
                verbose++;
                break;
            default:
                showHelp();
                return 1;
        }
    }
    try
    {
        return its::facility_main(verbose, WaveApp::Mmitss::theFacilityInfo,
            [] { return new WaveApp::Mmitss::MmitssTrajectoryAwareApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_trajectory_aware: " << ex.what() << std::endl;
    }
    return 0;
}
