#include "stdheader.h"

#include "mmitssTrafficControllerInterfaceApp.hpp"

namespace WaveApp
{
namespace Mmitss
{

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssTrafficCtrl",
    .name = "mmitss_controller",
    .description = "MMITSS Traffic Controller Interface",
    .uuid = "af207ff2-0fcb-47a9-8009-ec443a294acf",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_traffic_control",
    .busPath = "/com/siemens/c2x/mmitss_traffic_control",
    .busInterface = "com.siemens.c2x.mmitss_traffic_control.Service",

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

MmitssTrafficControllerInterfaceApp::MmitssTrafficControllerInterfaceApp()
    : m_trafficControl(*this)
{
}

SERVICE_STATE
MmitssTrafficControllerInterfaceApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssTrafficControllerInterfaceApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssTrafficControllerInterfaceApp::getCommandList(its::ItsFacilityCommandList& cmd)
{
    NOTUSED(cmd);
}

void MmitssTrafficControllerInterfaceApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssTrafficControllerInterfaceApp::onStartup()
{
    return m_trafficControl.onStartup();
}

void MmitssTrafficControllerInterfaceApp::onShutdown()
{
    m_trafficControl.onShutdown();
}

void MmitssTrafficControllerInterfaceApp::onExit()
{
    m_trafficControl.onExit();
}

void MmitssTrafficControllerInterfaceApp::onSystemTick()
{
    m_trafficControl.tick();
}

void MmitssTrafficControllerInterfaceApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssTrafficControllerInterfaceApp::onFitReset()
{
    return true;
}

bool MmitssTrafficControllerInterfaceApp::onFitCommand(
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
    fprintf(stdout, "mmitss_traffic_control [-d]\n");
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
            [] { return new WaveApp::Mmitss::MmitssTrafficControllerInterfaceApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_traffic_control: " << ex.what() << std::endl;
    }
    return 0;
}
