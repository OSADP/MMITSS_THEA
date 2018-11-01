#include "stdheader.h"

#include "mmitssSignalControlApp.hpp"

#include "Poco/Dynamic/Struct.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"

namespace WaveApp
{
namespace Mmitss
{

static const std::string MMITSS_CTRL_DB_ROOT_KEY = "its/us/mmitss/control";

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssSigCtrl",
    .name = "mmitss_isig",
    .description = "MMITSS Signal Control",
    .uuid = "47595a0c-4c08-4567-8371-41d3f964fbb8",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_isig",
    .busPath = "/com/siemens/c2x/mmitss_isig",
    .busInterface = "com.siemens.c2x.mmitss_isig.Service",

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

MmitssSignalControlApp::MmitssSignalControlApp()
    : m_signalControl(*this)
{
}

SERVICE_STATE MmitssSignalControlApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssSignalControlApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssSignalControlApp::getCommandList(its::ItsFacilityCommandList& cmdList)
{
    NOTUSED(cmdList);
}

void MmitssSignalControlApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssSignalControlApp::onStartup()
{
    return m_signalControl.onStartup();
}

void MmitssSignalControlApp::onShutdown()
{
    m_signalControl.onShutdown();
}

void MmitssSignalControlApp::onExit()
{
    m_signalControl.onExit();
}

void MmitssSignalControlApp::onSystemTick()
{
    m_signalControl.tick();
}

void MmitssSignalControlApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssSignalControlApp::onFitReset()
{
    return true;
}

bool MmitssSignalControlApp::onFitCommand(
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
    fprintf(stdout, "mmitss_isig [-d]\n");
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
            [] { return new WaveApp::Mmitss::MmitssSignalControlApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_isig: " << ex.what() << std::endl;
    }
    return 0;
}
