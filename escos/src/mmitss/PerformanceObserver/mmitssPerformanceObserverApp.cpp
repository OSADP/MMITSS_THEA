#include "stdheader.h"
#include "mmitssPerformanceObserverApp.hpp"

#include "facilityDatabaseAdapter.hpp"
#include "SieMmitssConfig.hpp"
#include "Poco/Base64Encoder.h"
#include "Poco/DynamicStruct.h"
#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Parser.h"

namespace WaveApp
{
namespace Mmitss
{

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssPerf",
    .name = "mmitss_perf_observer",
    .description = "MMITSS Performance Observer",
    .uuid = "6fb39c66-e12b-4165-bde3-d0e9bb9a7b2f",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_perf_observer",
    .busPath = "/com/siemens/c2x/mmitss_perf_observer",
    .busInterface = "com.siemens.c2x.mmitss_perf_observer.Service",

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

MmitssPerformanceObserverApp::MmitssPerformanceObserverApp()
    : m_perfObs(*this)
{
}

SERVICE_STATE MmitssPerformanceObserverApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssPerformanceObserverApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssPerformanceObserverApp::getCommandList(its::ItsFacilityCommandList& cmdList)
{
    cmdList.emplace_back(its::ItsFacilityCommand{"json", "JSON command interface",
        [this](int argc, char* argv[]) { return this->handle_json(argc, argv); }});
}

std::string MmitssPerformanceObserverApp::handle_json(int argc, char* argv[])
{
    Poco::DynamicStruct res;

    if (argc <= 0)
    {
        ITSAPP_LOG("invalid args error for %s", argv[0]);
        res["error"] = "invalid args";
        return res.toString();
    }

    try
    {
        std::string strval;
        // join args
        for (int i = 0; i < argc; i++)
        {
            if (!strval.empty())
                strval.append(" ");
            strval.append(argv[i]);
        }
        Poco::JSON::Parser parser;
        auto json = parser.parse(strval).extract<Poco::JSON::Object::Ptr>();

        if (!json->has("command"))
        {
            res["error"] = "no command attr";
            return res.toString();
        }
        std::string cmd = json->get("command").toString();
        ITSAPP_TRACE("handle command %s", cmd.c_str());

        if (cmd == "detectorEvent")
        {
            int lane_id{0};
            double speed{0.0}, distance{0.0};
            if (json->has("speed"))
            {
                speed = std::stod(json->get("speed").toString());
            }
            if (json->has("lane_id"))
            {
                lane_id = std::stoi(json->get("lane_id").toString(), nullptr, 10);
            }
            if (json->has("distance"))
            {
                distance = std::stod(json->get("distance").toString());
            }
            // insert to forwarded list of m_perfObs
            ITSAPP_TRACE("distance: %f,lane_id %d, speed %f", distance, lane_id, speed);
            m_perfObs.addDetectedVeh(lane_id, speed, distance);
        }
        else if (cmd == "changeRadarLaneOrder")
        {
            int increased{1};
            if (json->has("order"))
            {
                increased = std::stoi(json->get("order").toString(), nullptr, 10);
            }
            m_perfObs.setRadarReportedLaneOrder(increased > 0 ? true : false);
        }
        else
        {
            res["error"] = "Unknown command: " + cmd;
        }
    }
    catch (const std::exception& ex)
    {
        ITSAPP_WRN("JSON Processing: %s", ex.what());
        res["error"] = std::string("ERROR: ") + std::string(ex.what());
    }
    return res.toString();
}

void MmitssPerformanceObserverApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssPerformanceObserverApp::onStartup()
{
    return m_perfObs.onStartup();
}

void MmitssPerformanceObserverApp::onShutdown()
{
    m_perfObs.onShutdown();
}

void MmitssPerformanceObserverApp::onExit()
{
    m_perfObs.onExit();
}

void MmitssPerformanceObserverApp::onSystemTick()
{
    m_perfObs.tick();
}

void MmitssPerformanceObserverApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssPerformanceObserverApp::onFitReset()
{
    return true;
}

bool MmitssPerformanceObserverApp::onFitCommand(
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
    fprintf(stdout, "mmitss_perf_observer [-d]\n");
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
            [] { return new WaveApp::Mmitss::MmitssPerformanceObserverApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_perf_observer: " << ex.what() << std::endl;
    }
    return 0;
}
