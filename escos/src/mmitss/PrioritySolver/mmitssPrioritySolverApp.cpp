#include "stdheader.h"

#include "mmitssPrioritySolverApp.hpp"
#include "Poco/Dynamic/Struct.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/ByteOrder.h"

namespace WaveApp
{
namespace Mmitss
{

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssPrioSolv",
    .name = "mmitss_solver",
    .description = "MMITSS Priority Solver",
    .uuid = "038745fa-51eb-4c5d-a2bb-ef899cf037d0",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_prio_solver",
    .busPath = "/com/siemens/c2x/mmitss_prio_solver",
    .busInterface = "com.siemens.c2x.mmitss_prio_solver.Service",

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

MmitssPrioritySolverApp::MmitssPrioritySolverApp()
    : m_prioSolver(*this)
{
}

SERVICE_STATE MmitssPrioritySolverApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssPrioritySolverApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssPrioritySolverApp::getCommandList(its::ItsFacilityCommandList& cmd)
{
    cmd.emplace_back(its::ItsFacilityCommand{"json", "JSON command interface",
        [this](int argc, char* argv[]) { return this->handle_json(argc, argv); }});
}

std::string MmitssPrioritySolverApp::handle_json(int argc, char* argv[])
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
        ITSAPP_LOG("handle command %s", cmd.c_str());

        if (cmd == "needSolve")
        {
            ITSAPP_LOG("Command: needSolve");
            if (json->has("list"))
            {
                m_prioSolver.solve(json->get("list").toString());
            }
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

void MmitssPrioritySolverApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssPrioritySolverApp::onStartup()
{
    return m_prioSolver.onStartup();
}

void MmitssPrioritySolverApp::onShutdown()
{
    m_prioSolver.onShutdown();
}

void MmitssPrioritySolverApp::onExit()
{
    m_prioSolver.onExit();
}

void MmitssPrioritySolverApp::onSystemTick()
{
    m_prioSolver.tick();
}

void MmitssPrioritySolverApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssPrioritySolverApp::onFitReset()
{
    return true;
}

bool MmitssPrioritySolverApp::onFitCommand(
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
    fprintf(stdout, "mmitss_prio_solver [-d]\n");
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
            [] { return new WaveApp::Mmitss::MmitssPrioritySolverApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_prio_solver: " << ex.what() << std::endl;
    }
    return 0;
}
