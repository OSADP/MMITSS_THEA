#include "mmitssPriorityRequestServerApp.hpp"

#include <getopt.h>
#include <Poco/Dynamic/Struct.h>
#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <chrono>
#include <cstdio>
#include <exception>
#include <iostream>
#include <sstream>
#include <vector>

#include "facilityInfo.hpp"
#include "facilityLayer.hpp"
#include "timeUtil.hpp"

namespace WaveApp
{
namespace Mmitss
{

MmitssPriorityRequestServerApp::MmitssPriorityRequestServerApp()
    : m_prioReq(*this)
{
}

SERVICE_STATE MmitssPriorityRequestServerApp::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    NOTUSED(result);
    return SERVICE_STATE::UNKNOWN;
}

void MmitssPriorityRequestServerApp::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(key);
    NOTUSED(value);
}

void MmitssPriorityRequestServerApp::getCommandList(its::ItsFacilityCommandList& cmd)
{
    cmd.emplace_back(its::ItsFacilityCommand{"json", "JSON command interface",
        [this](int argc, char* argv[]) { return this->handle_json(argc, argv); }});
}

std::string MmitssPriorityRequestServerApp::handle_json(int argc, char* argv[])
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

        if (cmd == "grantedStatus")
        {
            std::string vehicleId{""}, granted{"true"};
            if (json->has("vehicle_id"))
            {
                vehicleId = json->get("vehicle_id").toString();
            }
            if (json->has("request_granted"))
            {
                granted = json->get("request_granted").toString();
            }
            m_prioReq.updateGrantedStatus(vehicleId, granted);
        }
        else if (cmd == "sendSRM")
        {
            std::string vehicleId{""}, name{""};
            int type{0}, inLane{0}, outLane{0}, eta{0};
            if (json->has("vehId"))
            {
                vehicleId = json->get("vehId").toString();
            }
            if (json->has("name"))
            {
                name = json->get("name").toString();
            }
            if (json->has("type"))
            {
                type = std::stoi(json->get("type").toString());
            }
            if (json->has("inLane"))
            {
                inLane = std::stoi(json->get("inLane").toString());
            }
            if (json->has("outLane"))
            {
                outLane = std::stoi(json->get("outLane").toString());
            }
            if (json->has("eta"))
            {
                eta = std::stoi(json->get("eta").toString());
            }
            auto turnTypeIntoString = [](int _type) {
                switch (_type)
                {
                    case 0:
                        return "priorityRequest";
                    case 1:
                        return "priorityRequestUpdate";
                    case 2:
                        return "priorityCancellation";
                    default:
                        return "N/A";
                }
            };
            std::chrono::seconds sec(eta);

            auto arrivingAt = chrono::system_clock::now() + std::chrono::duration_cast<mSecs>(sec);
            auto minutes = WaveApp::timeUtil::toMinuteOfTheYear(arrivingAt);
            auto seconds = WaveApp::timeUtil::toDSecond(arrivingAt);
            auto endHour = (minutes / 60) % 24;
            auto endMinute = minutes % 60;
            auto endSecond = (seconds / 1000) % 60;
            ITSAPP_TRACE("vehicleId: %s, name: %s, type: %d, inLane: %d, outLane: %d, eta: %d",
                vehicleId.c_str(), name.c_str(), type, inLane, outLane, eta);
            ITSAPP_TRACE(
                "endHour: %d, endMinute: %d, endSecond: %d", endHour, endMinute, endSecond);
            std::string srm_json = getSRMJson(m_prioReq.getIntersectionId(), vehicleId, name,
                turnTypeIntoString(type), inLane, outLane, minutes, seconds);

            m_prioReq.RecvSrmFromCommand(srm_json.c_str(), srm_json.size());
        }
        else if (cmd == "dispReqList")
        {
            res["dispReqList"] = m_prioReq.reqListAsString();
        }
        else if (cmd == "enableGrantRequest")
        {
            int enable{0};
            if (json->has("enable"))
            {
                enable = std::stoi(json->get("enable").toString(), nullptr, 10);
            }
            m_prioReq.enableGrantRequest(enable > 0 ? true : false);
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

std::string MmitssPriorityRequestServerApp::getSRMJson(int intersection,
    std::string vehId,
    std::string name,
    std::string type,
    int inLane,
    int outLane,
    int64_t endTimeMin,
    int64_t endTimeSec)
{
    std::ostringstream ss;

    ss << R"({"msgCnt":22,"speed":0,"speed2":0.0,"vehicleID":)" << vehId << R"(,"name":)" << name
       << R"(,"vehicleRole":"transit","requests":[{"duration":5000,"endTimeMin":)" << endTimeMin
       << R"(,"endTimeSec":)" << endTimeSec << R"(,"inLane":)" << inLane << R"(,"intersectionID":)"
       << intersection << R"(,"outLane":)" << outLane << R"(,"requestID":3,"requestType":")" << type
       << R"("}]})";
    return ss.str();
}

void MmitssPriorityRequestServerApp::getDatabaseHooks(KeyList& keys) const
{
    NOTUSED(keys);
}

bool MmitssPriorityRequestServerApp::onStartup()
{
    return m_prioReq.onStartup();
}

void MmitssPriorityRequestServerApp::onShutdown()
{
    m_prioReq.onShutdown();
}

void MmitssPriorityRequestServerApp::onExit()
{
    m_prioReq.onExit();
}

void MmitssPriorityRequestServerApp::onSystemTick()
{
    m_prioReq.tick();
}

void MmitssPriorityRequestServerApp::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssPriorityRequestServerApp::onFitReset()
{
    return true;
}

bool MmitssPriorityRequestServerApp::onFitCommand(
    std::string& result, const char* command, const char* args)
{
    NOTUSED(result);
    NOTUSED(command);
    NOTUSED(args);
    return true;
}

bool MmitssPriorityRequestServerApp::onNetRxSrm(
    const saeSignalRequestMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(pkt);
    return m_prioReq.onRxSrm(msg);
}

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssPrioReq",
    .name = "mmitss_requestor",
    .description = "MMITSS Priority Request Server",
    .uuid = "9012741d-dd34-4b37-8b00-de2d8036e6a6",

    .versionMajor = 1,
    .versionMinor = 0,

    .busName = "com.siemens.c2x.mmitss_prio_req",
    .busPath = "/com/siemens/c2x/mmitss_prio_req",
    .busInterface = "com.siemens.c2x.mmitss_prio_req.Service",

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

} // namespace Mmitss
} // namespace WaveApp

static void showHelp()
{
    fprintf(stdout, "mmitss_prio_req [-d]\n");
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
            [] { return new WaveApp::Mmitss::MmitssPriorityRequestServerApp(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running mmitss_prio_req: " << ex.what() << std::endl;
    }
    return 0;
}
