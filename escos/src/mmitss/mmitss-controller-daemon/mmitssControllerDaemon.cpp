#include "stdheader.h"

#include "mmitssControllerDaemon.hpp"

#include "cmdLineParser.hpp"
#include "Poco/Dynamic/Struct.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"
#include "facilityDatabaseAdapter.hpp"
#include "SieMmitssBsm.hpp"
#include "SieMmitssSrm.hpp"
#include "base64.hpp"
#include "Poco/Net/SocketAddress.h"
#include "SieMmitssConfig.hpp"
#include "asnHelper.hpp"
#include "timeUtil.hpp"
namespace WaveApp
{
namespace MmitssCtrl
{

using namespace ::SieMmitss;

static const char* SPATMAP_CFG_SIGNAL = "ConfigChange";
static const char* SPATMAP_CFG_SIGNAL_SIGNATURE = "bb";
static const char* SPATMAP_CFG_SIGNAL_MATCH = "type='signal',"
                                              "sender='com.siemens.c2x.spatmap',"
                                              "interface='com.siemens.c2x.spatmap.Service',"
                                              "member='ConfigChange'";

static const std::string SPAT_MAP_DB_ROOT_KEY = "its/us/spat-map";
static const std::string ASC_DB_CFG_KEY = "its/us/asc/config";
static const std::string ASC_DB_OIDS_KEY = "its/us/asc/oids";
static const std::string MMITSS_CTRL_DB_ROOT_KEY = "its/us/mmitss/control";
static const std::string MMITSS_CTRL_SIG_PLN_LST_KEY = "its/us/mmitss/control/signal_plan_list";
static const std::string MMITSS_REQ_SERV_NEED_GRANT_KEY = "its/us/mmitss/priority/ask_for_grant";

static const Poco::Net::SocketAddress BSM_SOCKET("127.0.0.1", getTrajectoryAwarePort());
static const Poco::Net::SocketAddress SRM_SOCKET("127.0.0.1", getPrioReqSrvPort());

MmitssControllerDaemon::MmitssControllerDaemon()
    : m_spatMap(*this)
    , m_mmitssCfg(*this)
    , m_perfData(m_mmitssCfg, *this)
{
}

SERVICE_STATE MmitssControllerDaemon::onSystemCheck(const char* arg, std::string& result)
{
    NOTUSED(arg);
    SERVICE_STATE state = SERVICE_STATE::UNKNOWN;
    if (m_mmitssEnabled)
    {
        state = m_serviceCtrl.getStatus(m_opMode, result);
    }
    else
    {
        result = "Disabled";
    }
    return state;
}

void MmitssControllerDaemon::getDatabaseHooks(KeyList& keys) const
{
    keys.push_back(MMITSS_CTRL_DB_ROOT_KEY.c_str());
    keys.push_back(MMITSS_REQ_SERV_NEED_GRANT_KEY.c_str());
    keys.push_back(ASC_DB_CFG_KEY.c_str());
    keys.push_back(ASC_DB_OIDS_KEY.c_str());
}

void MmitssControllerDaemon::onDatabaseChange(const char* key, const char* value)
{
    NOTUSED(value);
    ITSAPP_LOG("database change notification: %s, %s", key, value);
    if (std::string(key) == MMITSS_REQ_SERV_NEED_GRANT_KEY)
    {
        Poco::DynamicStruct cmd;
        cmd["command"] = "enableGrantRequest";
        cmd["enable"] = value;

        ITSAPP_TRACE("%s", cmd.toString().c_str());

        siekdbus::Message reply;
        siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_req",
            "/com/siemens/c2x/mmitss_prio_req", "com.siemens.c2x.mmitss_prio_req.Service",
            "Command", "ss", "json", cmd.toString().c_str());
        return;
    }
    if (std::string(key) == ASC_DB_OIDS_KEY)
        updateCurSignalPlan();
    else
        m_configDirty = true;
}

void MmitssControllerDaemon::getCommandList(its::ItsFacilityCommandList& cmdList)
{
    cmdList.emplace_back(its::ItsFacilityCommand{"status", "get current status",
        [this](int argc, char* argv[]) { return this->handle_status(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"json", "JSON command interface",
        [this](int argc, char* argv[]) { return this->handle_json(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"start", "start service",
        [this](int argc, char* argv[]) { return this->handle_start(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"stop", "stop service",
        [this](int argc, char* argv[]) { return this->handle_stop(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"change_radar",
        "change the order of lanes detected by radar between increased/decreased",
        [this](int argc, char* argv[]) { return this->handle_changeRadarLaneOrder(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"enable_grant",
        "require asking for grant of a bus priority request",
        [this](int argc, char* argv[]) { return this->handle_enableGrantRequest(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"ask_for_grant",
        "ask for grant of a bus priority request in json format",
        [this](int argc, char* argv[]) { return this->handle_requestPriority(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"send_srm", "send one srm in json format",
        [this](int argc, char* argv[]) { return this->handle_sendSRM(argc, argv); }});
    cmdList.emplace_back(
        its::ItsFacilityCommand{"grant_resp", "send one grant message in json format",
            [this](int argc, char* argv[]) { return this->handle_respGrant(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"disp_reqList", "display priority request list",
        [this](int argc, char* argv[]) { return this->handle_dispReqList(argc, argv); }});
    cmdList.emplace_back(its::ItsFacilityCommand{"conv_oct", "get a string from a oct string",
        [this](int argc, char* argv[]) { return this->handle_getIdFromOctetStr(argc, argv); }});
}

std::string MmitssControllerDaemon::handle_getIdFromOctetStr(int argc, char* argv[])
{
    if (argc != 4)
        return R"({ "handle_getIdFromOctetStr":{ "error":"invalid args" } })";
    asnOctetString str{4}, str2{4};
    asnbyte b[4];
    // input ascii number from 0-255
    b[0] = std::stoi(argv[0], nullptr, 10);
    b[1] = std::stoi(argv[1], nullptr, 10);
    b[2] = std::stoi(argv[2], nullptr, 10);
    b[3] = std::stoi(argv[3], nullptr, 10);
    for (unsigned int i = 0; i < 4; i++)
        ITSAPP_LOG("0x%02x ", b[i]);
    str.setOctetString(4, b);
    string out = AsnHelper::octetStringToString(str);
    string dispStr = Mmitss::forDisplayInHexFormat(out);
    ITSAPP_LOG("%s ", dispStr.c_str());
    str2.setOctetString(out.length(), (unsigned char*)out.c_str());
    ITSAPP_LOG("setOctetString of %d chars ", out.length());
    unsigned int len;
    unsigned char* p;
    str2.getOctetString(&len, &p);
    ITSAPP_LOG("getOctetString of %d chars ", len);
    for (unsigned int i = 0; i < len; i++)
        ITSAPP_LOG("0x%02x ", p[i]);
    return R"({ "handle_getIdFromOctetStr": "true"})";
}

std::string MmitssControllerDaemon::handle_dispReqList(int argc, char* argv[])
{
    if (not m_mmitssEnabled)
        return R"({"handle_dispReqList":{"enabled":false}})";
    NOTUSED(argc);
    NOTUSED(argv);
    Poco::DynamicStruct cmd;
    cmd["command"] = "dispReqList";

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_req",
        "/com/siemens/c2x/mmitss_prio_req", "com.siemens.c2x.mmitss_prio_req.Service", "Command",
        "ss", "json", cmd.toString().c_str());

    const char* buf = nullptr;
    if (reply.read("s", &buf) < 0)
    {
        ITSAPP_WRN("Error reading response from mmitss_prio_req call");
        return R"({ "handle_dispReqList": "failed"})";
    }

    std::string reqlist{buf};
    Poco::DynamicStruct res;
    res["handle_dispReqList"] = reqlist;

    return res.toString();
}

std::string MmitssControllerDaemon::handle_respGrant(int argc, char* argv[])
{
    if (not m_mmitssEnabled)
        return R"({"handle_respGrant":{"enabled":false}})";
    if (argc != 2)
        return R"({ "handle_respGrant":{ "error":"invalid args" } })";
    std::string vehId{argv[0]};
    std::string grant{argv[1]};

    Poco::DynamicStruct cmd;
    cmd["command"] = "grantedStatus";
    cmd["vehicle_id"] = vehId;
    cmd["request_granted"] = grant;

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_req",
        "/com/siemens/c2x/mmitss_prio_req", "com.siemens.c2x.mmitss_prio_req.Service", "Command",
        "ss", "json", cmd.toString().c_str());

    return R"({ "handle_respGrant": "true"})";
}

std::string MmitssControllerDaemon::handle_sendSRM(int argc, char* argv[])
{
    if (not m_mmitssEnabled)
        return R"({"handle_sendSRM":{"enabled":false}})";
    if (argc != 8)
        return R"({ "handle_sendSRM":{ "error":"invalid args" } })";
    std::string vehId{argv[0]};
    std::string name{argv[5]};

    Poco::DynamicStruct cmd;
    cmd["command"] = "sendSRM";
    cmd["vehId"] = vehId;
    cmd["type"] = std::stoi(argv[1], nullptr, 10);
    cmd["inLane"] = std::stoi(argv[2], nullptr, 10);
    cmd["outLane"] = std::stoi(argv[3], nullptr, 10);
    cmd["eta"] = std::stoi(argv[4], nullptr, 10);
    cmd["name"] = name;

    std::string rsuID{argv[6]};
    std::string intersectionID{argv[7]};

    requestPriority(
        rsuID, intersectionID, name); // won't be done in requestor for this testing mode

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_req",
        "/com/siemens/c2x/mmitss_prio_req", "com.siemens.c2x.mmitss_prio_req.Service", "Command",
        "ss", "json", cmd.toString().c_str());

    return R"({ "handle_sendSRM": "true"})";
}

std::string MmitssControllerDaemon::handle_changeRadarLaneOrder(int argc, char* argv[])
{
    if (argc != 1)
        return R"({ "handle_changeRadarLaneOrder":{ "error":"invalid args" } })";
    int increased = std::stoi(argv[0], nullptr, 10);
    Poco::DynamicStruct cmd;
    cmd["command"] = "changeRadarLaneOrder";
    cmd["order"] = increased;

    ITSAPP_TRACE("%s", cmd.toString().c_str());

    siekdbus::Message reply;
    siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_perf_observer",
        "/com/siemens/c2x/mmitss_perf_observer", "com.siemens.c2x.mmitss_perf_observer.Service",
        "Command", "ss", "json", cmd.toString().c_str());

    return R"({ "handle_changeRadarLaneOrder": "true"})";
}

std::string MmitssControllerDaemon::handle_enableGrantRequest(int argc, char* argv[])
{
    if (argc != 1)
        return R"({ "handle_enableGrantRequest":{ "error":"invalid args" } })";
    bool to_enable = (std::stoi(argv[0], nullptr, 10) > 0) ? true : false;

    ITSAPP_VERIFY(database().write(MMITSS_REQ_SERV_NEED_GRANT_KEY.c_str(), to_enable));

    return R"({ "handle_enableGrantRequest": "true"})";
}

std::string MmitssControllerDaemon::handle_requestPriority(int argc, char* argv[])
{
    if (argc != 3)
        return R"({ "handle_requestPriority":{ "error":"invalid args" } })";

    std::string vehId = argv[0];
    std::string intersectionID = argv[1];
    std::string rsuID = argv[2];
    requestPriority(rsuID, intersectionID, vehId);
    return R"({ "handle_requestPriority": "true"})";
}

void MmitssControllerDaemon::requestPriority(
    std::string rsuID, std::string intersectionID, std::string name)
{
    Poco::DynamicStruct msg;
    msg["command"] = "priority_request";
    msg["rsu_host"] = rsuID;
    msg["intersection_id"] = intersectionID;
    msg["vehicle_id"] = name;

    ITSAPP_LOG("%s", msg.toString().c_str());
    std::string current = timeUtil::getCurrentTimeStampInMS();
    ITSAPP_LOG("current time in ms: %s", current.c_str());

    ITSAPP_VERIFY(
        getSignal("SendpriorityMessageToMmitss")->emit(getVTable(), 1, msg.toString().c_str()));
}

std::string MmitssControllerDaemon::handle_status(int argc, char* argv[])
{
    NOTUSED(argc);
    NOTUSED(argv);
    std::string verbose;
    std::string status;
    verbose += "--== Service Status ==--\n";
    m_serviceCtrl.getStatus(m_opMode, status);
    verbose += "--== Configuration Status ==--\n";
    verbose += m_mmitssCfg.getStatus();
    return verbose;
}

std::string MmitssControllerDaemon::handle_json(int argc, char* argv[])
{
    Poco::DynamicStruct res;
    try
    {
        if (argc >= 1)
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
                res["error"] = "No command attr";
            }
            else
            {
                auto cmd = json->get("command").toString();
                ITSAPP_LOG("handle_json: command: %s", cmd.c_str());
                if (cmd == "status")
                {
                    Poco::DynamicStruct status;
                    status["run_status"] = m_status;
                    status["service_status"] = m_serviceCtrl.getJsonStatus();
                    status["signalPlan"] = m_mmitssCfg.getSignalPlan();
                    status["config_status"] = m_mmitssCfg.getStatusJson();
                    res["status"] = status;
                }
                else if (cmd == "requestPriority") // sent from mmitssPriorityRequestServer
                {
                    std::string rsu{""}, intersection{""}, veh{""};
                    if (json->has("rsu_host"))
                    {
                        rsu = json->get("rsu_host").toString();
                    }
                    if (json->has("intersection_id"))
                    {
                        intersection = json->get("intersection_id").toString();
                    }
                    if (json->has("vehicle_id"))
                    {
                        veh = json->get("vehicle_id").toString();
                    }
                    ITSAPP_LOG("request Priority for %s at %s from %s", veh.c_str(),
                        intersection.c_str(), rsu.c_str());
                    requestPriority(rsu, intersection, veh);
                }
                else if (cmd == "curConfigFiles")
                {
                    res = m_mmitssCfg.getCurrentConfigFilesJson();
                }
                else if ((cmd == "configFileStatus") && (json->has("filename")))
                {
                    res["configFileStatus"] =
                        m_mmitssCfg.getConfigFileStatusJson(json->get("filename").toString());
                }
                else if ((cmd == "readConfigFile") && (json->has("filename")))
                {
                    std::string cfgFile =
                        m_mmitssCfg.readConfigFile(json->get("filename").toString());
                    res["readConfigFile"] = Base64::encode(cfgFile);
                }
                else if ((cmd == "configFile") && (json->has("filename") && (json->has("content"))))
                {
                    auto filename = json->get("filename").toString();
                    auto contentEncoded = json->get("content").toString();
                    if (contentEncoded.size() <= MmitssConfig::MAX_CONFIG_FILE_SIZE)
                    {
                        ITSAPP_LOG("Write MMITSS config file %s", filename.c_str());
                        auto content = Base64::decode(json->get("content").toString());
                        if (m_mmitssCfg.writeConfigFile(getConfigDir() + filename, content))
                        {
                            m_configDirty = true;
                            res["configFile"] = true;
                        }
                        else
                        {
                            std::string result = "Couldn't write file. Expected:";
                            for (const auto& cfg : m_mmitssCfg.getUserConfigFiles())
                                result += cfg + std::string(",");
                            res["error"] = result;
                        }
                    }
                    else
                    {
                        res["error"] = "File size limit exceeded";
                    }
                }
                else
                {
                    res["error"] = "Invalid command: " + cmd;
                }
            }
        }
        else
        {
            ITSAPP_DBG("handle_json error for %s", argv[0]);
            res["error"] = "invalid args";
        }
    }
    catch (const Poco::JSON::JSONException& ex)
    {
        res["error"] = std::string("ERROR: ") + ex.message();
    }
    catch (const std::exception& ex)
    {
        ITSAPP_WRN("JSON Processing: %s", ex.what());
        res["error"] = std::string("ERROR: ") + std::string(ex.what());
    }
    return res.toString();
}

std::string MmitssControllerDaemon::handle_start(int argc, char* argv[])
{
    std::string result;
    if (argc >= 1)
    {
        if (m_serviceCtrl.startService(std::string(argv[0])))
            result += "SUCCESS";
        else
            result += "ERROR";
    }
    else
    {
        if (startMmitss())
            result = "SUCCESS";
        else
            result = "FAILED";
    }
    return result;
}

std::string MmitssControllerDaemon::handle_stop(int argc, char* argv[])
{
    std::string result;
    if (argc >= 1)
    {
        if (m_serviceCtrl.stopService(std::string(argv[0])))
            result += "SUCCESS";
        else
            result += "ERROR";
    }
    else
    {
        result = "invalid args";
    }
    return result;
}

bool MmitssControllerDaemon::startMmitss()
{
    MmitssOperatingMode mode = isMmitssSignalPlanActive() ? m_opMode : Passive;
    bool ret = m_serviceCtrl.start(mode);
    udpInit();
    m_perfData.start();
    ITSAPP_TRACE("Start MMITSS services(%d)", ret);
    return ret;
}

bool MmitssControllerDaemon::stopMmitss()
{
    ITSAPP_LOG("Stop MMITSS services");
    udpClose();
    m_perfData.stop();
    return m_serviceCtrl.stop();
}

void MmitssControllerDaemon::onSpatMapConfigChange(siekdbus::Message& in)
{
    int map_valid = 0;
    int spat_valid = 0;
    auto r = in.read(SPATMAP_CFG_SIGNAL_SIGNATURE, &map_valid, &spat_valid);
    if (r >= 0)
    {
        ITSAPP_LOG("SPAT/MAP config change: map_valid(%d), spat_valid(%d)", map_valid, spat_valid);
        m_configDirty = true;
    }
    else
    {
        ITSAPP_WRN("Couldn't get SPAT/MAP signal data: %s", in.interface());
    }
}

void MmitssControllerDaemon::readConfig()
{
    ITSAPP_DBG("read config");
    ITSAPP_VERIFY(database().read(
        (MMITSS_CTRL_DB_ROOT_KEY + std::string("/mmitss_enable")).c_str(), m_mmitssEnabled));
    ITSAPP_VERIFY(database().read(
        (MMITSS_CTRL_DB_ROOT_KEY + std::string("/generate_nmap")).c_str(), m_autoGenerateMap));

    // read map
    bool map_valid = false;
    ITSAPP_VERIFY(
        database().read((SPAT_MAP_DB_ROOT_KEY + std::string("/map_valid")).c_str(), map_valid));
    // read signal plan list
    its::DatabaseListData sigPlanListData;
    ITSAPP_VERIFY(database().getListData(MMITSS_CTRL_SIG_PLN_LST_KEY.c_str(), sigPlanListData));
    try
    {
        for (auto const& it : sigPlanListData)
            m_sigPlanList.emplace_back(std::stoi(it->content.at("plan_id")));
    }
    catch (std::exception& ex)
    {
        ITSAPP_WRN("Couldn't read signalPlanList: %s", ex.what());
        m_sigPlanList.clear();
    }
    int sigPlan = -1;
    ITSAPP_VERIFY(
        database().read((ASC_DB_OIDS_KEY + std::string("/cur_timing_plan")).c_str(), sigPlan));
    ITSAPP_LOG("m_curSignalPlan is : %d", m_curSignalPlan);
    if (sigPlan != m_curSignalPlan)
    {
        m_curSignalPlan = sigPlan;
        ITSAPP_LOG("m_curSignalPlan is changed to : %d", m_curSignalPlan);
    }
    // read op mode
    std::string mode;
    ITSAPP_VERIFY(database().read((MMITSS_CTRL_DB_ROOT_KEY + std::string("/mode")).c_str(), mode));
    m_opMode = Passive;
    if (mode == "Priority")
        m_opMode = Priority;
    if (mode == "ISIG")
        m_opMode = ISIG;

    if (m_autoGenerateMap)
    {
        if (map_valid)
            m_spatMap.readMap();
        else
            m_spatMap.clearMap();
    }

    if (m_mmitssCfg.updateConfig(m_autoGenerateMap ? m_spatMap.getMap() : nullptr))
    {
        ITSAPP_LOG("MMITSS configured")
    }
}

void MmitssControllerDaemon::updateCurSignalPlan()
{
    int sigPlan = -1;
    auto wasActive = isMmitssSignalPlanActive();
    ITSAPP_VERIFY(
        database().read((ASC_DB_OIDS_KEY + std::string("/cur_timing_plan")).c_str(), sigPlan));
    if (sigPlan != m_curSignalPlan)
    {
        m_curSignalPlan = sigPlan;
        if ((m_mmitssEnabled) && (m_opMode != Passive))
        {
            if (wasActive != isMmitssSignalPlanActive())
                m_configDirty = true;
        }
    }
}

bool MmitssControllerDaemon::isMmitssSignalPlanActive() const
{
    return (std::find(m_sigPlanList.begin(), m_sigPlanList.end(), m_curSignalPlan) !=
            m_sigPlanList.end());
}

inline void MmitssControllerDaemon::udpBsmXmit(const void* data, size_t size)
{
    if (m_bsmUdpSocket.sendBytes(data, size) > 0)
        m_bsmUdpXmit++;
    else
        m_bsmUdpError++;
}

inline void MmitssControllerDaemon::udpSrmXmit(const void* data, size_t size)
{
    if (m_srmUdpSocket.sendBytes(data, size) > 0)
        m_srmUdpXmit++;
    else
        m_srmUdpError++;
}

inline void MmitssControllerDaemon::udpInit()
{
    m_bsmUdpSocket.connect(BSM_SOCKET);
    m_srmUdpSocket.connect(SRM_SOCKET);
}

inline void MmitssControllerDaemon::udpClose()
{
    m_bsmUdpSocket.close();
    m_srmUdpSocket.close();
}

bool MmitssControllerDaemon::onStartup()
{
    ITSAPP_TRACE("Running MmitssControllerDaemon startup");

    // register for spat/map signal
    bool ret = true;
    m_spatMapSignal = std::make_unique<siekdbus::Signal>(SPATMAP_CFG_SIGNAL);
    m_spatMapSignal->setSignature(SPATMAP_CFG_SIGNAL_SIGNATURE);
    if (m_spatMapSignalConnection.connected())
        m_spatMapSignalConnection.disconnect();
    auto matchRule = std::string(SPATMAP_CFG_SIGNAL_MATCH);
    m_spatMapSignalConnection = m_spatMapSignal->connectHandler(
        matchRule.c_str(), sigc::mem_fun(*this, &MmitssControllerDaemon::onSpatMapConfigChange));
    if (!m_spatMapSignalConnection.connected())
    {
        ITSAPP_ERROR("registerSignal failed.");
        ret = false;
    }

    m_configDirty = true;

    return ret;
}

void MmitssControllerDaemon::onShutdown()
{
    if (m_spatMapSignalConnection.connected())
        m_spatMapSignalConnection.disconnect();

    stopMmitss();
}

void MmitssControllerDaemon::onExit()
{
    // cleanup
}

void MmitssControllerDaemon::onSystemTick()
{
    if (m_configDirty)
    {
        if (not m_serviceCtrl.completeStop())
        {
            ITSAPP_TRACE("MMITSS services not stopped completely.");
            stopMmitss();
        }
        readConfig();
        ITSAPP_TRACE("config read.");
        if (m_mmitssEnabled)
        {
            if (m_mmitssCfg.isValid())
            {
                ITSAPP_TRACE("start MMITSS services.");
                if (!startMmitss())
                {
                    ITSAPP_WRN("Couldn't start MMITSS services");
                    m_status = "Failed to start services";
                }
                else
                {
                    ITSAPP_TRACE("MMITSS services started");
                    m_status = "Running";
                }
            }
            else
            {
                m_mmitssEnabled = false;
                m_status = "Configuration invalid";
                ITSAPP_VERIFY(database().write(
                    (MMITSS_CTRL_DB_ROOT_KEY + std::string("/mmitss_enable")).c_str(),
                    m_mmitssEnabled));
                ITSAPP_WRN("Couldn't start MMITSS services - config invalid");
            }
        }
        else
        {
            m_status = "Offline";
        }
        m_configDirty = false;
        sielib::property_free();
    }
}
static const siekdbus::SignalData signals[] = {
    {"SendpriorityMessageToMmitss", "s"}, {nullptr, nullptr}};

static const its::FacilityInfo theFacilityInfo = {.logId = "mmitssctl",
    .name = "mmitssctl",
    .description = "MMITSS Control",
    .uuid = "cdeb9d6c-d399-4e3d-8fd2-9aad93c3b636",

    .versionMajor = 1,
    .versionMinor = 1,

    .busName = "com.siemens.c2x.mmitssctl",
    .busPath = "/com/siemens/c2x/mmitssctl",
    .busInterface = "com.siemens.c2x.mmitssctl.Service",

    .busMethods = nullptr,
    .busProperties = nullptr,
    .busSignals = signals,

    .memoryType = CACHEMEM_SIZE_TYPE::SMALL,
    .pipeType = sieipc::PipeType::SMALL,
    .idleDelay = 5000,
    .gpsActivate = false,

    // TODO: fix we're not running as root
    //       should be APPLICATION_UID, APPLICATION_GID then
    .facilityUser = 0,
    .facilityGroup = 0,
    .facilityCapabilities = 0};

} // namespace MmitssCtrl
} // namespace WaveApp

static void showHelp()
{
    fprintf(stdout, "mmitssctl [-d]\n");
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
        return its::facility_main(verbose, WaveApp::MmitssCtrl::theFacilityInfo,
            [] { return new WaveApp::MmitssCtrl::MmitssControllerDaemon(); });
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed running spat-map-daemon: " << ex.what() << std::endl;
    }
    return 0;
}
