#include "stdheader.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"
#include "facilityLayer.hpp"
#include "siekdbus-vtable.hpp"
#include "xferHandler.hpp"

namespace WaveApp
{
namespace Mmitss
{

// make sure these strings are same as signals defined in isgDaemon.cpp
static const char* ISG_SIGNAL = "SendIsgMessageToMmitss";
static const char* ISG_SIGNAL_SIGNATURE = "s";
static const char* ISG_SIGNAL_MATCH = "type='signal',"
                                      "sender='com.siemens.c2x.isg',"
                                      "interface='com.siemens.c2x.isg.Service',"
                                      "member='SendIsgMessageToMmitss'";

// make sure these strings are same as signals defined in mmitssPriorityRequestServerApp.cpp
static const char* PREQ_SIGNAL = "SendpriorityMessageToMmitss";
static const char* PREQ_SIGNAL_SIGNATURE = "s";
static const char* PREQ_SIGNAL_MATCH = "type='signal',"
                                       "sender='com.siemens.c2x.mmitssctl',"
                                       "interface='com.siemens.c2x.mmitssctl.Service',"
                                       "member='SendpriorityMessageToMmitss'";

static const std::string CLIENTCANHANDLEPRIOREQUEST = "MMITSS::canHandlePriorityRequest";

class __local mmitssCommand : public its::XferCommandHandler
{
    using inherited = its::XferCommandHandler;

  public:
    mmitssCommand() = default;
    virtual ~mmitssCommand() = default;

    bool registerDbusSignal();
    void unregisterDbusSignal();

    virtual const char* getCommand() const override;
    virtual bool onCommand(its::XferConnection& conn, const char* message) override;
    virtual bool needRegistration() const override { return true; }

    void onISGSignal(siekdbus::Message& in);
    void onPrioReqSignal(siekdbus::Message& in);

  private:
    bool reportDetectorEvent(const std::string& isgMessage);
    bool reportGrantedStatus(const std::string& isgMessage);

    sigc::connection m_sigConnectionFromIsg;
    std::unique_ptr<siekdbus::Signal> m_isgSignal;

    sigc::connection m_sigConnectionFromPrioReq;
    std::unique_ptr<siekdbus::Signal> m_prioReqSignal;
};

const char* mmitssCommand::getCommand() const
{
    return "MMITSS";
}

bool mmitssCommand::onCommand(its::XferConnection& conn, const char* message)
{
    if (!conn.hasCommandRegistration())
    {
        ITSAPP_WRN("mmitssCommand: No permissions");
        return conn.xmitError(*this, its::NO_PERMISSION, "No command registration");
    }

    ITSAPP_LOG("mmitssCommand::onCommand");
    if (!isempty(message))
        ITSAPP_LOG("message received: %s", message);

    if (!isempty(message) && (message[0] == '{'))
    {
        Poco::Dynamic::Var parsedResult = Poco::JSON::Parser().parse(message);
        Poco::JSON::Object::Ptr json = parsedResult.extract<Poco::JSON::Object::Ptr>();

        if (json->has("error"))
            return conn.xmitError(
                *this, its::INTERNAL_ERROR, json->get("error").toString().c_str());

        try
        {
            std::string cmd = json->get("command").toString();
            if (cmd == "forwardedISG")
            {
                if (reportDetectorEvent(message))
                    return conn.xmitFrame(R"({"success":{"MMITSS":{}}})");
                else
                    return conn.xmitError(*this, its::INTERNAL_ERROR, "Internal error");
            }
            else if (cmd == "priority_reply")
            {
                if (reportGrantedStatus(message))
                    return conn.xmitFrame(R"({"success":{"MMITSS":{}}})");
                else
                    return conn.xmitError(*this, its::INTERNAL_ERROR, "Internal error");
            }
            else if (cmd == "canHandlePriorityRequest")
            {
                conn.setContextData(CLIENTCANHANDLEPRIOREQUEST, "true");
                ITSAPP_LOG("Client canHandlePriorityRequest");
                return conn.xmitFrame(R"({"success":true})");
            }
            else if (cmd == "cancel_canHandlePriorityRequest")
            {
                conn.setContextData(CLIENTCANHANDLEPRIOREQUEST, "false");
                ITSAPP_LOG("Client canHandlePriorityRequest canceled");
                return conn.xmitFrame(R"({"success":true})");
            }
        }
        catch (Poco::InvalidAccessException&)
        {
            ITSAPP_LOG("value is missing in json message");
            return conn.xmitError(*this, its::INTERNAL_ERROR, "Internal error");
        }
    }
    return conn.xmitError(*this, its::INTERNAL_ERROR, "Internal error");
}

bool mmitssCommand::registerDbusSignal()
{
    ITSAPP_LOG("MMITSS: registerSignal for ISG");
    m_isgSignal = std::make_unique<siekdbus::Signal>(ISG_SIGNAL);
    m_isgSignal->setSignature(ISG_SIGNAL_SIGNATURE);
    if (m_sigConnectionFromIsg.connected())
        m_sigConnectionFromIsg.disconnect();
    m_sigConnectionFromIsg = m_isgSignal->connectHandler(
        ISG_SIGNAL_MATCH, sigc::mem_fun(*this, &mmitssCommand::onISGSignal));

    ITSAPP_LOG("MMITSS: registerSignal for PrioReq");
    m_prioReqSignal = std::make_unique<siekdbus::Signal>(PREQ_SIGNAL);
    m_prioReqSignal->setSignature(PREQ_SIGNAL_SIGNATURE);
    if (m_sigConnectionFromPrioReq.connected())
        m_sigConnectionFromPrioReq.disconnect();
    m_sigConnectionFromPrioReq = m_prioReqSignal->connectHandler(
        PREQ_SIGNAL_MATCH, sigc::mem_fun(*this, &mmitssCommand::onPrioReqSignal));

    bool ok = true;
    if (!m_sigConnectionFromIsg.connected())
    {
        ITSAPP_ERROR("registerSignal for ISG failed.");
        ok = false;
    }
    else if (!m_sigConnectionFromPrioReq.connected())
    {
        ITSAPP_ERROR("registerSignal for PrioReq failed.");
        ok = false;
    }
    else
    {
        ITSAPP_LOG("registerSignal done");
    }
    return ok;
}

/**
 * unregister the dbus signal channel for mmitss daemon
 */
void mmitssCommand::unregisterDbusSignal()
{
    if (m_sigConnectionFromIsg.connected())
        m_sigConnectionFromIsg.disconnect();
    if (m_sigConnectionFromPrioReq.connected())
        m_sigConnectionFromPrioReq.disconnect();
}

extern "C" void mmitssCommand::onISGSignal(siekdbus::Message& in)
{
    ITSAPP_LOG("onIsgSignal, %s", in.signature());
    const char* message = nullptr;

    if (in.read(ISG_SIGNAL_SIGNATURE, &message) < 0)
    {
        ITSAPP_WRN("ERROR: onIsgSignal, cannot read message");
        return;
    }
    ITSAPP_LOG("Isg message received %s", message);

    reportDetectorEvent(message);
}

extern "C" void mmitssCommand::onPrioReqSignal(siekdbus::Message& in)
{
    ITSAPP_LOG("onPrioReqSignal, %s", in.signature());
    const char* message = nullptr;

    if (in.read(PREQ_SIGNAL_SIGNATURE, &message) < 0)
    {
        ITSAPP_WRN("ERROR: onPrioReqSignal, cannot read message");
        return;
    }
    ITSAPP_LOG("PrioReq message received %s", message);

    for (auto& conn : getCurrentConnections())
    {
        if (conn.getContextData(CLIENTCANHANDLEPRIOREQUEST) == "true")
        {
            ITSAPP_LOG("   * sending %s to %s", message, conn.getPeer().c_str());
            conn.xmitFrame(message);
        }
        else
        {
            ITSAPP_LOG("   * cannot send to %s because it cannot HandlePriorityRequest",
                conn.getPeer().c_str());
        }
    }
}

bool mmitssCommand::reportDetectorEvent(const std::string& isgMessage)
{
    Poco::DynamicStruct event;
    event["command"] = "detectorEvent";

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(isgMessage.c_str());
    Poco::JSON::Object::Ptr json = result.extract<Poco::JSON::Object::Ptr>();
    if (json->has("payload"))
    {
        Poco::Dynamic::Var payload = json->get("payload");
        ITSAPP_LOG("payload: %s", payload.toString().c_str());

        parser.reset();
        auto subObject = parser.parse(payload.toString()).extract<Poco::JSON::Object::Ptr>();
        if (subObject->has("validity"))
        {
            if (subObject->get("validity").toString() == "false")
                return false;
        }
        if (subObject->has("speed"))
        {
            event["speed"] = std::stod(subObject->get("speed").toString());
        }
        if (subObject->has("lane_id"))
        {
            event["lane_id"] = std::stoi(subObject->get("lane_id").toString(), nullptr, 10);
        }
        if (subObject->has("distance"))
        {
            event["distance"] = std::stod(subObject->get("distance").toString());
        }
        if (subObject->has("heading"))
        {
            event["heading"] = std::stod(subObject->get("heading").toString());
        }
        ITSAPP_LOG("received: %s", event.toString().c_str());
    }

    siekdbus::Message reply;
    return siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_perf_observer",
        "/com/siemens/c2x/mmitss_perf_observer", "com.siemens.c2x.mmitss_perf_observer.Service",
        "Command", "ss", "json", event.toString().c_str());
}

bool mmitssCommand::reportGrantedStatus(const std::string& isgMessage)
{
    Poco::JSON::Object status;
    status.set("command", "grantedStatus");

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(isgMessage.c_str());
    Poco::JSON::Object::Ptr json = result.extract<Poco::JSON::Object::Ptr>();
    if (json->has("vehicle_id"))
    {
        std::string vehicleId = json->get("vehicle_id").toString();
        ITSAPP_LOG("vehicle_id: %s", vehicleId.c_str());
        status.set("vehicle_id", json->get("vehicle_id"));
    }
    if (json->has("request_granted"))
    {
        std::string granted = json->get("request_granted").toString();
        ITSAPP_LOG("vehicle_id: %s", granted.c_str());
        status.set("request_granted", json->get("request_granted"));
    }

    std::ostringstream buf;
    status.stringify(buf, 0, 0);
    siekdbus::Message reply;
    bool ok = siekdbus::Method::call(reply, "com.siemens.c2x.mmitss_prio_req",
        "/com/siemens/c2x/mmitss_prio_req", "com.siemens.c2x.mmitss_prio_req.Service", "Command",
        "ss", "json", buf.str().c_str());
    if (ok)
    {
        const char* msg = nullptr;
        if (reply.read("s", &msg) > 0)
        {
            ITSAPP_TRACE("reply got from mmitss_prio_req.Service %s", msg);
        }
    }
    else
    {
        ITSAPP_WRN("error calling to mmitss_prio_req.Service");
    }

    return ok;
}

} // namespace Mmitss
} // namespace WaveApp

extern "C" its::XferCommandHandler* xferMakeInstance(void)
{
    auto handler = new WaveApp::Mmitss::mmitssCommand();

    if (handler->registerDbusSignal())
        return handler;
    else
        return nullptr;
}

extern "C" void xferFreeInstance(its::XferCommandHandler* handler)
{
    static_cast<WaveApp::Mmitss::mmitssCommand*>(handler)->unregisterDbusSignal();
    delete handler;
}
