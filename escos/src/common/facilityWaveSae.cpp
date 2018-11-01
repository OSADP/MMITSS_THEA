#include "facilityWaveSae.hpp"

#include "facilityLayer.hpp"
#include "facilityDatabaseAdapter.hpp"
#include "facilityWave.hpp"

namespace WaveApp
{

ItsFacilityApplicationWaveSae::ItsFacilityApplicationWaveSae() {}

ItsFacilityApplicationWaveSae::~ItsFacilityApplicationWaveSae() {}

bool ItsFacilityApplicationWaveSae::doXmit(const asntype& msg, XmitContext& ctx)
{
    try
    {
        // UPER encode C++ BSM object
        its::PayloadBytes payloadBytes;
        if (!toUper(&msg, payloadBytes))
        {
            ITSAPP_ERROR("UPER encode failed: %d", static_cast<int>(ctx.errorCode));
            return false;
        }

        // initiate WSMP transmission
        if (!xmitWSMP(payloadBytes.buffer(), payloadBytes.size(), ctx))
        {
            ITSAPP_ERROR("XMIT failed: %d", static_cast<int>(ctx.errorCode));
            return false;
        }

        return true;
    }
    catch (const asnException& ex)
    {
        ITSAPP_ERROR("UPER encode exception: %s", toString(ex));
    }
    catch (const std::exception& ex)
    {
        ITSAPP_ERROR("XMIT exception: %s", ex.what());
    }
    return false;
}

bool ItsFacilityApplicationWaveSae::xmitMAP(saeMapData& msg, MapXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saemapData__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitMAP failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitSPAT(saeSPAT& msg, SpatXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saesignalPhaseAndTimingMessage__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitSPAT failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitTIM(saeTravelerInformation& msg, TimXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saetravelerInformation__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitTIM failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitRSA(saeRoadSideAlert& msg, RsaXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saeroadSideAlert__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitRSA failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitBSM(saeBasicSafetyMessage& msg, BsmXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saebasicSafetyMessage__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitBSM failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitSRM(saeSignalRequestMessage& msg, SrmXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saesignalRequestMessage__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitSRM failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitSSM(saeSignalStatusMessage& msg, SsmXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saesignalStatusMessage__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitSSM failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitPDM(saeProbeDataManagement& msg, PdmXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saeprobeDataManagement__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitPDM failed");
    return ok;
}

bool ItsFacilityApplicationWaveSae::xmitPSM(saePersonalSafetyMessage& msg, PsmXmitContext& ctx)
{
    // wrap message in frame
    saeMessageFrame frame;
    frame.messageId = saepersonalSafetyMessage__V;
    frame.value = &msg;
    bool ok = xmitWSMP(&frame, ctx);
    frame.value = nullptr;
    if (!ok)
        ITSAPP_WRN("xmitPSM failed");
    return ok;
}

/// callbacks to Networklayer
bool ItsFacilityApplicationWaveSae::onNetRxWSMP(
    Int32u psid, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(psid);
    if (!pkt || pkt->getPayloadLength() == 0)
    {
        ITSAPP_WRN("Empty network packet");
        return false;
    }

    asnMemoryStream is(
        const_cast<asnbyte*>(pkt->getPayload()), pkt->getPayloadLength(), asnFSTREAM_READ);

    try
    {
        saeMessageFrame frame;
        frame.UPERdecode(getAsnContext(), &is);

        if (frame.value == nullptr)
        {
            ITSAPP_WRN("Invalid message value");
            return false;
        }

        switch (frame.messageId)
        {
            case saemapData__V:
                onNetRxMap(*static_cast<saeMapData*>(frame.value), pkt);
                break;
            case saesignalPhaseAndTimingMessage__V:
                onNetRxSpat(*static_cast<saeSPAT*>(frame.value), pkt);
                break;
            case saetravelerInformation__V:
                onNetRxTim(*static_cast<saeTravelerInformation*>(frame.value), pkt);
                break;
            case saeroadSideAlert__V:
                onNetRxRsa(*static_cast<saeRoadSideAlert*>(frame.value), pkt);
                break;
            case saebasicSafetyMessage__V:
                onNetRxBsm(*static_cast<saeBasicSafetyMessage*>(frame.value), pkt);
                break;
            case saesignalRequestMessage__V:
                onNetRxSrm(*static_cast<saeSignalRequestMessage*>(frame.value), pkt);
                break;
            case saeprobeDataManagement__V:
                onNetRxPdm(*static_cast<saeProbeDataManagement*>(frame.value), pkt);
                break;
            default:
                ITSAPP_WRN("Unhandled message type: %d", frame.messageId);
                return false;
        }
    }
    catch (const asnException& ex)
    {
        ITSAPP_WRN("Invalid MessageFrame encoding: %s", toString(ex));
        return false;
    }

    return true;
}

bool ItsFacilityApplicationWaveSae::onNetRxMap(
    const saeMapData& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxSpat(
    const saeSPAT& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxBsm(
    const saeBasicSafetyMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxTim(
    const saeTravelerInformation& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxRsa(
    const saeRoadSideAlert& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxSrm(
    const saeSignalRequestMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

bool ItsFacilityApplicationWaveSae::onNetRxPdm(
    const saeProbeDataManagement& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt)
{
    NOTUSED(msg);
    NOTUSED(pkt);
    return false;
}

void ItsFacilityApplicationWaveSae::readDefault(MapXmitContext& ctx)
{
    ctx.dot3.channel = its::ItsChannel::SCH1;
    ctx.dot3.psid = its::DOT3_P_PSID_MAPCV;
    makeDot3(ctx.dot3, its::DOT3_P_PSID_MAPCV);
}

void ItsFacilityApplicationWaveSae::readDefault(SpatXmitContext& ctx)
{
    ctx.dot3.channel = its::ItsChannel::SCH1;
    ctx.dot3.psid = its::DOT3_P_PSID_ISSA;
    makeDot3(ctx.dot3, its::DOT3_P_PSID_ISSA);
}

void ItsFacilityApplicationWaveSae::readDefault(TimXmitContext& ctx)
{
    ctx.dot3.channel = its::ItsChannel::CCH;
    ctx.dot3.psid = its::DOT3_P_PSID_TIRS;
    makeDot3(ctx.dot3, its::DOT3_P_PSID_TIRS);
}

void ItsFacilityApplicationWaveSae::readDefault(RsaXmitContext& ctx)
{
    makeDot3(ctx.dot3); // TODO(unknown) set correct PSID
}

void ItsFacilityApplicationWaveSae::readDefault(PdmXmitContext& ctx)
{
    makeDot3(ctx.dot3); // TODO(unknown) set correct PSID
}

void ItsFacilityApplicationWaveSae::readDefault(PsmXmitContext& ctx)
{
    ctx.dot3.channel = its::ItsChannel::SCH3;
    ctx.dot3.psid = its::DOT3_P_PSID_VULS;
    makeDot3(ctx.dot3, its::DOT3_P_PSID_VULS);
}

void ItsFacilityApplicationWaveSae::readDefault(SrmXmitContext& ctx)
{
    ctx.dot3.psid = PSID_THEA_SRM;
    makeDot3(ctx.dot3, PSID_THEA_SRM);
}

void ItsFacilityApplicationWaveSae::readDefault(SsmXmitContext& ctx)
{
    ctx.dot3.psid = PSID_THEA_SSM;
    makeDot3(ctx.dot3, PSID_THEA_SSM);
}

void ItsFacilityApplicationWaveSae::readDefault(BsmXmitContext& ctx)
{
    ctx.dot3.psid = its::DOT3_P_PSID_VVSA;
    makeDot3(ctx.dot3, its::DOT3_P_PSID_VVSA);
}

} // namespace WaveApp
