#pragma once

#include "facilityWave.hpp"

namespace WaveApp
{

// TODO(unknown) add type specific context data
// clang-format off
struct MapXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct SpatXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct TimXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct RsaXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct SrmXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct SsmXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct BsmXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct PdmXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
struct PsmXmitContext : public its::ItsFacilityApplicationWave::XmitContext {};
// clang-format on

/// PSID in P-format
enum ProviderServiceIdentifier_THEA
{
    PSID_THEA_SSM = 0xe0000015,
    PSID_THEA_SRM = 0xe0000016
};

/** Base class for Wave Facility applications using the SAE message set
 *
 *  Provides additional helper methods for SAE message handling
 *  like sending and receiving MAP, SPAT etc.
 */
class ItsFacilityApplicationWaveSae : public its::ItsFacilityApplicationWave
{
  public:
    ItsFacilityApplicationWaveSae();
    virtual ~ItsFacilityApplicationWaveSae();
    ItsFacilityApplicationWaveSae(const ItsFacilityApplicationWaveSae&) = delete;
    ItsFacilityApplicationWaveSae& operator=(const ItsFacilityApplicationWaveSae&) = delete;

    // Receive
    virtual bool onNetRxWSMP(Int32u psid, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt) override;

    virtual bool onNetRxMap(const saeMapData& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxSpat(const saeSPAT& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxBsm(
        const saeBasicSafetyMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxTim(
        const saeTravelerInformation& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxRsa(
        const saeRoadSideAlert& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxSrm(
        const saeSignalRequestMessage& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);
    virtual bool onNetRxPdm(
        const saeProbeDataManagement& msg, ieee1609::IEEE1609_IND_Packet::PacketPtr pkt);

    /// send to Networklayer
    bool xmitMAP(saeMapData& msg, MapXmitContext& ctx);
    bool xmitSPAT(saeSPAT& msg, SpatXmitContext& ctx);
    bool xmitTIM(saeTravelerInformation& msg, TimXmitContext& ctx);
    bool xmitRSA(saeRoadSideAlert& msg, RsaXmitContext& ctx);
    bool xmitSRM(saeSignalRequestMessage& msg, SrmXmitContext& ctx);
    bool xmitSSM(saeSignalStatusMessage& msg, SsmXmitContext& ctx);
    bool xmitBSM(saeBasicSafetyMessage& msg, BsmXmitContext& ctx);
    bool xmitPDM(saeProbeDataManagement& msg, PdmXmitContext& ctx);
    bool xmitPSM(saePersonalSafetyMessage& msg, PsmXmitContext& ctx);

    // set default context values
    void readDefaults(XmitContext& ctx);

    void readDefault(MapXmitContext& ctx);
    void readDefault(SpatXmitContext& ctx);
    void readDefault(TimXmitContext& ctx);
    void readDefault(RsaXmitContext& ctx);
    void readDefault(SrmXmitContext& ctx);
    void readDefault(SsmXmitContext& ctx);
    void readDefault(BsmXmitContext& ctx);
    void readDefault(PdmXmitContext& ctx);
    void readDefault(PsmXmitContext& ctx);

    bool doXmit(const asntype& msg, XmitContext& ctx);
};

} // namespace WaveApp
