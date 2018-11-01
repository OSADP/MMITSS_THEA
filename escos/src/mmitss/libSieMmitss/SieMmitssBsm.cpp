/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitssBsm.cpp
 *
 *  Created on: May 15, 2017
 *      Author: M.Venus
 */

#include "stdtypes.h"
#include "Poco/Dynamic/Struct.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"

#include "Brakes.h"
#include "SieMmitssBsm.hpp"
#include "SieMmitss.hpp"

namespace SieMmitss
{

static e_TractionControlState tractionCtrlfromJSON(const Poco::JSON::Object::Ptr& brakes)
{
    auto tractionCtrlStatus = brakes->getValue<std::string>("traction");
    if (tractionCtrlStatus == "unavailable")
        return e_TractionControlState::TracCntrlunavailable;
    else if (tractionCtrlStatus == "off")
        return e_TractionControlState::TracCntrloff;
    else if (tractionCtrlStatus == "on")
        return e_TractionControlState::TracCntrlon;
    else if (tractionCtrlStatus == "engaged")
        return e_TractionControlState::TracCntrlengaged;
    return e_TractionControlState::TracCntrlunavailable;
}

static std::string tractionCtrlToJSON(e_TractionControlState traction)
{
    switch (traction)
    {
        case e_TractionControlState::TracCntrloff:
            return "off";
        case e_TractionControlState::TracCntrlon:
            return "on";
        case e_TractionControlState::TracCntrlengaged:
            return "engaged";
        default:
            break;
    }
    return "unavailable";
}

static e_StabilityControlStatus stabilityCtrlfromJSON(const Poco::JSON::Object::Ptr& brakes)
{
    auto stabilityCtrlStatus = brakes->getValue<std::string>("scs");
    if (stabilityCtrlStatus == "unavailable")
        return e_StabilityControlStatus::Stabilitycntlunavailable;
    else if (stabilityCtrlStatus == "off")
        return e_StabilityControlStatus::Stabilitycntloff1;
    else if (stabilityCtrlStatus == "on")
        return e_StabilityControlStatus::Stabilitycntlon;
    else if (stabilityCtrlStatus == "engaged")
        return e_StabilityControlStatus::Stabilitycntlon;
    return e_StabilityControlStatus::Stabilitycntlunavailable;
}

static std::string stabilityCtrlToJSON(e_StabilityControlStatus stability)
{
    switch (stability)
    {
        case e_StabilityControlStatus::Stabilitycntloff1:
            return "off";
        case e_StabilityControlStatus::Stabilitycntlon:
            return "on";
        default:
            break;
    }
    return "unavailable";
}

static e_AntiLockBrakeStatus antilockfromJSON(const Poco::JSON::Object::Ptr& brakes)
{
    auto antilockStatus = brakes->getValue<std::string>("abs");
    if (antilockStatus == "unavailable")
        return e_AntiLockBrakeStatus::AntiLockunavailable;
    else if (antilockStatus == "off")
        return e_AntiLockBrakeStatus::AntiLockoff;
    else if (antilockStatus == "on")
        return e_AntiLockBrakeStatus::AntiLockon;
    else if (antilockStatus == "engaged")
        return e_AntiLockBrakeStatus::AntiLockengaged;
    return e_AntiLockBrakeStatus::AntiLockunavailable;
}

static std::string antilockToJSON(e_AntiLockBrakeStatus antiLock)
{
    switch (antiLock)
    {
        case e_AntiLockBrakeStatus::AntiLockoff:
            return "off";
        case e_AntiLockBrakeStatus::AntiLockon:
            return "on";
        case e_AntiLockBrakeStatus::AntiLockengaged:
            return "engaged";
        default:
            break;
    }
    return "unavailable";
}

static e_BrakeBoostApplied brakeBoostfromJSON(const Poco::JSON::Object::Ptr& brakes)
{
    auto brakeBoostStatus = brakes->getValue<std::string>("brakeBoost");
    if (brakeBoostStatus == "unavailable")
        return e_BrakeBoostApplied::BrakeBoostunavailable;
    else if (brakeBoostStatus == "off")
        return e_BrakeBoostApplied::BrakeBoostoff;
    else if (brakeBoostStatus == "on")
        return e_BrakeBoostApplied::BrakeBooston;
    return e_BrakeBoostApplied::BrakeBoostunavailable;
}

static std::string brakeBoostToJSON(e_BrakeBoostApplied antiLock)
{
    switch (antiLock)
    {
        case e_BrakeBoostApplied::BrakeBoostoff:
            return "off";
        case e_BrakeBoostApplied::BrakeBooston:
            return "on";
        default:
            break;
    }
    return "unavailable";
}

static e_AuxiliaryBrakeStatus auxBrakefromJSON(const Poco::JSON::Object::Ptr& brakes)
{
    auto auxBrakeStatus = brakes->getValue<std::string>("auxBrakes");
    if (auxBrakeStatus == "unavailable")
        return e_AuxiliaryBrakeStatus::AuxiliaryBrkunavailable;
    else if (auxBrakeStatus == "off")
        return e_AuxiliaryBrakeStatus::AuxiliaryBrkoff;
    else if (auxBrakeStatus == "on")
        return e_AuxiliaryBrakeStatus::AuxiliaryBrkon;
    else if (auxBrakeStatus == "reserved")
        return e_AuxiliaryBrakeStatus::AuxiliaryBrkreserved;
    return e_AuxiliaryBrakeStatus::AuxiliaryBrkunavailable;
}

static std::string auxBrakeToJSON(e_AuxiliaryBrakeStatus boost)
{
    switch (boost)
    {
        case e_AuxiliaryBrakeStatus::AuxiliaryBrkoff:
            return "off";
        case e_AuxiliaryBrakeStatus::AuxiliaryBrkon:
            return "on";
        case e_AuxiliaryBrakeStatus::AuxiliaryBrkreserved:
            return "reserved";
        default:
            break;
    }
    return "unavailable";
}

static int degToTenthMicroDeg(double deg)
{
    return static_cast<int>(deg * 10000000);
}

static double tenthMicroDegToDeg(int deg)
{
    return static_cast<double>(deg) / 10000000;
}

static double centiMetersToMeters(int cm)
{
    return static_cast<double>(cm) / 10;
}

static int metersToCentiMeters(double m)
{
    return m * 10;
}

static Int64u tempIdFromOctetString(asnOctetString& id)
{
    Int64u ret = 0;
    asnbyte* data;
    asnMAXUINT len;
    id.getOctetString(&len, &data);
    if (len <= 8)
    {
        for (Int32u i = 0; i < len; i++)
        {
            ret += (data[len - (i + 1)] & 0xFF) << (i * 8);
        }
    }
    return ret;
}

bool basicVehicleFromJSON(const std::string& jsonStr, ::BasicVehicle& veh)
{
    try
    {
        Poco::JSON::Parser parser;
        auto json = parser.parse(jsonStr).extract<Poco::JSON::Object::Ptr>();

        json = json->getObject("BasicSafetyMessage")->getObject("coreData");
        veh.MsgCount = std::stoi(json->getValue<std::string>("msgCnt"));
        veh.DSRCmsgID = 0x20;
        veh.TemporaryID = std::stol(json->getValue<std::string>("id"), 0, 16);
        veh.DSecond = std::stoi(json->getValue<std::string>("secMark"));
        veh.motion.speed = std::stod(json->getValue<std::string>("speed")) /
                           50; // convert from 0.02m/s units -> m/s
        veh.motion.heading = std::stod(json->getValue<std::string>("heading")) /
                             80; // convert from 0.0125 degree units -> degrees
        veh.motion.angle = std::stod(json->getValue<std::string>("angle"));

        veh.pos.latitude = tenthMicroDegToDeg(std::stod(json->getValue<std::string>("lat")));
        veh.pos.longitude = tenthMicroDegToDeg(std::stod(json->getValue<std::string>("lon")));
        veh.pos.elevation = centiMetersToMeters(std::stoi(json->getValue<std::string>("elev")));

        auto subObj = json->getObject("accuracy");
        veh.pos.positionAccuracy = 0;
        int32_t acc = 0;
        acc |= (std::stoi(subObj->getValue<std::string>("semiMajor")) & 0xFF);
        acc |= (std::stoi(subObj->getValue<std::string>("semiMinor")) & 0xFF) << 8;
        acc |= (std::stoi(subObj->getValue<std::string>("orientation")) & 0xFFFF) << 16;
        veh.pos.positionAccuracy = (double)acc;

        subObj = json->getObject("accelSet");
        veh.motion.accel.latAcceleration = std::stod(subObj->getValue<std::string>("lat"));
        veh.motion.accel.longAcceleration = std::stod(subObj->getValue<std::string>("lon"));
        veh.motion.accel.verticalAcceleration = std::stod(subObj->getValue<std::string>("vert"));
        veh.motion.accel.yawRate = std::stod(subObj->getValue<std::string>("yaw"));

        subObj = json->getObject("brakes");
        veh.brakes.tractionCntrStat = tractionCtrlfromJSON(subObj);
        veh.brakes.stabilityCtrlStat = stabilityCtrlfromJSON(subObj);
        veh.brakes.antiLckBrkStat = antilockfromJSON(subObj);
        veh.brakes.brakeBstAppld = brakeBoostfromJSON(subObj);
        veh.brakes.ausciliaryBrkStat = auxBrakefromJSON(subObj);
        veh.brakes.spareBit = false;
        veh.brakes.wheelBrakesUnavialable = true;

        subObj = json->getObject("size");
        veh.length = static_cast<float>(std::stoi(subObj->getValue<std::string>("length")));
        veh.width = static_cast<float>(std::stoi(subObj->getValue<std::string>("width")));
    }
    catch (const std::exception& e)
    {
        MMITSS_LOG("EXCEPTION: %s", e.what());
        return false;
    }

    return true;
}

std::string basicVehicleToJSON(const ::BasicVehicle& veh)
{
    Poco::DynamicStruct core;

    core["msgCnt"] = std::to_string(veh.MsgCount);
    core["id"] = std::to_string(veh.TemporaryID);
    core["secMark"] = std::to_string(veh.DSecond);
    core["speed"] = std::to_string(static_cast<int>(veh.motion.speed * 50));
    core["heading"] = std::to_string(static_cast<int>(veh.motion.heading * 80));
    core["angle"] = std::to_string(static_cast<int>(veh.motion.angle));

    core["lat"] = std::to_string(degToTenthMicroDeg(veh.pos.latitude));
    core["lon"] = std::to_string(degToTenthMicroDeg(veh.pos.longitude));
    core["elev"] = std::to_string(metersToCentiMeters(veh.pos.elevation));

    Poco::DynamicStruct accuracy;
    accuracy["semiMajor"] = std::to_string(static_cast<int>(veh.pos.positionAccuracy) & 0xFF);
    accuracy["semiMinor"] =
        std::to_string((static_cast<int>(veh.pos.positionAccuracy) >> 8) & 0xFF);
    accuracy["orientation"] =
        std::to_string((static_cast<int>(veh.pos.positionAccuracy) >> 16) & 0xFF);
    core["accuracy"] = accuracy;

    Poco::DynamicStruct accelSet;
    accelSet["lat"] = std::to_string(static_cast<int>(veh.motion.accel.latAcceleration));
    accelSet["lon"] = std::to_string(static_cast<int>(veh.motion.accel.longAcceleration));
    accelSet["vert"] = std::to_string(static_cast<int>(veh.motion.accel.verticalAcceleration));
    accelSet["yaw"] = std::to_string(static_cast<int>(veh.motion.accel.yawRate));
    core["accelSet"] = accelSet;

    Poco::DynamicStruct brakes;
    brakes["traction"] = tractionCtrlToJSON(veh.brakes.tractionCntrStat);
    brakes["scs"] = stabilityCtrlToJSON(veh.brakes.stabilityCtrlStat);
    brakes["abs"] = antilockToJSON(veh.brakes.antiLckBrkStat);
    brakes["brakeBoost"] = brakeBoostToJSON(veh.brakes.brakeBstAppld);
    brakes["auxBrakes"] = auxBrakeToJSON(veh.brakes.ausciliaryBrkStat);
    core["brakes"] = brakes;

    Poco::DynamicStruct size;
    size["length"] = std::to_string(static_cast<int>(veh.length));
    size["width"] = std::to_string(static_cast<int>(veh.width));
    core["size"] = size;

    Poco::DynamicStruct coreData;
    coreData["coreData"] = core;
    Poco::DynamicStruct bsm;
    bsm["BasicSafetyMessage"] = coreData;

    return bsm.toString();
}

::BasicVehicle basicVehicleFromBsmCore(const SieMmitssBsmCore& in)
{
    ::BasicVehicle out;
    out.DSRCmsgID = 0x20; // BSM PSID
    out.DSecond = in.secMark;
    out.MsgCount = in.msgCnt;
    out.TemporaryID = in.id;
    out.length = static_cast<float>(in.size.length);
    out.width = static_cast<float>(in.size.width);
    out.motion.angle = static_cast<double>(in.steering);
    out.motion.heading =
        static_cast<double>(in.heading) / 80; // convert from 0.0125 degree units -> degrees
    out.motion.speed = static_cast<double>(in.speed) / 50; // convert from 0.02m/s units -> m/s
    out.motion.accel.latAcceleration = static_cast<double>(in.accelSet.lat);
    out.motion.accel.longAcceleration = static_cast<double>(in.accelSet.lon);
    out.motion.accel.verticalAcceleration = static_cast<double>(in.accelSet.vert);
    out.motion.accel.yawRate = static_cast<double>(in.accelSet.yaw);
    out.pos.latitude = tenthMicroDegToDeg(in.lat);
    out.pos.longitude = tenthMicroDegToDeg(in.lon);
    out.pos.elevation = centiMetersToMeters(in.elev);
    out.brakes.antiLckBrkStat = static_cast<e_AntiLockBrakeStatus>(in.brakes.abs);
    out.brakes.ausciliaryBrkStat = static_cast<e_AuxiliaryBrakeStatus>(in.brakes.auxBrakes);
    out.brakes.brackAppPres = e_BrakeAppliedPressure::unavailable; // no support currently
    out.brakes.brakeBstAppld = static_cast<e_BrakeBoostApplied>(in.brakes.brakeBoost);
    out.brakes.stabilityCtrlStat =
        static_cast<e_StabilityControlStatus>((in.brakes.scs < 3) ? in.brakes.scs : 2);
    out.brakes.tractionCntrStat = static_cast<e_TractionControlState>(in.brakes.traction);
    return out;
}

::BasicVehicle basicVehicleFromSaeBsm(const saeBasicSafetyMessage& in)
{
    ::BasicVehicle out;

    out.DSRCmsgID = its::DOT3_P_PSID_VVSA;
    out.DSecond = in.coreData->secMark;
    out.MsgCount = in.coreData->msgCnt;
    out.TemporaryID = tempIdFromOctetString(in.coreData->id);
    out.length = static_cast<float>(in.coreData->size->length);
    out.width = static_cast<float>(in.coreData->size->width);
    out.motion.angle = static_cast<double>(in.coreData->angle);
    out.motion.heading = static_cast<double>(in.coreData->heading) /
                         80; // convert from 0.0125 degree units -> degrees
    out.motion.speed =
        static_cast<double>(in.coreData->speed) / 50; // convert from 0.02m/s units -> m/s
    out.motion.accel.latAcceleration = static_cast<double>(in.coreData->accelSet->lat);
    out.motion.accel.longAcceleration = static_cast<double>(in.coreData->accelSet->sae__long);
    out.motion.accel.verticalAcceleration = static_cast<double>(in.coreData->accelSet->vert);
    out.motion.accel.yawRate = static_cast<double>(in.coreData->accelSet->yaw);
    out.pos.latitude = tenthMicroDegToDeg(in.coreData->lat);
    out.pos.longitude = tenthMicroDegToDeg(in.coreData->sae__long);
    out.pos.elevation = centiMetersToMeters(in.coreData->elev);
    out.brakes.antiLckBrkStat = static_cast<e_AntiLockBrakeStatus>(in.coreData->brakes->sae__abs);
    out.brakes.ausciliaryBrkStat =
        static_cast<e_AuxiliaryBrakeStatus>(in.coreData->brakes->auxBrakes);
    out.brakes.brackAppPres = e_BrakeAppliedPressure::unavailable; // no support currently
    out.brakes.brakeBstAppld = static_cast<e_BrakeBoostApplied>(in.coreData->brakes->brakeBoost);
    out.brakes.stabilityCtrlStat = static_cast<e_StabilityControlStatus>(
        (in.coreData->brakes->scs < 3) ? in.coreData->brakes->scs : 2);
    out.brakes.tractionCntrStat =
        static_cast<e_TractionControlState>(in.coreData->brakes->traction);

    return out;
}

} // namespace SieMmitss
