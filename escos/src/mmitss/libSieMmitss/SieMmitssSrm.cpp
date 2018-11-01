/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitssBsm.cpp
 *
 *  Created on: May 15, 2017
 *      Author: M.Venus
 */

#include "SieMmitssSrm.hpp"
#include "facilityLayer.hpp"
#include "Poco/JSON/Parser.h"

namespace SieMmitss
{

std::string requestTypeToString(int req)
{
    return requestTypeToString(static_cast<RequestType>(req));
}

std::string requestTypeToString(RequestType req)
{
    if (req == RequestType::RESERVED)
        return "priorityRequestTypeReserved";
    if (req == RequestType::REQUEST)
        return "priorityRequest";
    if (req == RequestType::REQUEST_UPDATE)
        return "priorityRequestUpdate";
    if (req == RequestType::CANCELLATION)
        return "priorityCancellation";
    return "invalid";
}

RequestType requestTypeFromString(const std::string& req)
{
    if (req == "priorityRequestTypeReserved")
        return RequestType::RESERVED;
    if (req == "priorityRequest")
        return RequestType::REQUEST;
    if (req == "priorityRequestUpdate")
        return RequestType::REQUEST_UPDATE;
    if (req == "priorityCancellation")
        return RequestType::CANCELLATION;
    return RequestType::REQ_INVALID;
}

std::string vehicleRoleToString(int role)
{
    return vehicleRoleToString(static_cast<VehicleRole>(role));
}

std::string vehicleRoleToString(VehicleRole role)
{
    if (role == VehicleRole::E_BASICVEHICLE)
        return "basicVehicle";
    if (role == VehicleRole::E_PUBLICTRANSPORT_EU)
        return "publicTransport";
    if (role == VehicleRole::E_SPECIALTRANSPORT_EU)
        return "specialTransport";
    if (role == VehicleRole::E_DANGEROUSGOODS_EU)
        return "dangerousGoods";
    if (role == VehicleRole::E_ROADWORK)
        return "roadWork";
    if (role == VehicleRole::E_ROADRESCUE)
        return "roadRescue";
    if (role == VehicleRole::E_EMERGENCY_EU)
        return "emergency";
    if (role == VehicleRole::E_SAFETYCAR_EU)
        return "safetyCar";
    if (role == VehicleRole::E_UNKNOWN)
        return "none-unknown";
    if (role == VehicleRole::E_TRUCK)
        return "truck";
    if (role == VehicleRole::E_MOTORCYCLE)
        return "motorcycle";
    if (role == VehicleRole::E_ROADSIDESOURCE)
        return "roadSideSource";
    if (role == VehicleRole::E_POLICE)
        return "police";
    if (role == VehicleRole::E_FIRE)
        return "fire";
    if (role == VehicleRole::E_AMBULANCE)
        return "ambulance";
    if (role == VehicleRole::E_DOT)
        return "dot";
    if (role == VehicleRole::E_TRANSIT)
        return "transit";
    if (role == VehicleRole::E_SLOWMOVING)
        return "slowMoving";
    if (role == VehicleRole::E_STOPNGO)
        return "stopNgo";
    if (role == VehicleRole::E_CYCLIST)
        return "cyclist";
    if (role == VehicleRole::E_PEDESTRIAN)
        return "pedestrian";
    if (role == VehicleRole::E_NONMOTORIZED)
        return "nonMotorized";
    if (role == VehicleRole::E_MILITARY)
        return "military";
    return "invalid";
}

VehicleRole vehicleRoleFromString(const std::string& role)
{
    if (role == "basicVehicle")
        return VehicleRole::E_BASICVEHICLE;
    if (role == "publicTransport")
        return VehicleRole::E_PUBLICTRANSPORT_EU;
    if (role == "specialTransport")
        return VehicleRole::E_SPECIALTRANSPORT_EU;
    if (role == "dangerousGoods")
        return VehicleRole::E_DANGEROUSGOODS_EU;
    if (role == "roadWork")
        return VehicleRole::E_ROADWORK;
    if (role == "roadRescue")
        return VehicleRole::E_ROADRESCUE;
    if (role == "emergency")
        return VehicleRole::E_EMERGENCY_EU;
    if (role == "safetyCar")
        return VehicleRole::E_SAFETYCAR_EU;
    if (role == "none-unknown")
        return VehicleRole::E_UNKNOWN;
    if (role == "truck")
        return VehicleRole::E_TRUCK;
    if (role == "motorcycle")
        return VehicleRole::E_MOTORCYCLE;
    if (role == "roadSideSource")
        return VehicleRole::E_ROADSIDESOURCE;
    if (role == "police")
        return VehicleRole::E_POLICE;
    if (role == "fire")
        return VehicleRole::E_FIRE;
    if (role == "ambulance")
        return VehicleRole::E_AMBULANCE;
    if (role == "dot")
        return VehicleRole::E_DOT;
    if (role == "transit")
        return VehicleRole::E_TRANSIT;
    if (role == "slowMoving")
        return VehicleRole::E_SLOWMOVING;
    if (role == "stopNgo")
        return VehicleRole::E_STOPNGO;
    if (role == "cyclist")
        return VehicleRole::E_CYCLIST;
    if (role == "pedestrian")
        return VehicleRole::E_PEDESTRIAN;
    if (role == "nonMotorized")
        return VehicleRole::E_NONMOTORIZED;
    if (role == "military")
        return VehicleRole::E_MILITARY;
    return VehicleRole::E_ROLE_INVALID;
}

SieMmitssSrm::SieMmitssSrm(const std::string& srmJsonStr)
{
    try
    {
        Poco::JSON::Parser parser;
        m_json = parser.parse(srmJsonStr).extract<JsonPtr>();
    }
    catch (const std::exception& e)
    {
        m_json = nullptr;
    }
}

bool SieMmitssSrm::isValid(bool requireGrant) const
{
    try
    {
        if (!m_json->has("vehicleID"))
        {
            ITSAPP_WRN("vehicleID missing");
            return false;
        }
        if (requireGrant and !m_json->has("name"))
        {
            ITSAPP_WRN("name missing");
            return false;
        }
        auto arr = m_json->getArray("requests");
        if (arr->size() == 0)
        {
            ITSAPP_WRN("requests missing");
            return false;
        }

        for (const auto& req : *arr)
        {
            auto obj = req.extract<JsonPtr>();
            if (!obj->has("requestType"))
            {
                ITSAPP_WRN("requestType missing");
                return false;
            }
            if (!obj->has("requestID"))
            {
                ITSAPP_WRN("requestID missing");
                return false;
            }
            if (obj->get("requestType").toString() == "priorityCancellation")
                return true;
        }
    }
    catch (const std::exception& e)
    {
        ITSAPP_TRACE("data missing from message: %s.", e.what());
        return false;
    }

    try
    {
        if (!m_json->has("msgCnt"))
        {
            ITSAPP_WRN("msgCnt missing");
            return false;
        }
        if (!m_json->has("vehicleRole"))
        {
            ITSAPP_WRN("vehicleRole missing");
            return false;
        }
        if (!m_json->has("speed"))
        {
            ITSAPP_WRN("speed missing");
            return false;
        }
        auto arr = m_json->getArray("requests");
        if (arr->size() == 0)
            return false;
        for (const auto& req : *arr)
        {
            auto obj = req.extract<JsonPtr>();
            if (!obj->has("intersectionID"))
            {
                ITSAPP_WRN("intersectionID missing");
                return false;
            }
            if (!obj->has("inLane"))
            {
                ITSAPP_WRN("inLane missing");
                return false;
            }
            if (!obj->has("outLane"))
            {
                ITSAPP_WRN("outLane missing");
                return false;
            }
            if (!obj->has("endTimeMin"))
            {
                ITSAPP_WRN("endTimeMin missing");
                return false;
            }
            if (!obj->has("endTimeSec"))
            {
                ITSAPP_WRN("endTimeSec missing");
                return false;
            }
            if (!obj->has("duration"))
            {
                ITSAPP_WRN("duration missing");
                return false;
            }
        }
    }
    catch (const std::exception& e)
    {
        ITSAPP_TRACE("data missing from message: %s.", e.what());
        return false;
    }
    return true;
}

int SieMmitssSrm::getRequestCount() const
{
    if (m_json->has("requests"))
        return m_json->getArray("requests")->size();
    return -1;
}

int SieMmitssSrm::getMsgCount() const
{
    return m_json->getValue<int>("msgCnt");
}

long SieMmitssSrm::getVehicleId() const
{
    return std::stol(m_json->getValue<std::string>("vehicleID"), 0, 16);
}

std::string SieMmitssSrm::getVehicleIdString() const
{
    return m_json->getValue<std::string>("vehicleID");
}

std::string SieMmitssSrm::getVehicleNameString() const
{
    return m_json->getValue<std::string>("name");
}

VehicleRole SieMmitssSrm::getVehicleRole() const
{
    return vehicleRoleFromString(m_json->getValue<std::string>("vehicleRole"));
}

int SieMmitssSrm::getSpeed() const
{
    return m_json->getValue<int>("speed");
}

int SieMmitssSrm::getLat() const
{
    return m_json->getValue<int>("lat");
}

int SieMmitssSrm::getLong() const
{
    return m_json->getValue<int>("lon");
}

int SieMmitssSrm::getHeading() const
{
    return m_json->getValue<int>("heading");
}

/*int SieMmitssSrm::getVehicleState() const
{
    return m_json->getValue<int>("vehicleState");
}*/

int SieMmitssSrm::getIntersectionID(int reqIdx) const
{
    return m_json->getArray("requests")
        ->get(reqIdx)
        .extract<JsonPtr>()
        ->getValue<int>("intersectionID");
}

int SieMmitssSrm::getRequestID(int reqIdx) const
{
    return m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->getValue<int>("requestID");
}

RequestType SieMmitssSrm::getRequestType(int reqIdx) const
{
    return requestTypeFromString(m_json->getArray("requests")
                                     ->get(reqIdx)
                                     .extract<JsonPtr>()
                                     ->getValue<std::string>("requestType"));
}

int SieMmitssSrm::getInLaneID(int reqIdx) const
{
    if (!m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->has("inLane"))
        return -1;
    return m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->getValue<int>("inLane");
}

int SieMmitssSrm::getInApproachID(int reqIdx) const
{
    if (!m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->has("inApproach"))
        return -1;
    return m_json->getArray("requests")
        ->get(reqIdx)
        .extract<JsonPtr>()
        ->getValue<int>("inApproach");
}

int SieMmitssSrm::getOutLaneID(int reqIdx) const
{
    if (!m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->has("outLane"))
        return -1;
    return m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->getValue<int>("outLane");
}

int SieMmitssSrm::getOutApproachID(int reqIdx) const
{
    if (!m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->has("outApproach"))
        return -1;
    return m_json->getArray("requests")
        ->get(reqIdx)
        .extract<JsonPtr>()
        ->getValue<int>("outApproach");
}

int SieMmitssSrm::getEndTimeMinute(int reqIdx) const
{
    return m_json->getArray("requests")
        ->get(reqIdx)
        .extract<JsonPtr>()
        ->getValue<int>("endTimeMin");
}

int SieMmitssSrm::getEndTimeSecond(int reqIdx) const
{
    return m_json->getArray("requests")
        ->get(reqIdx)
        .extract<JsonPtr>()
        ->getValue<int>("endTimeSec");
}

int SieMmitssSrm::getDuration(int reqIdx) const
{
    return m_json->getArray("requests")->get(reqIdx).extract<JsonPtr>()->getValue<int>("duration");
}

} // namespace SieMmitss
