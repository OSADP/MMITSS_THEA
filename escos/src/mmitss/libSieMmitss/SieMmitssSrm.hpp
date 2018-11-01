/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitssSrm.hpp
 *
 *  Created on: Aug 01, 2017
 *      Author: M. Venus
 */

#pragma once

#include "Poco/DynamicStruct.h"
#include "Poco/JSON/Object.h"

namespace SieMmitss
{

enum VehicleRole : int
{
    E_BASICVEHICLE = 0,
    E_PUBLICTRANSPORT_EU = 1,
    E_SPECIALTRANSPORT_EU = 2,
    E_DANGEROUSGOODS_EU = 3,
    E_ROADWORK = 4,
    E_ROADRESCUE = 5,
    E_EMERGENCY_EU = 6,
    E_SAFETYCAR_EU = 7,
    E_UNKNOWN = 8,
    E_TRUCK = 9,
    E_MOTORCYCLE = 10,
    E_ROADSIDESOURCE = 11,
    E_POLICE = 12,
    E_FIRE = 13,
    E_AMBULANCE = 14,
    E_DOT = 15,
    E_TRANSIT = 16,
    E_SLOWMOVING = 17,
    E_STOPNGO = 18,
    E_CYCLIST = 19,
    E_PEDESTRIAN = 20,
    E_NONMOTORIZED = 21,
    E_MILITARY = 22,
    E_ROLE_INVALID = -1
};

enum RequestType : int
{
    RESERVED = 0,
    REQUEST = 1,
    REQUEST_UPDATE = 2,
    CANCELLATION = 3,
    REQ_INVALID = -1
};

// requestType conversion for JSON
std::string requestTypeToString(int req);
std::string requestTypeToString(RequestType req);
RequestType requestTypeFromString(const std::string& req);
// vehicleRole conversion for JSON
std::string vehicleRoleToString(int role);
std::string vehicleRoleToString(VehicleRole role);
VehicleRole vehicleRoleFromString(const std::string& role);

class SieMmitssSrm
{
  public:
    /**
     * Construct an SRM from a JSON string
     * @param srmJsonStr string containing the SRM in JSON format
     */
    SieMmitssSrm(const std::string& srmJsonStr);

    /**
     * Check if SRM object is valid
     * @return true if valid (all keys are present), false otherwise
     */
    bool isValid(bool requireGrant) const;

    /**
     * Get the number of requests in SRM
     * @return number of requests in SRM
     */
    int getRequestCount() const;

    /**
     * Get SRM message count
     * @return msgCnt, or -1 if not present
     */
    int getMsgCount() const;

    /**
     * Get VehicleID
     * @return vehicle ID, or -1 if not present
     */
    long getVehicleId() const;

    std::string getVehicleIdString() const;

    std::string getVehicleNameString() const;
    /**
     * Get vehicle type
     * @return vehicleType, or INVALID if not present
     */
    VehicleRole getVehicleRole() const;

    /**
     * Get speed in units of 0.02 m/s
     * @return speed value, or -1 if not present
     */
    int getSpeed() const;

    /**
     * Get latitude in 1/10th microdegrees
     * @return latitude value, or -1 if not present
     */
    int getLat() const;

    /**
     * Get longitude in 1/10th microdegrees
     * @return longitude value, or -1 if not present
     */
    int getLong() const;

    /**
     * Get heading in 0.0125 microdegrees
     * @return longitude value, or -1 if not present
     */
    int getHeading() const;

    /**
     * Get vehicle state
     * @return vehicleState, or -1 if not present
     */
    // int getVehicleState() const;

    int getIntersectionID(int reqIdx = 0) const;

    /**
     * Get the request identifier
     * @return requestID or -1 if not present
     */
    int getRequestID(int reqIdx = 0) const;

    /**
     * Get the request type
     * @return requestType or INVALID if not present
     */
    RequestType getRequestType(int reqIdx = 0) const;

    /**
     * Get inBoundLaneId
     * @return laneID, or -1 if not present
     */
    int getInLaneID(int reqIdx = 0) const;

    /**
     * Get inBoundApproachID
     * @return approachID, or -1 if not present
     */
    int getInApproachID(int reqIdx = 0) const;

    /**
     * Get outBoundLaneId
     * @return laneID, or -1 if not present
     */
    int getOutLaneID(int reqIdx = 0) const;

    /**
     * Get outBoundApproachID
     * @return approachID, or -1 if not present
     */
    int getOutApproachID(int reqIdx = 0) const;

    /**
     * Get EndTimeMinute
     * @return endTimeMinute, or -1 if not present
     */
    int getEndTimeMinute(int reqIdx = 0) const;

    /**
     * Get EndTimeSecond
     * @return endTimeSecond, or -1 if not present
     */
    int getEndTimeSecond(int reqIdx = 0) const;

    /**
     * Get Duration
     * @return duration, or -1 if not present
     */
    int getDuration(int reqIdx = 0) const;

  private:
    using JsonPtr = Poco::JSON::Object::Ptr;
    JsonPtr m_json;
};

} // namespace SieMmitss
