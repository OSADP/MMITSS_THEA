/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitssBsm.hpp
 *
 *  Created on: May 15, 2017
 *      Author: M. Venus
 */

#pragma once

#include "BasicVehicle.h"
#include "facilityLayer.hpp"
#include "facilityWave.hpp"

namespace SieMmitss
{

#pragma pack(1)

struct SieMmitssPositionalAcc
{
    uint8_t semiMajorAcc;
    int8_t semiMinorAcc;
    int16_t semiMajorOrientation;
};

struct SieMmitssAcceleration
{
    int16_t lon;
    int16_t lat;
    int8_t vert;
    int16_t yaw;
};

struct SieMmitssBrakes
{
    uint8_t wheelBrakes;
    uint8_t traction;
    uint8_t abs;
    uint8_t scs;
    uint8_t brakeBoost;
    uint8_t auxBrakes;
};

struct SieMmitssVehicleSize
{
    int16_t width;
    int16_t length;
};

struct SieMmitssBsmCore
{
    uint8_t msgCnt;
    uint64_t id;
    int16_t secMark;
    int32_t lat;
    int32_t lon;
    int32_t elev;
    SieMmitssPositionalAcc accuracy;
    uint8_t transmission;
    int16_t speed;
    int16_t heading;
    int8_t steering;
    SieMmitssAcceleration accelSet;
    SieMmitssBrakes brakes;
    SieMmitssVehicleSize size;
};

#pragma pack()

/**
 * Convert a BasicVehicle object to JSON
 * @param veh BasicVehicle (BSM)
 * @return JSON string or empty on failure
 */
std::string basicVehicleToJSON(const ::BasicVehicle& veh);

/**
 * Convert a JSON input string to a BasicVehicle Object
 * @param json input JSON string (BSM)
 * @param veh output BasicVehicle object
 * @return true on success, false otherwise
 */
bool basicVehicleFromJSON(const std::string& json, ::BasicVehicle& veh);

/**
 * Convert a binary BsmCore structure to a BasicVehicle object
 * @param in binary BSM input
 * @return BasicVehicle object
 */
::BasicVehicle basicVehicleFromBsmCore(const SieMmitssBsmCore& in);

/**
 * Convert an saeBasicSafetyMessage to a BasicVehicle object
 * @param in saeBasicSafetyMessage
 * @return BasicVehicle object
 */
::BasicVehicle basicVehicleFromSaeBsm(const saeBasicSafetyMessage& in);

} // namespace SieMmitss
