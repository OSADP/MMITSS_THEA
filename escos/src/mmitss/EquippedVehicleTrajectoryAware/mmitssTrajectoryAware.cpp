/* Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* MMITSS_rsu_BSM_receiver_Gavilan_Savari.cpp
 * Created by Yiheng Feng on 7/11/14.
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 *
 */

// Work with Vissim 6 DriverModel through UDP
// Need drivermodel_udp_R.dll from "DriverModel_DLL_UDP_InFusion" running

// 2014.4.16: Added function
// Receive request from Signal_Control and Performance Measure to send trajectory data

// 2014.7.11: Changed Function
// BSM unpacking is now using Savari function.

#include "mmitssTrajectoryAware.hpp"

#include <arpa/inet.h>
#include <facilityLayer.hpp>
#include <netinet/in.h>
#include <Poco/Base64Encoder.h>
#include <Poco/Dynamic/Struct.h>
#include <Poco/Dynamic/Var.h>
#include <Poco/Exception.h>
#include <Poco/Net/IPAddress.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/String.h>
#include <sigc++/functors/mem_fun.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>

#include "mmitssUtils.h"
#include "Signal.h"
#include "Motion.h"
#include "PositionLocal3D.h"
#include "SieMmitssBsm.hpp"
#include "SieMmitssConfig.hpp"

class saeBasicSafetyMessage;

#ifndef DEG2ASNunits
#define DEG2ASNunits (1000000.0) // used for ASN 1/10 MICRO deg to unit converts
#endif

static const int BSM_RX_BUF_SIZE = 4096;
static const int SEND_BUF_SIZE = 500000;

namespace WaveApp
{
namespace Mmitss
{

void DistanceFromLine(double cx,
    double cy,
    double ax,
    double ay,
    double bx,
    double by,
    double& distanceSegment,
    double& distanceLine);

const uint32_t UDP_RX_TIMEOUT_MS = 500;

MmitssTrajectoryAware::MmitssTrajectoryAware(MmitssTrajectoryAwareApp& parent)
    : m_parent(parent)
    , m_udpReceiver(::SieMmitss::getTrajectoryAwarePort(),
          BSM_RX_BUF_SIZE,
          [this](const void* data, size_t len) { return this->onUdpReceive(data, len); })
{
}

void MmitssTrajectoryAware::onRxBsm(const saeBasicSafetyMessage& msg)
{
    m_rxVehicles.emplace_back(SieMmitss::basicVehicleFromSaeBsm(msg));
}

bool MmitssTrajectoryAware::onStartup()
{
    return mmitssInit();
}

void MmitssTrajectoryAware::onShutdown()
{
    if (m_timerId != 0)
        siekdbus::timerSetEnabled(m_timerId, false);
    if (m_sendingSocket > 0)
        close(m_sendingSocket);
    m_udpReceiver.stop();
}

void MmitssTrajectoryAware::onExit()
{
    ITSAPP_TRACE("exiting..");
    if (m_timerId > 0)
    {
        m_timerConn.disconnect();
        m_timerId = siekdbus::timerDestroy(m_timerId);
    }
}

bool MmitssTrajectoryAware::mmitssInit()
{
    ITSAPP_TRACE("mmitssInit start..");
    m_trackedVeh.clear();
    m_mapNodes.clear();
    m_rxVehicles.clear();
    m_trajDataSize = 0;

    m_started = false;
    // This can take really long time and we do not want to trigger watchdog killer.
    // We read this in tick() function
    m_delayedInit = std::async(std::launch::async, [this]() { return loadMmitssConfig(); });

    m_sendingSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sendingSocket < 0)
    {
        ITSAPP_ERROR("sending socket failed: %m");
        return false;
    }

    m_testSystemAddr = SieMmitss::getTestSystemSocketAddress();
    if (!m_testSystemAddr.empty())
    {
        ITSAPP_LOG("Test System address: %s", m_testSystemAddr.c_str());
    }
    ITSAPP_TRACE("mmitssInit end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssTrajectoryAware::onUdpReceive(const void* data, size_t len)
{
    if ((data == nullptr) || (len == 0))
        return;
    const char* rx = reinterpret_cast<const char*>(data);
    // trajectory requests
    if (std::string(rx).substr(0, strlen("Request")) == std::string("Request"))
    {
        std::string str(rx);
        std::stringstream ss(str);
        std::string request{""}, from{""};
        uint32_t reqSeq = 0;
        ss >> request >> from >> reqSeq;
        // trajectory request from performance observer
        if (from == "performance_observer")
        {
            sendTrajectoryData(Poco::Net::SocketAddress("127.0.0.1", SieMmitss::getPerfDataPort()));
            if (m_trajDataSize > 0)
                ITSAPP_TRACE(
                    "Send trajectory data to Performance Observer per request %d! The size is %d.",
                    reqSeq, m_trajDataSize);
        }

        // trajectory request from priority solver
        if (from == "priority_solver")
        {
            sendTrajectoryData(
                Poco::Net::SocketAddress("127.0.0.1", SieMmitss::getPrioSolverPort()), true);

            if (m_trajDataSize > 0)
                ITSAPP_TRACE(
                    "Send trajectory data to priority solver per request %d! The size is %d.",
                    reqSeq, m_trajDataSize);
        }

        // trajectory request from signal control
        if (from == "signal_control")
        {
            sendTrajectoryData(
                Poco::Net::SocketAddress("127.0.0.1", SieMmitss::getSignalControlTrajectoryPort()),
                true);

            if (m_trajDataSize > 0)
                ITSAPP_TRACE(
                    "Send trajectory data to signal control per request %d! The size is %d.",
                    reqSeq, m_trajDataSize);
        }
    }
    // BSM reception
    else if (rx[0] == '{')
    { // JSON interface
        BasicVehicle vehIn;
        if (SieMmitss::basicVehicleFromJSON(rx, vehIn))
        {
            for (const auto& veh : m_rxVehicles)
            {
                if (vehIn.TemporaryID == veh.TemporaryID)
                    return;
            }
            m_rxVehicles.emplace_back(vehIn);
        }
        else
            ITSAPP_WRN("ERROR: Invalid JSON BSM");
    }
    else if (len >= (int)sizeof(SieMmitss::SieMmitssBsmCore))
    { // binary interface
        BasicVehicle vehIn;
        auto* bsmCore = reinterpret_cast<const SieMmitss::SieMmitssBsmCore*>(rx);
        vehIn = SieMmitss::basicVehicleFromBsmCore(*bsmCore);
        m_rxVehicles.emplace_back(vehIn);
    }
}

void MmitssTrajectoryAware::onTimer(siekdbus::timerid_t id)
{
    (void)id;
    if (not m_started)
        return;

    for (const auto& vehIn : m_rxVehicles)
    {
        int Veh_pos_inlist = 0;

        auto veh = m_trackedVeh.begin();

        int found = 0; // flag to indicate whether a vehicle vehIn in m_rxVehicles is found in the
                       // list m_trackedVeh
        int pro_flag = 0; // indicate whether to map the received data in the MAP

        double t1 = GetSeconds();

        while (veh != m_trackedVeh.end()) // match vehicle according to vehicle ID
        {
            // if ID is a match, store trajectory information
            // If ID will change, then need an algorithm to do the match!!!!
            if (veh->TempID == vehIn.TemporaryID) // every 0.5s processes a trajectory
            {
                if (t1 - veh->receive_timer > 0.5)
                {
                    pro_flag = 1;            // need to map the point
                    veh->receive_timer = t1; // reset the timer
                    veh->traj[veh->nFrame].latitude = vehIn.pos.latitude;
                    veh->traj[veh->nFrame].longitude = vehIn.pos.longitude;
                    veh->traj[veh->nFrame].elevation = vehIn.pos.elevation;
                    // change GPS points to local points
                    m_refPoint.lla2ecef(vehIn.pos.longitude, vehIn.pos.latitude,
                        vehIn.pos.elevation, &m_ecefX, &m_ecefY, &m_ecefZ);
                    m_refPoint.ecef2local(
                        m_ecefX, m_ecefY, m_ecefZ, &m_localX, &m_localY, &m_localZ);
                    veh->N_Offset[veh->nFrame] = m_localX;
                    veh->E_Offset[veh->nFrame] = m_localY;
                    // instantaneously acceleration added 4/20/2014
                    veh->acceleration = (vehIn.motion.speed - veh->Speed) * 2;
                    // Speed from Savari BSM decoder is KM/H, need to change to m/s
                    veh->Speed = vehIn.motion.speed / (double)m_speedCoif;
                    veh->setHeadingDegree(vehIn.motion.heading);
                    veh->Dsecond = vehIn.DSecond;
                    veh->time[veh->nFrame] = t1;

                    if (veh->Speed < 0.8 && veh->stop_flag == 0)
                    { // vehicle has not stopped before, for the ELVS algorithm
                        veh->stop_flag = 1;
                        veh->queue_flag = 1; // for Performance Observer
                        veh->time_stop = GetSeconds();
                    }

                    if (veh->Speed > 0.8 && veh->stop_flag == 1)
                    { // Added by Shayan to keep track of the last stopped vehicle
                        veh->stop_flag = 0;
                    }

                    veh->nFrame++;

                    // if reached the end of the trajectory, start over
                    if (veh->nFrame == ConnectedVehicle::MAX_FRAMES)
                        veh->nFrame = 0;
                    // store this vehicle's position in the tracked vehicle list, start from 0
                    Veh_pos_inlist = veh - m_trackedVeh.begin();
                }
                found = 1;
                break;
            }
            veh++;
        }

        ConnectedVehicle TempVeh;
        if (found == 0) // this is a new vehicle
        {
            TempVeh.TempID = vehIn.TemporaryID;
            TempVeh.traj[0].latitude = vehIn.pos.latitude;
            TempVeh.traj[0].longitude = vehIn.pos.longitude;
            TempVeh.traj[0].elevation = vehIn.pos.elevation;
            // change GPS points to local points
            m_refPoint.lla2ecef(vehIn.pos.longitude, vehIn.pos.latitude, vehIn.pos.elevation,
                &m_ecefX, &m_ecefY, &m_ecefZ);
            m_refPoint.ecef2local(m_ecefX, m_ecefY, m_ecefZ, &m_localX, &m_localY, &m_localZ);
            // localX and localY here are different from normal localX(E) and localY(N)
            TempVeh.N_Offset[0] = m_localX;
            TempVeh.E_Offset[0] = m_localY;
            TempVeh.Speed = vehIn.motion.speed / (double)m_speedCoif;
            TempVeh.setHeadingDegree(vehIn.motion.heading);
            TempVeh.acceleration = 0;
            TempVeh.nFrame = 1;
            TempVeh.Phase_Request_Counter = 0;
            TempVeh.active_flag = m_vehDeleteTimeSec; // initilize the active_flag
            TempVeh.receive_timer = GetSeconds();
            TempVeh.time[0] = TempVeh.receive_timer;
            TempVeh.stop_flag = 0;
            TempVeh.queue_flag = 0;
            TempVeh.processed = 0;
            TempVeh.entry_time = GetSeconds();
            // To keep track of the approach for Performance by movement
            TempVeh.previous_approach = 0;

            TempVeh.temp_approach = 0;
            // To keep track of the lane for performance by movement
            TempVeh.previous_lane = 0;
            TempVeh.temp_lane = 0;

            TempVeh.time_stop = 0;

            //----------Vehicle Entrance into the Geo Fenced Area----------//

            // Find the Nearest Lane info from the current vehicle location
            int N_App = 0;
            int N_Node = 0; // Nearest Approach, Lane, and Node
            int N_pos = 0;  // node position in the vector
            double Tempdis = 10000000.0;
            unsigned int s = 0;
            // The output of DistanceFromLine Function:
            double distanceSegment, distanceLine;
            double dis_to_ref; // Distance to the Refernce Point
            // find the nearest lane node
            for (s = 0; s < m_mapNodes.size(); s++)
            {
                double Dis = sqrt(pow((m_localX - m_mapNodes[s].N_Offset), 2) +
                                  pow((m_localY - m_mapNodes[s].E_Offset), 2));
                if (Dis < Tempdis)
                {
                    Tempdis = Dis;
                    N_App = m_mapNodes[s].index.Approach;
                    N_Node = m_mapNodes[s].index.Node;
                    N_pos = s;
                }
            }

            // cout<<"nearest node is: "<<N_App<<"."<<N_Lane<<"."<<N_Node<<endl;

            if (Tempdis < 1000000.0) // Find a valid node
            {
                // cout<<N_App<<"."<<N_Lane<<"."<<N_Node<<endl;
                if (N_Node == 1)
                { // Case when the nearest node is the first node of the lane: The line bw
                    // m_mapNodes[s] and m_mapNodes[s+1]
                    DistanceFromLine(m_localX, m_localY, m_mapNodes[N_pos].N_Offset,
                        m_mapNodes[N_pos].E_Offset, m_mapNodes[N_pos + 1].N_Offset,
                        m_mapNodes[N_pos + 1].E_Offset, distanceSegment, distanceLine);
                    // cout<<"Two points are:
                    // "<<m_mapNodes[N_pos].index.Approach<<"."<<m_mapNodes[N_pos].index.Lane<<"."<<m_mapNodes[N_pos].index.Node<<"
                    // and
                    // "<<m_mapNodes[N_pos+1].index.Approach<<"."<<m_mapNodes[N_pos+1].index.Lane<<"."<<m_mapNodes[N_pos+1].index.Node<<endl;
                }
                else
                { // line bw m_mapNodes[s] and m_mapNodes[s-1]
                    DistanceFromLine(m_localX, m_localY, m_mapNodes[N_pos].N_Offset,
                        m_mapNodes[N_pos].E_Offset, m_mapNodes[N_pos - 1].N_Offset,
                        m_mapNodes[N_pos - 1].E_Offset, distanceSegment, distanceLine);
                    // cout<<"Two points are:
                    // "<<m_mapNodes[N_pos].index.Approach<<"."<<m_mapNodes[N_pos].index.Lane<<"."<<m_mapNodes[N_pos].index.Node<<"
                    // and
                    // "<<m_mapNodes[N_pos-1].index.Approach<<"."<<m_mapNodes[N_pos-1].index.Lane<<"."<<m_mapNodes[N_pos-1].index.Node<<endl;
                }
                // cout<<"Distance To Line is: "<<distanceLine<<endl;
                TempVeh.approach = m_mapNodes[N_pos].index.Approach;
                TempVeh.lane = m_mapNodes[N_pos].index.Lane;

                // Add the vehcile into the list only when it is inside the GeoFenced area

                dis_to_ref = sqrt(pow(m_localX, 2) + pow(m_localY, 2));

                ITSAPP_TRACE("dis_to_ref %f, m_dsrcRange[N_App-1] is %f, distanceLine is %f.",
                    dis_to_ref, (double)m_dsrcRange[N_App - 1], distanceLine);

                if ((float)dis_to_ref <= m_dsrcRange[N_App - 1] && (distanceLine <= 8))
                {
                    m_trackedVeh.emplace_back(TempVeh);

                    ITSAPP_TRACE(
                        "Add Vehicle ID=%d at %lf, the list size is %d, approach: %d, lane: %d",
                        TempVeh.TempID, GetSeconds(), m_trackedVeh.size(), TempVeh.approach,
                        TempVeh.lane);

                    Veh_pos_inlist = m_trackedVeh.size() - 1;
                }
                else
                {
                    ITSAPP_TRACE("Vehicle out of range: %lf,  %lf", dis_to_ref, distanceLine);
                }
            }
            else
            {
                ITSAPP_TRACE("Vehicle not in intersection geo area!");
            }
        }

        // Find the vehicle in the map: output: distance, eta and requested phase

        // t_2=GetSeconds();

        // sprintf(temp_log,"The time for Adding vehicle to the list is: %lf second \n",t_2-t_1);
        // outputlog(temp_log); cout<<temp_log;

        if (pro_flag == 1)
        {
            // t_1=GetSeconds();
            veh = m_trackedVeh.begin() + Veh_pos_inlist;
            double tmp_speed = veh->Speed;
            double tmp_heading = veh->getHeading();
            int tmp_nFrame = veh->nFrame;

            double tmp_N_Offset1 = veh->N_Offset[(tmp_nFrame - 1 + ConnectedVehicle::MAX_FRAMES) %
                                                 ConnectedVehicle::MAX_FRAMES];
            double tmp_E_Offset1 = veh->E_Offset[(tmp_nFrame - 1 + ConnectedVehicle::MAX_FRAMES) %
                                                 ConnectedVehicle::MAX_FRAMES];
            double tmp_N_Offset2 = veh->N_Offset[(tmp_nFrame - 2 + ConnectedVehicle::MAX_FRAMES) %
                                                 ConnectedVehicle::MAX_FRAMES];
            double tmp_E_Offset2 = veh->E_Offset[(tmp_nFrame - 2 + ConnectedVehicle::MAX_FRAMES) %
                                                 ConnectedVehicle::MAX_FRAMES];

            //--------Vehicle Exit from the Geo Fenced Area--------//

            // Find the Nearest Lane info from the current vehicle location
            int N_App = 0;
            int N_Node = 0; // Nearest Approach, Lane, and Node
            int N_pos = 0;  // node position in the vector
            double Tempdis = 1000000.0;
            unsigned int s = 0;
            // The output of DistanceFromLine Function:
            double distanceSegment, distanceLine;
            double dis_to_ref; // Distance to the reference point

            // find the nearest lane node
            for (s = 0; s < m_mapNodes.size(); s++)
            {
                double Dis = sqrt(pow((m_localX - m_mapNodes[s].N_Offset), 2) +
                                  pow((m_localY - m_mapNodes[s].E_Offset), 2));
                if (Dis < Tempdis)
                {
                    Tempdis = Dis;
                    N_App = m_mapNodes[s].index.Approach;
                    N_Node = m_mapNodes[s].index.Node;
                    N_pos = s;
                }
            }

            if (Tempdis < 1000000.0)
            {
                // cout<<N_App<<"."<<N_Lane<<"."<<N_Node<<endl;

                if (N_Node == 1) // Case when the nearest node is the first node of the lane: The
                                 // line bw m_mapNodes[s] and m_mapNodes[s+1]
                {
                    DistanceFromLine(m_localX, m_localY, m_mapNodes[N_pos].N_Offset,
                        m_mapNodes[N_pos].E_Offset, m_mapNodes[N_pos + 1].N_Offset,
                        m_mapNodes[N_pos + 1].E_Offset, distanceSegment, distanceLine);
                    // cout<<"Two points are:
                    // "<<m_mapNodes[N_pos].index.Approach<<"."<<m_mapNodes[N_pos].index.Lane<<"."<<m_mapNodes[N_pos].index.Node<<"
                    // and
                    // "<<m_mapNodes[N_pos+1].index.Approach<<"."<<m_mapNodes[N_pos+1].index.Lane<<"."<<m_mapNodes[N_pos+1].index.Node<<endl;
                }
                else
                {
                    // line bw m_mapNodes[s] and m_mapNodes[s-1]
                    DistanceFromLine(m_localX, m_localY, m_mapNodes[N_pos].N_Offset,
                        m_mapNodes[N_pos].E_Offset, m_mapNodes[N_pos - 1].N_Offset,
                        m_mapNodes[N_pos - 1].E_Offset, distanceSegment, distanceLine);
                    // cout<<"Two points are:
                    // "<<m_mapNodes[N_pos].index.Approach<<"."<<m_mapNodes[N_pos].index.Lane<<"."<<m_mapNodes[N_pos].index.Node<<"
                    // and
                    // "<<m_mapNodes[N_pos-1].index.Approach<<"."<<m_mapNodes[N_pos-1].index.Lane<<"."<<m_mapNodes[N_pos-1].index.Node<<endl;
                }

                // cout<<"Distance To Line is: "<<distanceLine<<endl;

                dis_to_ref = sqrt(pow(tmp_N_Offset1, 2) + pow(tmp_E_Offset1, 2));

                if ((float)dis_to_ref >= m_dsrcRange[N_App - 1] ||
                    distanceLine >= 8) // Outside the GeoFence Area
                {
                    veh->req_phase = -1;
                    veh->ETA = 9999;
                    veh->stopBarDistance = 9999;
                    ITSAPP_TRACE("693:DELETE vehicle : ID=%d", veh->TempID);
                    veh = m_trackedVeh.erase(veh);
                }
                else
                {
                    veh->leaving_time = GetSeconds();

                    int pre_app = veh->previous_approach;

                    if (veh->req_phase < 0 && dis_to_ref < 30)
                    { // vehicle passes the stopbar -> set the approach and lane to be previous
                        // approach and lane
                        veh->approach = veh->previous_approach; // veh->approach = pre_app;
                        veh->lane = veh->previous_lane;         // veh->lane = pre_lane;
                        ITSAPP_TRACE(
                            "ID=%d,nFrame=%d,Appr=%d,Ln=%d,Ph=%d,Head=%3.2lf,Spd=%4.1lf,RefDist=%4."
                            "2lf: Vehicle passed stopbar",
                            veh->TempID, veh->nFrame, veh->approach, veh->lane, veh->req_phase,
                            veh->getHeading(), veh->Speed, dis_to_ref);
                    }
                    else // if(veh->req_phase >= 0 || dis_to_ref >= 35) //Inside the GeoFence but
                         // not within the intersection
                    {    // vehicle
                        int curApproach = 0;
                        int curLane = 0;
                        FindVehInMap(tmp_speed, tmp_heading, tmp_nFrame, tmp_N_Offset1,
                            tmp_E_Offset1, tmp_N_Offset2, tmp_E_Offset2, m_newMap, pre_app,
                            veh->stopBarDistance, veh->ETA, veh->req_phase, curApproach, curLane);
                        veh->approach = curApproach;
                        veh->lane = curLane;
                        veh->processed =
                            1; // already go through the find vehicle in the mAP function
                        veh->active_flag = m_vehDeleteTimeSec;

                        if (veh->req_phase < 0 && dis_to_ref < 30)
                        { // For the first times after passing the stop bar ignore what FindVehInMap
                            // did,
                            // because it was not correct!
                            veh->approach = veh->previous_approach;
                            veh->lane = veh->previous_lane;
                        }

                        ITSAPP_TRACE(
                            "ID=%d,nFrame=%d,Appr=%d,Ln=%d,Ph=%d,Head=%3.2lf,Spd=%4.1lf,StopDist=%"
                            "4.2lf: Vehicle approaching",
                            veh->TempID, veh->nFrame, curApproach, curLane, veh->req_phase,
                            veh->getHeading(), veh->Speed, veh->stopBarDistance);

                        if (veh->temp_approach != veh->approach)
                        {
                            veh->previous_approach = veh->temp_approach;
                            veh->temp_approach = veh->approach;

                            if (veh->previous_approach == 0)
                                veh->previous_approach = curApproach;

                            //----------------------------Dealing with U-Turn
                            // Movements------------------------------
                            if (veh->previous_approach != 0)
                            {
                                if (veh->approach % 2 == 1) // U-turn after passing the intersection
                                {
                                    if (veh->previous_approach == veh->approach + 1)
                                    {
                                        // cout<<"Vehicle ID of "<<veh->TempID<<" is
                                        // Deleted!"<<endl;
                                        //                                        m_trackedVeh.DeleteAt();
                                        ITSAPP_TRACE("765:Delete Vehicle No. %d ", veh->TempID);
                                        veh = m_trackedVeh.erase(veh);
                                        //                                        m_trackedVeh.erase(m_trackedVeh.begin()
                                        //                                        + Veh_pos_inlist);
                                    }
                                }
                                else // U-turn at the signal
                                {
                                    if (veh->previous_approach == veh->approach - 1)
                                    {
                                        // cout<<"Vehicle ID of "<<veh->TempID<<" is
                                        // Deleted!"<<endl;
                                        //                                        m_trackedVeh.DeleteAt();
                                        ITSAPP_TRACE("744:Delete Vehicle No. %d ", veh->TempID);
                                        veh = m_trackedVeh.erase(veh);
                                        //                                        m_trackedVeh.erase(m_trackedVeh.begin()
                                        //                                        + Veh_pos_inlist);
                                    }
                                }
                            }
                        }

                        if (veh->temp_lane != veh->lane)
                        {
                            veh->previous_lane = veh->temp_lane;
                            veh->temp_lane = veh->lane;

                            if (veh->previous_lane == 0)
                                veh->previous_lane = curLane;
                        }
                    }
                }
            }
            else
            {
                ITSAPP_TRACE("!Vehicle not in intersection geo area");
            }
            pro_flag = 0;
        }
    }
    m_rxVehicles.clear();

    // go through the tracked list to create the arrival table every rolling-horizon time
    int currenttime = time(NULL);

    int difference = currenttime - m_beginTime;

    int rolling_horizon = 1; // every 1 seconds, write the arrival table to a file
    if (difference >= rolling_horizon)
    {
        // go through the vehicle list and delete the already left vehicle
        for (auto veh = m_trackedVeh.begin(); veh != m_trackedVeh.end();)
        {
            if ((--veh->active_flag) < 0)
            {
                ITSAPP_TRACE("812:Delete Vehicle No. %d ", veh->TempID);
                veh = m_trackedVeh.erase(veh);
            }
            else
            {
                ++veh;
            }
        }

        // reset begintime;
        m_beginTime = currenttime;
    }
}

bool MmitssTrajectoryAware::get_lane_phase_mapping() // Yiheng add 07/18/2014
{
    fstream fs;

    fs.open(SieMmitss::getLanePhaseMapFilePath().c_str());

    std::string temp_string;

    getline(fs, temp_string); // First line is comment
    getline(fs, temp_string); // Second line contains information

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading Lane_Phase_Mapping_File problem");
        fs.close();
        return false;
    }

    char tmp[128];
    strcpy(tmp, temp_string.c_str());
    sscanf(tmp, "%d %d %d %d %d %d %d %d", &m_phaseMapping[0], &m_phaseMapping[1],
        &m_phaseMapping[2], &m_phaseMapping[3], &m_phaseMapping[4], &m_phaseMapping[5],
        &m_phaseMapping[6], &m_phaseMapping[7]);

    fs.close();
    return true;
}

bool MmitssTrajectoryAware::get_DSRC_Range()
{
    fstream sh;
    sh.open(SieMmitss::getDsrcRangeFilePath().c_str());

    std::string temp_string;

    getline(sh, temp_string); // Reading the first line which explains the legs of the intersection
                              // assignment
    getline(sh, temp_string); // Reading the second line which has the values for each leg

    if ((temp_string.size() == 0) || (temp_string.size() >= 128))
    {
        ITSAPP_WRN("Reading DSRC_Range_File problem!!");
        sh.close();
        return false;
    }

    char tmp[128];
    strcpy(tmp, temp_string.c_str());
    sscanf(tmp, "%f %f %f %f %f %f %f %f", &m_dsrcRange[0], &m_dsrcRange[1], &m_dsrcRange[2],
        &m_dsrcRange[3], &m_dsrcRange[4], &m_dsrcRange[5], &m_dsrcRange[6], &m_dsrcRange[7]);

    getline(sh, temp_string); // Reading the third line which explains the central rectangle
    getline(sh,
        temp_string); // Reading the fourth line which has the values for each side of the rectangle

    sh.close();
    return true;
}

void MmitssTrajectoryAware::sendTrajectoryData(
    const Poco::Net::SocketAddress& socketAddr, bool toSignalControl, bool json)
{
    try
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(
            socketAddr
                .port()); // to performance observer the port is 22222!!!!!!!!!!!!!!!!!!!!!!!!!!
        dest_addr.sin_addr.s_addr =
            inet_addr(socketAddr.host().toString().c_str()); // Send to local host;
        memset(dest_addr.sin_zero, '\0', sizeof dest_addr.sin_zero);

        int addr_length = sizeof(dest_addr);

        // Pack the trajectory data to a octet stream
        char traj_data[SEND_BUF_SIZE]; // 500k is the maximum can be sent
        // Pack the data from m_trackedVeh to a octet string
        if (toSignalControl)
            packTrajData1(traj_data, m_trajDataSize);
        else
            packTrajData(traj_data, m_trajDataSize);
        if (m_trajDataSize < SEND_BUF_SIZE)
            traj_data[m_trajDataSize + 1] = 0;

        // Send trajectory data
        // Poco::Net::DatagramSocket dest;

        if (!json)
        {
            sendto(m_sendingSocket, traj_data, m_trajDataSize + 1, 0, (struct sockaddr*)&dest_addr,
                addr_length);
            // dest.sendTo(traj_data, m_trajDataSize + 1, socketAddr);
            return;
        }
        // send as json
        // convert binary to base64
        std::ostringstream s;
        Poco::Base64Encoder base64enc(s);
        base64enc.write(traj_data, m_trajDataSize + 1);
        base64enc.close();
        Poco::DynamicStruct res;
        res["traj_data"] = Poco::replace(s.str(), "\r\n", "").data();
        std::string trajJsonStr = res.toString();
        // send json
        // dest.sendTo(trajJsonStr.data(), trajJsonStr.size(), socketAddr);
    }
    catch (const Poco::Exception& e)
    {
        ITSAPP_WRN("Caught Poco:Exception: %s, %s", e.what(), e.displayText().c_str());
    }
}

void MmitssTrajectoryAware::FindVehInMap(double Speed,
    double Heading,
    int nFrame,
    double N_Offset1,
    double E_Offset1,
    double N_Offset2,
    double E_Offset2,
    MAP NewMap,
    int pre_app,
    double& Dis_curr,
    double& est_TT,
    int& request_phase,
    int& approach,
    int& lane)
{
    (void)NewMap;
    (void)pre_app;

    // Find Vehicle position in the map
    // find the nearest Lane nodes from the vehicle location
    int t_App = 0;
    int t_Lane = 0;
    int t_Node = 0;
    int t_pos = 0; // node position in the vector
    double lane_heading;
    // temp vehicle point

    // calculate the vehicle in the MAP
    // find lane, requesting phase and distance
    double tempdis = 1000000.0;
    // find the nearest lane node
    for (size_t j = 0; j < m_mapNodes.size() - 1; j++)
    {
        // only judge over two nodes on the same lane
        // if this is the last node on this lane
        if (m_mapNodes[j].index.Lane != m_mapNodes[j + 1].index.Lane)
            continue;
        double dis = sqrt(distbeforeSqrt(m_mapNodes[j].N_Offset, m_mapNodes[j].E_Offset,
            m_mapNodes[j + 1].N_Offset, m_mapNodes[j + 1].E_Offset, m_localX, m_localY));
        if (dis < tempdis)
        {
            tempdis = dis;
            t_App = m_mapNodes[j].index.Approach;
            t_Lane = m_mapNodes[j].index.Lane;
            t_Node = m_mapNodes[j].index.Node;
            t_pos = j;
        }
    }

    approach = t_App;
    lane = t_Lane;

    if (nFrame >= 2) // start from second frame
    {
        // determine it is approaching the intersection or leaving the intersection or in queue
        // The threshold for determing in queue: 89.4 cm = 2mph
        // calculate the distance from the reference point here is 0,0;
        int veh_state = 0; // 1: appraoching; 2: leaving; 3: queue
        double N_Pos;      // current vehicle position
        double E_Pos;
        double E_Pos2; // previous vehicle position
        double N_Pos2;

        int match = 0; // whether the vehicle's heading match the lanes heading

        // find the first node (nearest of intersection) of the lane
        double inter_pos_N = m_mapNodes[t_pos - t_Node + 1].N_Offset;
        double inter_pos_E = m_mapNodes[t_pos - t_Node + 1].E_Offset;

        N_Pos = N_Offset1; // current position
        E_Pos = E_Offset1;
        N_Pos2 = N_Offset2; // previous frame position
        E_Pos2 = E_Offset2;

        // double veh_heading=atan2(N_Pos-N_Pos2,E_Pos-E_Pos2)*180/M_PI;
        double veh_heading = Heading;

        if (veh_heading < 0)
            veh_heading += 360;

        lane_heading = m_mapNodes[t_pos].Heading;

        if (lane_heading < 0)
            lane_heading += 360;

        if (abs(veh_heading - lane_heading) > 120 && abs(veh_heading - lane_heading) < 240)
        {
            if (approach % 2 == 1) // When the approach is 1, 3, 5,or 7
                approach = approach + 1;
            else // When the approach number is 2, 4, 6,or 8
                approach = approach - 1;
        }

        if (abs(veh_heading - lane_heading) < 20) // threshold for the difference of the heading
            match = 1;
        if (veh_heading > 340 && lane_heading < 20)
        {
            veh_heading = veh_heading - 360;
            if (abs(veh_heading - lane_heading) < 20)
                match = 1;
        }
        if (lane_heading > 340 && veh_heading < 20)
        {
            lane_heading = lane_heading - 360;
            if (abs(lane_heading - veh_heading) < 20)
                match = 1;
        }

        double Dis_pre = sqrt((N_Pos2 - inter_pos_N) * (N_Pos2 - inter_pos_N) +
                              (E_Pos2 - inter_pos_E) * (E_Pos2 - inter_pos_E));
        Dis_curr = sqrt((N_Pos - inter_pos_N) * (N_Pos - inter_pos_N) +
                        (E_Pos - inter_pos_E) * (E_Pos - inter_pos_E));

        if (match == 1 && approach % 2 == 1) // odd approach: ingress approaches
        {
            if (fabs(Dis_pre - Dis_curr) < 0.894 / 10) // unit m/0.1s =2mph //Veh in queue is also
                                                       // approaching the intersection, leaving no
                                                       // queue
            {
                veh_state = 3; // in queue
            }
            if (fabs(Dis_pre - Dis_curr) >= 0.894 / 10)
            {
                if (Dis_curr < Dis_pre)
                    veh_state = 1; // approaching
                else
                {
                    veh_state = 2;      // leaving
                    request_phase = -1; // if leaving, no requested phase
                }
            }
        }
        if (match == 1 && approach % 2 == 0) // even approach: engress approaches
        {
            veh_state = 2; // leaving
            request_phase = -1;
        }

        if (match == 0)
        {
            veh_state = 4; // NOT IN THE MAP
            request_phase = -1;
        }

        if (veh_state == 1 || veh_state == 3)
        { // only vehicle approaching intersection need to do something
            request_phase = m_laneNodePhaseMapping[t_App][t_Lane][t_Node];

            // sprintf(temp_log,"Request Phase is %d \n",request_phase);
            // outputlog(temp_log); cout<<temp_log;
            // cout<<"Request Phase is "<<request_phase;
            // cout<<" Current distance to stopbar is "<<Dis_curr;
            // calculate estimated travel time to stop bar
            if (Speed < 1)
                est_TT = 0; // if the vehicle is in queue, assume ETA is 0
            else
                est_TT = Dis_curr / Speed;
            if (est_TT > 9999)
                est_TT = 9999;
            // sprintf(temp_log,"Dis_Stop_bar %f ETA %f Req_Phase %d appr %d lane %d\n
            // \n",Dis_curr,est_TT,request_phase,approach,lane);
            // outputlog(temp_log); //cout<<temp_log;

            // cout<<" TT to stop bar is "<<est_TT<<endl;
        }
        else // Veh in State 2 or 4
        {
            est_TT = 99999;
            Dis_curr = 99999;
        }
    }
}

void MmitssTrajectoryAware::tick()
{
    ITSAPP_TRACE("tick start..started: %d (0: not started yet, 1: started)", m_started);
    if (m_started)
        return;
    // When loading from mmitss config files or mib is ready
    if (m_delayedInit.valid() && m_delayedInit.wait_for(1ms) == std::future_status::ready)
    {
        if (m_delayedInit.get() == true)
        {
            if (m_timerId == 0)
            {
                m_timerConn = siekdbus::timerCreate(m_timerId, m_timerInterval, true,
                    sigc::mem_fun(*this, &MmitssTrajectoryAware::onTimer));
            }
            else
            {
                ITSAPP_TRACE("Setting interval %i", m_timerInterval);
                siekdbus::timerSetInterval(m_timerId, m_timerInterval);
                siekdbus::timerSetEnabled(m_timerId, true);
            }
            if (m_udpReceiver.start() == false)
            {
                ITSAPP_ERROR("m_udpReceiver started failed.");
                return;
            }
            ITSAPP_TRACE("Started successfully..");
            m_started = true;
        }
    }
    ITSAPP_TRACE("tick end..started: %d (0: not started yet, 1: started)", m_started);
}

bool MmitssTrajectoryAware::loadMmitssConfig()
{
    ITSAPP_TRACE("loadMmitssConfig start..");

    // read the new map from the .nmap file and save to NewMap;

    m_newMap.ID = 1;
    m_newMap.Version = 1;
    // Parse the MAP file
    if (!m_newMap.ParseIntersection(SieMmitss::getMapFilePath().c_str()))
        return false;

    ITSAPP_LOG("Read the map successfully (%ld)", time(NULL));

    m_refPoint.init(m_newMap.intersection.Ref_Long, m_newMap.intersection.Ref_Lat,
        m_newMap.intersection.Ref_Ele / 10);

    ITSAPP_LOG("MAP Reference point: %lf %lf %lf ", m_newMap.intersection.Ref_Lat,
        m_newMap.intersection.Ref_Long, m_newMap.intersection.Ref_Ele / 10);

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 20; k++)
            {
                m_laneNodePhaseMapping[i][j][k] = 0;
            }
        }
    }

    // store all nodes information to m_mapNodes after parsing the message
    // This is used for calculating vehicle positions in the MAP
    for (size_t i = 0; i < m_newMap.intersection.Approaches.size(); i++)
        for (size_t j = 0; j < m_newMap.intersection.Approaches[i].Lanes.size(); j++)
            for (size_t k = 0; k < m_newMap.intersection.Approaches[i].Lanes[j].Nodes.size(); k++)
            {
                LaneNodes temp_node;
                temp_node.index.Approach =
                    m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Approach;
                temp_node.index.Lane =
                    m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Lane;
                temp_node.index.Node =
                    m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Node;
                temp_node.Latitude = m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].Latitude;
                temp_node.Longitude =
                    m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].Longitude;
                m_refPoint.lla2ecef(temp_node.Longitude, temp_node.Latitude,
                    m_newMap.intersection.Ref_Ele / 10, &m_ecefX, &m_ecefY, &m_ecefZ);
                m_refPoint.ecef2local(m_ecefX, m_ecefY, m_ecefZ, &m_localX, &m_localY, &m_localZ);
                temp_node.N_Offset = m_localX;
                temp_node.E_Offset = m_localY;

                m_mapNodes.push_back(temp_node);

                ITSAPP_TRACE("%d %d %d %lf %lf %lf %lf ", temp_node.index.Approach,
                    temp_node.index.Lane, temp_node.index.Node, temp_node.N_Offset,
                    temp_node.E_Offset, temp_node.Latitude, temp_node.Longitude);
            }

    // Construct the lane node phase mapping matrix
    if (!get_lane_phase_mapping())
    {
        ITSAPP_WRN("Coudldn't read lane-phase-mapping");
        return false;
    }

    for (size_t iii = 0; iii < m_mapNodes.size(); iii++)
    {
        int flag = 0;
        for (size_t i = 0; i < m_newMap.intersection.Approaches.size(); i++)
        {
            for (size_t j = 0; j < m_newMap.intersection.Approaches[i].Lanes.size(); j++)
            {
                for (size_t k = 0; k < m_newMap.intersection.Approaches[i].Lanes[j].Nodes.size();
                     k++)
                {
                    if (m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Approach ==
                            m_mapNodes[iii].index.Approach &&
                        m_newMap.intersection.Approaches[i].Lanes[j].Nodes[k].index.Lane ==
                            m_mapNodes[iii].index.Lane)
                    {
                        // determine requesting phase
                        if (m_mapNodes[iii].index.Approach == 1) // south bound
                        {
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[1] ==
                                1) // through
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[0];
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[2] ==
                                1) // left turn
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[1];
                        }
                        if (m_mapNodes[iii].index.Approach == 3)
                        {
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[1] ==
                                1) // through
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[2];
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[2] ==
                                1) // left turn
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[3];
                        }
                        if (m_mapNodes[iii].index.Approach == 5)
                        {
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[1] ==
                                1) // through
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[4];
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[2] ==
                                1) // left turn
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[5];
                        }
                        if (m_mapNodes[iii].index.Approach == 7)
                        {
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[1] ==
                                1) // through
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[6];
                            if (m_newMap.intersection.Approaches[i].Lanes[j].Attributes[2] ==
                                1) // left turn
                                m_laneNodePhaseMapping[m_mapNodes[iii].index.Approach]
                                                      [m_mapNodes[iii].index.Lane]
                                                      [m_mapNodes[iii].index.Node] =
                                                          m_phaseMapping[7];
                        }
                        flag = 1;
                        break;
                    }
                }
                if (flag == 1)
                    break;
            }
            if (flag == 1)
                break;
        }
    }

    ITSAPP_LOG("Lane Nodes Headings:");
    int tmp_pos = 0;
    for (unsigned int i = 0; i < m_newMap.intersection.Approaches.size(); i++)
        for (unsigned int j = 0; j < m_newMap.intersection.Approaches[i].Lanes.size(); j++)
            for (unsigned k = 0; k < m_newMap.intersection.Approaches[i].Lanes[j].Nodes.size(); k++)
            {
                if (m_mapNodes[tmp_pos].index.Approach % 2 == 1) // odd approaches, approching lanes
                {
                    if (k < m_newMap.intersection.Approaches[i].Lanes[j].Nodes.size() - 1)
                    {
                        m_mapNodes[tmp_pos].Heading =
                            atan2(m_mapNodes[tmp_pos].N_Offset - m_mapNodes[tmp_pos + 1].N_Offset,
                                m_mapNodes[tmp_pos].E_Offset - m_mapNodes[tmp_pos + 1].E_Offset) *
                            180.0 / M_PI;
                        m_mapNodes[tmp_pos].Heading = 90.0f - m_mapNodes[tmp_pos].Heading;
                    }
                    else
                    {
                        m_mapNodes[tmp_pos].Heading = m_mapNodes[tmp_pos - 1].Heading;
                    }
                    ITSAPP_LOG("Approaching heading %d is %f", tmp_pos,
                        (double)m_mapNodes[tmp_pos].Heading);
                }
                if (m_mapNodes[tmp_pos].index.Approach % 2 == 0) // even approaches, leaving lanes
                {
                    if (k < m_newMap.intersection.Approaches[i].Lanes[j].Nodes.size() - 1)
                    {
                        m_mapNodes[tmp_pos].Heading =
                            atan2(m_mapNodes[tmp_pos + 1].N_Offset - m_mapNodes[tmp_pos].N_Offset,
                                m_mapNodes[tmp_pos + 1].E_Offset - m_mapNodes[tmp_pos].E_Offset) *
                            180.0 / M_PI;
                        m_mapNodes[tmp_pos].Heading = 90.0f - m_mapNodes[tmp_pos].Heading;
                    }
                    else
                    {
                        m_mapNodes[tmp_pos].Heading = m_mapNodes[tmp_pos - 1].Heading;
                    }
                    ITSAPP_LOG(
                        "Leaving heading %d is %f", tmp_pos, (double)m_mapNodes[tmp_pos].Heading);
                }

                if (m_mapNodes[tmp_pos].Heading < 0)
                    m_mapNodes[tmp_pos].Heading += 360;
                tmp_pos++;
            }

    // read the DSRC ranges and central rectangle attributes
    if (!get_DSRC_Range())
        return false;

    m_beginTime = time(NULL);

    ITSAPP_LOG("The format of the veh traj data is: ID Lat Long Speed Heading GPS_Time");

    ITSAPP_TRACE("loadMmitssConfig end..started: %d (0: not started yet, 1: started)", m_started);
    return true;
}

void MmitssTrajectoryAware::packTrajData(char* tmp_traj_data, int& size)
{
    int offset = 0;
    char* pByte; // pointer used (by cast)to get at each byte
                 // of the shorts, longs, and blobs
    unsigned short tempUShort;
    long tempLong;

    // header 2 bytes
    tmp_traj_data[offset++] = 0xFF;
    tmp_traj_data[offset++] = 0xFF;
    // MSG ID: 0x01 for trajectory data send to performance observer
    tmp_traj_data[offset++] = 0x01;
    // No. of Vehicles in the list
    int temp_veh_No = 0;
    for (const auto& veh : m_trackedVeh)
        if (veh.processed == 1)
            temp_veh_No++;

    tempUShort = (unsigned short)temp_veh_No;

    // cout<<"send list number is: "<<tempUShort<<endl;
    // cout<<"total list number is: "<<m_trackedVeh.ListSize()<<endl;

    pByte = (char*)&tempUShort;
    tmp_traj_data[offset++] = *(pByte + 1);
    tmp_traj_data[offset++] = *(pByte + 0);
    // for each vehicle
    auto veh = m_trackedVeh.begin();
    while (veh != m_trackedVeh.end())
    {
        if (veh->processed == 1)
        {
            // vehicle temporary id
            tempUShort = (unsigned short)veh->TempID;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // the number of trajectory points: nFrame
            tempUShort = (unsigned short)veh->nFrame;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Speed of the vehicle
            tempLong = (long)(veh->Speed * DEG2ASNunits); // to 1/10th micro degees units
            pByte = (char*)&tempLong;
            tmp_traj_data[offset++] = *(pByte + 3);
            tmp_traj_data[offset++] = *(pByte + 2);
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // entry time
            tempLong = (long)(veh->entry_time);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset++] = *(pByte + 3);
            tmp_traj_data[offset++] = *(pByte + 2);
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // leaving time
            tempLong = (long)(veh->leaving_time);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset++] = *(pByte + 3);
            tmp_traj_data[offset++] = *(pByte + 2);
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Added by Shayan for PerformanceObserver 08122014

            // vehicle approach
            tempUShort = (unsigned short)veh->approach;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // vehicle lane
            tempUShort = (unsigned short)veh->lane;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Distance To Stop Bar
            tempLong = (long)(veh->stopBarDistance * DEG2ASNunits); // to 1/10th micro degees units
            pByte = (char*)&tempLong;
            tmp_traj_data[offset++] = *(pByte + 3);
            tmp_traj_data[offset++] = *(pByte + 2);
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // vehicle stop_flag
            tempUShort = (unsigned short)veh->stop_flag;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // vehicle queue_flag
            tempUShort = (unsigned short)veh->queue_flag;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Time when vehicle stops
            tempLong = (long)(veh->time_stop);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset++] = *(pByte + 3);
            tmp_traj_data[offset++] = *(pByte + 2);
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Previous approach of vehicle
            tempUShort = (unsigned short)veh->previous_approach;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // Previous lane of vehicle
            tempUShort = (unsigned short)veh->previous_lane;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // reqPhase
            tempUShort = (unsigned short)veh->req_phase;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset++] = *(pByte + 1);
            tmp_traj_data[offset++] = *(pByte + 0);

            // do each trajectory point
            // this following order: latitude,longitude,N_Offset,E_Offset
            // get index from this rotary array
            for (int k = 2; k > 0; k--)
            {
                int j =
                    (veh->nFrame - k + ConnectedVehicle::MAX_FRAMES) % ConnectedVehicle::MAX_FRAMES;
                /*
                //latitude
                tempLong = (long)(veh->traj[j].latitude * DEG2ASNunits);
                pByte = (byte* ) &tempLong;
                tmp_traj_data[offset+0] = (byte) *(pByte + 3);
                tmp_traj_data[offset+1] = (byte) *(pByte + 2);
                tmp_traj_data[offset+2] = (byte) *(pByte + 1);
                tmp_traj_data[offset+3] = (byte) *(pByte + 0);
                offset = offset + 4;
                //longitude
                tempLong = (long)(veh->traj[j].longitude * DEG2ASNunits); // to 1/10th micro degees
                units
                pByte = (byte* ) &tempLong;
                tmp_traj_data[offset+0] = (byte) *(pByte + 3);
                tmp_traj_data[offset+1] = (byte) *(pByte + 2);
                tmp_traj_data[offset+2] = (byte) *(pByte + 1);
                tmp_traj_data[offset+3] = (byte) *(pByte + 0);
                offset = offset + 4;
                */
                // N_Offset
                tempLong = (long)(veh->N_Offset[j] * DEG2ASNunits); // to 1/10th micro degees units
                pByte = (char*)&tempLong;
                tmp_traj_data[offset++] = *(pByte + 3);
                tmp_traj_data[offset++] = *(pByte + 2);
                tmp_traj_data[offset++] = *(pByte + 1);
                tmp_traj_data[offset++] = *(pByte + 0);
                // E_Offset
                tempLong = (long)(veh->E_Offset[j] * DEG2ASNunits); // to 1/10th micro degees units
                pByte = (char*)&tempLong;
                tmp_traj_data[offset++] = *(pByte + 3);
                tmp_traj_data[offset++] = *(pByte + 2);
                tmp_traj_data[offset++] = *(pByte + 1);
                tmp_traj_data[offset++] = *(pByte + 0);
                // cout<<"East offset "<<j<<" :"<<veh->E_Offset[j]<<endl;
            }
        }
        veh++;
    }
    size = offset;
    ITSAPP_TRACE("size: %d", size);
}

void MmitssTrajectoryAware::packTrajData1(char* tmp_traj_data, int& size)
{
    int offset = 0;
    char* pByte; // pointer used (by cast)to get at each byte
                 // of the shorts, longs, and blobs
    unsigned short tempUShort;
    long tempLong;
    // header 2 bytes
    tmp_traj_data[offset] = 0xFF;
    offset += 1;
    tmp_traj_data[offset] = 0xFF;
    offset += 1;
    // MSG ID: 0x01 for trajectory data send to traffic control
    tmp_traj_data[offset] = 0x02;
    offset += 1;
    // No. of Vehicles in the list
    int temp_veh_No = 0;
    for (const auto& veh : m_trackedVeh)
        if (veh.processed == 1)
            temp_veh_No++;

    tempUShort = (unsigned short)temp_veh_No;

    // cout<<"send list number is: "<<tempUShort<<endl;
    // cout<<"total list number is: "<<m_trackedVeh.ListSize()<<endl;

    pByte = (char*)&tempUShort;
    tmp_traj_data[offset + 0] = *(pByte + 1);
    tmp_traj_data[offset + 1] = *(pByte + 0);
    offset = offset + 2;
    // for each vehicle
    auto veh = m_trackedVeh.begin();
    while (veh != m_trackedVeh.end())
    {
        if (veh->processed == 1)
        {
            // vehicle temporary id
            tempUShort = (unsigned short)veh->TempID;
            pByte = (char*)&tempUShort;
            tmp_traj_data[offset + 0] = *(pByte + 1);
            tmp_traj_data[offset + 1] = *(pByte + 0);
            offset = offset + 2;

            // vehicle current speed
            tempLong = (long)(veh->Speed * DEG2ASNunits);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset + 0] = *(pByte + 3);
            tmp_traj_data[offset + 1] = *(pByte + 2);
            tmp_traj_data[offset + 2] = *(pByte + 1);
            tmp_traj_data[offset + 3] = *(pByte + 0);
            offset = offset + 4;
            // vehicle current distance to stop bar
            tempLong = (long)(veh->stopBarDistance * DEG2ASNunits);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset + 0] = *(pByte + 3);
            tmp_traj_data[offset + 1] = *(pByte + 2);
            tmp_traj_data[offset + 2] = *(pByte + 1);
            tmp_traj_data[offset + 3] = *(pByte + 0);
            offset = offset + 4;
            // vehicle current acceleration
            tempLong = (long)(veh->acceleration * DEG2ASNunits);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset + 0] = *(pByte + 3);
            tmp_traj_data[offset + 1] = *(pByte + 2);
            tmp_traj_data[offset + 2] = *(pByte + 1);
            tmp_traj_data[offset + 3] = *(pByte + 0);
            offset = offset + 4;
            // approach
            tmp_traj_data[offset] = (char)veh->approach;
            offset = offset + 1; // move past to next item
            // lane
            tmp_traj_data[offset] = (char)veh->lane;
            offset = offset + 1; // move past to next item
            // req_phase
            tmp_traj_data[offset] = (char)veh->req_phase;
            offset = offset + 1; // move past to next item
            // vehicle first stopped time (absolute time)
            tempLong = (long)(veh->time_stop);
            pByte = (char*)&tempLong;
            tmp_traj_data[offset + 0] = *(pByte + 3);
            tmp_traj_data[offset + 1] = *(pByte + 2);
            tmp_traj_data[offset + 2] = *(pByte + 1);
            tmp_traj_data[offset + 3] = *(pByte + 0);
            offset = offset + 4;
        }
        veh++;
    }
    size = offset;
}

void DistanceFromLine(double cx,
    double cy,
    double ax,
    double ay,
    double bx,
    double by,
    double& distanceSegment,
    double& distanceLine)
{
    //
    // find the distance from the point (cx,cy) to the line
    // determined by the points (ax,ay) and (bx,by)
    //
    // distanceSegment = distance from the point to the line segment
    // distanceLine = distance from the point to the line (assuming
    //                    infinite extent in both directions
    //

    /*
    How do I find the distance from a point to a line?
        Let the point be C (Cx,Cy) and the line be AB (Ax,Ay) to (Bx,By).
        Let P be the point of perpendicular projection of C on AB.  The parameter
        r, which indicates P's position along AB, is computed by the dot product
        of AC and AB divided by the square of the length of AB:

        (1)     AC dot AB
            r = ---------
                ||AB||^2

        r has the following meaning:
            r=0      P = A
            r=1      P = B
            r<0      P is on the backward extension of AB
            r>1      P is on the forward extension of AB
            0<r<1    P is interior to AB

        The length of a line segment in d dimensions, AB is computed by:

            L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 + ... + (Bd-Ad)^2)

        so in 2D:

            L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 )

        and the dot product of two vectors in d dimensions, U dot V is computed:

            D = (Ux * Vx) + (Uy * Vy) + ... + (Ud * Vd)

        so in 2D:

            D = (Ux * Vx) + (Uy * Vy)

        So (1) expands to:

                (Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
            r = -------------------------------
                              L^2

        The point P can then be found:

            Px = Ax + r(Bx-Ax)
            Py = Ay + r(By-Ay)

        And the distance from A to P = r*L.

        Use another parameter s to indicate the location along PC, with the
        following meaning:
               s<0      C is left of AB
               s>0      C is right of AB
               s=0      C is on AB

        Compute s as follows:

                (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
            s = -----------------------------
                            L^2

        Then the distance from C to P = |s|*L.
    */

    double r_numerator = (cx - ax) * (bx - ax) + (cy - ay) * (by - ay);
    double r_denomenator = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
    double r = r_numerator / r_denomenator;
    double s = ((ay - cy) * (bx - ax) - (ax - cx) * (by - ay)) / r_denomenator;

    distanceLine = fabs(s) * sqrt(r_denomenator);

    if ((r >= 0) && (r <= 1))
    {
        distanceSegment = distanceLine;
    }
    else
    {

        double dist1 = (cx - ax) * (cx - ax) + (cy - ay) * (cy - ay);
        double dist2 = (cx - bx) * (cx - bx) + (cy - by) * (cy - by);
        if (dist1 < dist2)
        {
            distanceSegment = sqrt(dist1);
        }
        else
        {
            distanceSegment = sqrt(dist2);
        }
    }

    return;
}

} // namespace Mmitss
} // namespace WaveApp
