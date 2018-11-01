/*
 * mmitssMap.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "stdheader.h"
#include "stdtypes.h"

#include "mmitssMap.hpp"
#include "geolib.hpp"
#include "facilityLayer.hpp"
#include "asnHelper.hpp"
#include <sstream>

#include "SieMmitssConfig.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

const std::string SEP = " ";

static void writeCfgVal(std::ostringstream& out, std::string key)
{
    out << key << "\n";
}

template <class T>
static void writeCfgVal(std::ostringstream& out, std::string key, T value)
{
    out << key << SEP << value << "\n";
}

template <class T>
static void writeCfgVal(
    std::ostringstream& out, std::string key, std::initializer_list<T> valueList)
{
    out << key;
    for (auto elem : valueList)
        out << SEP << elem;
    out << "\n";
}

static std::string makeLaneAttributesMmitss(asncsaeLaneLists* lane)
{
    std::ostringstream out;

    int attr = 0;
    if (lane->laneAttributes->directionalUse.getBit(saeegressPath))
        attr |= 1;
    if (lane->maneuvers.getBit(saemaneuverStraightAllowed))
        attr |= 2;
    if (lane->maneuvers.getBit(saemaneuverLeftAllowed))
        attr |= 4;
    if (lane->maneuvers.getBit(saemaneuverRightAllowed))
        attr |= 8;
    if (lane->maneuvers.getBit(saeyieldAllwaysRequired))
        attr |= 16;
    if (!lane->maneuvers.getBit(saemaneuverUTurnAllowed))
        attr |= 32;
    if (!lane->maneuvers.getBit(saemaneuverRightTurnOnRedAllowed))
        attr |= 64;
    if (lane->maneuvers.getBit(saemaneuverNoStoppingAllowed))
        attr |= 128;
    asncsaeLaneTypeAttributes__vehicle* vehAttr =
        static_cast<asncsaeLaneTypeAttributes__vehicle*>(lane->laneAttributes->laneType);
    if (vehAttr->vehicle.getBit(saehovLaneUseOnly))
        attr |= 1024;
    if (vehAttr->vehicle.getBit(saerestrictedToTaxiUse) &&
        vehAttr->vehicle.getBit(saerestrictedToBusUse))
        attr |= 4096;
    else if (vehAttr->vehicle.getBit(saerestrictedToBusUse))
        attr |= 2048;
    for (int i = 0; i < 16; i++)
    {
        if (attr & (1 << (15 - i)))
            out << "1";
        else
            out << "0";
    }
    return out.str();
}

template <class T>
static void processNodes(
    std::ostringstream& out, std::string key, std::initializer_list<T> valueList)
{
    out << key;
    for (auto elem : valueList)
        out << SEP << elem;
    out << "\n";
}

static void getXYFromNode(saeNodeOffsetPointXY__C* nodeOffset, int& x, int& y)
{
    if (nodeOffset->alternative == asn_saenode_XY1)
    {
        x = static_cast<asncsaenode_XY1*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY1*>(nodeOffset)->y;
    }
    else if (nodeOffset->alternative == asn_saenode_XY2)
    {
        x = static_cast<asncsaenode_XY2*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY2*>(nodeOffset)->y;
    }
    else if (nodeOffset->alternative == asn_saenode_XY3)
    {
        x = static_cast<asncsaenode_XY3*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY3*>(nodeOffset)->y;
    }
    else if (nodeOffset->alternative == asn_saenode_XY4)
    {
        x = static_cast<asncsaenode_XY4*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY4*>(nodeOffset)->y;
    }
    else if (nodeOffset->alternative == asn_saenode_XY5)
    {
        x = static_cast<asncsaenode_XY5*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY5*>(nodeOffset)->y;
    }
    else if (nodeOffset->alternative == asn_saenode_XY6)
    {
        x = static_cast<asncsaenode_XY6*>(nodeOffset)->x;
        y = static_cast<asncsaenode_XY6*>(nodeOffset)->y;
    }
    else
    {
        x = -1;
        y = -1;
    }
}

static std::string getManeuverCodeFromManeuver(saeConnectingLane* connect)
{
    if (connect->maneuver.getBit(saemaneuverUTurnAllowed))
    {
        return "1";
    }
    if (connect->maneuver.getBit(saemaneuverLeftAllowed))
    {
        return "2";
    }
    if (connect->maneuver.getBit(saemaneuverRightAllowed))
    {
        return "3";
    }
    if (connect->maneuver.getBit(saemaneuverStraightAllowed))
    {
        return "4";
    }
    if (connect->maneuver.getBit(saemaneuverLeftTurnOnRedAllowed))
    {
        return "5";
    }
    if (connect->maneuver.getBit(saemaneuverRightTurnOnRedAllowed))
    {
        return "6";
    }
    return "";
}

bool MmitssMap::load(const MapPtr& map)
{
    bool valid = true;
    m_saeMap = map;
    m_approaches.clear();
    if (m_saeMap->intersections.count > 0)
    {
        asncsaeIntersectionGeometryLists* intersec = static_cast<asncsaeIntersectionGeometryLists*>(
            m_saeMap->intersections.getFirstElement());

        // get all lanes for each approach
        while (1)
        {
            int curApprNr = m_approaches.size() + 1;
            const int MAX_APPROACHS = 8;
            if (curApprNr > MAX_APPROACHS)
                break;
            std::vector<asncsaeLaneLists*> app;
            asncsaeLaneLists* lane =
                static_cast<asncsaeLaneLists*>(intersec->laneSet.getFirstElement());
            for (Int32u i = 0; i < intersec->laneSet.count; i++)
            {
                if (lane->laneAttributes->laneType->alternative ==
                    asn_saeLaneTypeAttributes__vehicle)
                { // only vehicle lanes
                    if (lane->optional.getPresence(asn_saeegressApproach))
                    { // egress
                        if (lane->egressApproach <= 0)
                        {
                            ITSAPP_WRN("Invalid egress approach number (%d) for lane %d",
                                lane->egressApproach, lane->laneID);
                            app.clear();
                            m_approaches.clear();
                            break;
                        }
                        else if (lane->egressApproach == curApprNr)
                        {
                            app.emplace_back(lane);
                        }
                    }
                    else if (lane->optional.getPresence(asn_saeingressApproach))
                    { // ingress
                        if (lane->ingressApproach <= 0)
                        {
                            ITSAPP_WRN("Invalid ingress approach number (%d) for lane %d",
                                lane->ingressApproach, lane->laneID);
                            app.clear();
                            m_approaches.clear();
                            break;
                        }
                        else if (lane->ingressApproach == curApprNr)
                        {
                            app.emplace_back(lane);
                        }
                    }
                }
                lane = static_cast<asncsaeLaneLists*>(lane->getNextElement());
            }
            m_approaches.emplace_back(app); // app might be empty if it's missing in map
        }
        ITSAPP_LOG("Approaches read %d", m_approaches.size());
        if (m_approaches.empty())
        {
            valid = false;
        }
    }
    m_saeMapValid = valid;
    return valid;
}

bool MmitssMap::loadNmap()
{
    return m_nmap.ParseIntersection(SieMmitss::getMapFilePath().c_str());
}

bool MmitssMap::isMmitssConformant() const
{
    // TODO
    return false;
}

int MmitssMap::getLaneId(int approach, int lane) const
{
    int ret = -1;
    if (m_saeMapValid)
    {
        if ((approach > 0) && (approach <= (int)m_approaches.size()))
        {
            if ((lane > 0) && (lane <= (int)m_approaches[approach - 1].size()))
            {
                return m_approaches[approach - 1][lane - 1]->laneID;
            }
        }
    }
    else
    {
        return m_nmap.getLaneID(approach - 1, lane - 1);
    }
    return ret;
}

int MmitssMap::getApproachId(int laneId) const
{
    int appr = -1;
    if (laneId > 0)
    {
        for (const auto& app : m_approaches)
        {
            for (const auto& l : app)
            {
                if (l->laneID == laneId)
                {
                    if (l->optional.getPresence(asn_saeegressApproach))
                    { // egress
                        return l->egressApproach;
                    }
                    if (l->optional.getPresence(asn_saeingressApproach))
                    { // ingress
                        return l->ingressApproach;
                    }
                }
            }
        }
    }
    return appr;
}

std::string MmitssMap::toInOutMapping()
{
    std::ostringstream inOut;
    if (m_saeMap->intersections.count > 0)
    {
        auto intersec = static_cast<asncsaeIntersectionGeometryLists*>(
            m_saeMap->intersections.getFirstElement());

        writeCfgVal(inOut, "IntersectionID", intersec->id->id);

        std::map<int, std::string> laneIds;
        int curApprNr = 1;
        int no_approach = 0;

        std::vector<asnMAXSINT> laneIdsOfAppr;
        for (const auto& appr : m_approaches)
        {
            if (not appr.empty())
                no_approach++;
            int laneNr = 1;
            laneIdsOfAppr.clear();
            for (const auto& lane : appr)
                laneIdsOfAppr.push_back(lane->laneID);
            std::sort(laneIdsOfAppr.begin(), laneIdsOfAppr.end());
            for (const auto& id : laneIdsOfAppr)
            {
                laneIds[id] =
                    std::to_string(curApprNr).append(".").append(std::to_string(laneNr++));
            }
            curApprNr++;
        }

        writeCfgVal(inOut, "No_Approach", no_approach);

        StringList approaches;
        std::map<int, int> sgs;
        int numIngress = 0;
        curApprNr = 1;
        for (const auto& appr : m_approaches)
        {
            std::list<std::string> curAppr;
            for (const auto& lane : appr)
            {
                Approach ingress;
                if ((lane->ingressApproach > 0) && (lane->optional.getPresence(asn_saeconnectsTo)))
                {
                    auto conn =
                        static_cast<asncsaeConnectsToLists*>(lane->connectsTo.getFirstElement());
                    for (size_t i = 0; i < lane->connectsTo.getCount(); i++)
                    {
                        if (conn->optional.getPresence(asn_saesignalGroup))
                        {
                            curAppr.emplace_back(laneIds[lane->laneID] + SEP +
                                                 laneIds[conn->connectingLane->lane] + SEP +
                                                 std::to_string(conn->signalGroup));
                            sgs[conn->signalGroup] = 0;
                        }
                        conn = static_cast<asncsaeConnectsToLists*>(conn->getNextElement());
                    }
                }
            }
            if (!curAppr.empty())
            {
                numIngress++;
                curAppr.emplace_front(
                    std::string("No_Lane") + SEP + std::to_string(curAppr.size()));
                curAppr.emplace_front(std::string("Approach") + SEP + std::to_string(curApprNr));
                curAppr.emplace_back(std::string("end_Approach"));
            }
            for (auto s : curAppr)
                approaches.emplace_back(s);
            curApprNr++;
        }
        writeCfgVal(inOut, "No_Phase", sgs.size());
        writeCfgVal(inOut, "No_Ingress", numIngress);
        for (auto s : approaches)
            inOut << s << "\n";
    }
    return inOut.str();
}

std::string MmitssMap::toNMAP(std::string& mapName)
{
    std::ostringstream nmap;
    if (m_saeMap->intersections.count > 0)
    {
        try
        {
            auto intersec = static_cast<asncsaeIntersectionGeometryLists*>(
                m_saeMap->intersections.getFirstElement());

            std::string intersecName = AsnHelper::octetStringToString(intersec->name);
            intersecName = intersecName.empty() ? "map" : intersecName;
            std::string nmapFileName = intersecName + std::string(".nmap");
            mapName = nmapFileName;
            // intersection values
            writeCfgVal(nmap, "MAP_Name", nmapFileName);
            writeCfgVal(nmap, "RSU_ID", intersecName);
            writeCfgVal(nmap, "IntersectionID", intersec->id->id);
            writeCfgVal(nmap, "Intersection_attributes", "00000000"); // TODO
            writeCfgVal(nmap, "Reference_point",
                {AsnHelper::latLonToString(intersec->refPoint->lat),
                    AsnHelper::latLonToString(intersec->refPoint->sae__long),
                    std::to_string(intersec->refPoint->elevation)});

            std::map<int, std::string> laneIds;
            int curApprNr = 1;
            std::vector<asnMAXSINT> laneIdsOfAppr;
            int no_approach = 0;
            for (const auto& appr : m_approaches)
            {
                if (not appr.empty())
                    no_approach++;
                int laneNr = 1;
                laneIdsOfAppr.clear();
                for (const auto& lane : appr)
                    laneIdsOfAppr.push_back(lane->laneID);
                std::sort(laneIdsOfAppr.begin(), laneIdsOfAppr.end());
                for (const auto& id : laneIdsOfAppr)
                {
                    laneIds[id] =
                        std::to_string(curApprNr).append(".").append(std::to_string(laneNr++));
                    ITSAPP_TRACE("laneIds[%d] is set to %s", id, laneIds[id].c_str());
                }
                curApprNr++;
            }

            writeCfgVal(nmap, "No_Approach", no_approach);

            for (size_t i = 0; i < m_approaches.size(); i++)
            {
                if (m_approaches[i].empty())
                    continue;
                int apprId = i + 1;
                writeCfgVal(nmap, "Approach", apprId);
                int apprType = m_approaches[i][0]->ingressApproach
                                   ? 1
                                   : m_approaches[i][0]->egressApproach ? 2 : -1;
                writeCfgVal(nmap, "Approach_type", apprType);
                writeCfgVal(nmap, "No_lane", m_approaches[i].size());
                std::map<asnMAXSINT, Int32u> laneIdIndexTable;
                laneIdIndexTable.clear();
                for (Int32u laneIdx = 0; laneIdx < m_approaches[i].size(); laneIdx++)
                {
                    laneIdIndexTable[m_approaches[i][laneIdx]->laneID] = laneIdx;
                }
                for (const auto& mapping : laneIdIndexTable)
                {
                    Int32u laneIdx = mapping.second;
                    asncsaeLaneLists* lane = m_approaches[i][laneIdx];
                    std::string lane_num = laneIds.at(lane->laneID);
                    ITSAPP_TRACE(
                        "lane_num for lane->laneID:% is %s", lane->laneID, lane_num.c_str());
                    writeCfgVal(nmap, "Lane", lane_num);
                    writeCfgVal(nmap, "Lane_ID", lane->laneID);
                    writeCfgVal(nmap, "Lane_type", 1); // only vehicle lanes supported
                    writeCfgVal(nmap, "Lane_attributes", makeLaneAttributesMmitss(lane));
                    writeCfgVal(
                        nmap, "Lane_width", intersec->laneWidth); // use default lane width only

                    // node list
                    if (lane->nodeList->alternative == asn_saeNodeListXY__nodes)
                    {
                        asncsaeNodeListXY__nodes* nodeList =
                            static_cast<asncsaeNodeListXY__nodes*>(lane->nodeList);
                        writeCfgVal(nmap, "No_nodes", nodeList->nodes.count);
                        sielib::GeoLatLonInt refPointNodeXy;
                        refPointNodeXy.fromHostLatLon(
                            intersec->refPoint->lat, intersec->refPoint->sae__long);

                        // process node offset values
                        asncsaeNodeSetXYs* node =
                            static_cast<asncsaeNodeSetXYs*>(nodeList->nodes.firstElement);

                        int accumulatedX = 0, accumulatedY = 0;
                        sielib::GeoLatLonInt curNodeXy;

                        for (Int32u nodeIdx = 0; nodeIdx < nodeList->nodes.count; nodeIdx++)
                        {
                            std::string nodeId = lane_num + "." + std::to_string(nodeIdx + 1);
                            if (node->delta->alternative == asn_saeNodeOffsetPointXY__node_LatLon)
                            {
                                asncsaeNodeOffsetPointXY__node_LatLon* delta =
                                    static_cast<asncsaeNodeOffsetPointXY__node_LatLon*>(
                                        node->delta);
                                writeCfgVal(nmap, nodeId,
                                    {AsnHelper::latLonToString(delta->lat),
                                        AsnHelper::latLonToString(delta->lon)});
                                // store lat lon
                                //                                curNodeXy.fromHostLatLon(delta->lat,
                                //                                delta->lon);
                            }
                            else
                            {
                                try
                                {
                                    int x, y;
                                    getXYFromNode(node->delta, x, y);
                                    accumulatedX += x;
                                    accumulatedY += y;
                                    sielib::GeoLatLonDouble nextPos;
                                    geoCalcDestinationLatLonFromXY(refPointNodeXy.toLatLonDouble(),
                                        nextPos, static_cast<double>(accumulatedX) / 100,
                                        static_cast<double>(accumulatedY) / 100);
                                    curNodeXy = nextPos.toLatLonInt();
                                    writeCfgVal(nmap, nodeId,
                                        {AsnHelper::latLonToString(curNodeXy.lat),
                                            AsnHelper::latLonToString(curNodeXy.lon)});
                                }
                                catch (const std::exception& e)
                                {
                                    ITSAPP_ERROR(
                                        "map config error: lane node calculation failed- %s",
                                        e.what());
                                    return std::string();
                                }
                            }
                            node = static_cast<asncsaeNodeSetXYs*>(node->nextElement);
                        }
                    }
                    else if (lane->nodeList->alternative == asn_saecomputed) // not supported
                    {
                        ITSAPP_ERROR("config error: MAP computed lanes not supported for MMITSS!");
                        return std::string();
                    }
                    // connecting lanes
                    writeCfgVal(nmap, "No_Conn_lane", lane->connectsTo.count);
                    asncsaeConnectsToLists* con =
                        static_cast<asncsaeConnectsToLists*>(lane->connectsTo.getFirstElement());
                    for (Int32u conIdx = 0; conIdx < lane->connectsTo.count; conIdx++)
                    {
                        ITSAPP_TRACE("connect to lane:%d, which is %s", con->connectingLane->lane,
                            laneIds.at(con->connectingLane->lane).c_str());
                        writeCfgVal(nmap, laneIds.at(con->connectingLane->lane),
                            getManeuverCodeFromManeuver(con->connectingLane));
                        con = static_cast<asncsaeConnectsToLists*>(con->nextElement);
                    }
                    writeCfgVal(nmap, "end_lane");
                }
                writeCfgVal(nmap, "end_approach");
            }
            writeCfgVal(nmap, "end_map");
        }
        catch (const out_of_range& e)
        {
            ITSAPP_WRN("Out of range error on map read: %s", e.what());
        }
    }
    return nmap.str();
}

void MmitssMap::geoCalcDestinationLatLonFromXY(const sielib::GeoLatLonDouble& posA,
    sielib::GeoLatLonDouble& posB,
    double xInMeters,
    double yInMeters)
{
    /*
     * refer to SAE_J2945 standard Appendix A.2 for below algorithm
     */
    double RefLat = sielib::degreeToRad(posA.lat);
    double RefLon = sielib::degreeToRad(posA.lon);

    double a = 6378137;                       // semi-major axis of earth
    double f = 1 / 298.257223563;             // flattening
    double f1 = std::pow((f * (2 - f)), 0.5); // eccentricity
    double f2 = a * (1 - std::pow(f1, 2)) /
                std::pow((1 - std::pow(f1, 2) * std::pow((sin(RefLat)), 2)),
                    (3 / 2)); // radius of earth in meridian
    double f3 = a / std::pow((1 - std::pow(f1, 2) * std::pow((sin(RefLat)), 2)),
                        (1 / 2)); // radius of earth in prime vertical

    posB.lat = sielib::radToDegree((1 / f2) * yInMeters + RefLat);
    posB.lon = sielib::radToDegree((1 / (f3 * std::cos(RefLat))) * xInMeters + RefLon);
}

} // namespace MmitssCtrl
} // namespace WaveApp
