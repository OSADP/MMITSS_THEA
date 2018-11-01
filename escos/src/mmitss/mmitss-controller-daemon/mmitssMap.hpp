/*
 * mmitssMap.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include "facilityLayer.hpp"
#include "facilityBase.hpp"
#include "NMAP.h"

namespace WaveApp
{
namespace MmitssCtrl
{

class MmitssMap
{
  public:
    using MapPtr = std::shared_ptr<saeMapData>;

    MmitssMap() = default;

    bool load(const MapPtr& map);
    bool loadNmap();

    bool isMmitssConformant() const;

    /**
     * Get the saeMap LaneID
     * @param approach mmitss approach index
     * @param lane mmitss lane idx
     * @return LaneID as in saeMap, or -1 if not found
     */
    int getLaneId(int approach, int lane) const;

    /**
     * Get approachId for
     * @param laneId lane identifier as in sae MAP
     */
    int getApproachId(int laneId) const;

    /**
     * Convert to MMITSS NMAP format
     * @return NMAP string
     */
    std::string toNMAP(std::string& mapName);

    /**
     * Convert to MMITSS InLane_OutLane_Phase_Mapping format
     * @return InLane_OutLane_Phase_Mapping string
     */
    std::string toInOutMapping();

  private:
    using Approach = std::vector<asncsaeLaneLists*>;
    using Approaches = std::vector<Approach>;

    void geoCalcDestinationLatLonFromXY(const sielib::GeoLatLonDouble& posA,
        sielib::GeoLatLonDouble& posB,
        double xInMeters,
        double yInMeters);

    Approaches m_approaches;
    MapPtr m_saeMap;
    MAP m_nmap;
    bool m_saeMapValid{false};
};

} // namespace MmitssCtrl
} // namespace WaveApp
