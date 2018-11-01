/*
 * spatMapProxy.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include "facilityLayer.hpp"
#include "facilityBase.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

using MapPtr = std::shared_ptr<saeMapData>;

class SpatMapProxy
{
  public:
    SpatMapProxy(its::ItsFacilityApplication& parent)
        : m_parent(parent){};
    SpatMapProxy() = delete;
    virtual ~SpatMapProxy() = default;
    SpatMapProxy(const SpatMapProxy&) = delete;
    SpatMapProxy& operator=(const SpatMapProxy&) = delete;

    void readMap();
    void clearMap();

    MapPtr getMap() const { return m_map; };

  private:
    its::ItsFacilityApplication& m_parent;

    MapPtr m_map;
};

} // namespace MmitssCtrl
} // namespace WaveApp
