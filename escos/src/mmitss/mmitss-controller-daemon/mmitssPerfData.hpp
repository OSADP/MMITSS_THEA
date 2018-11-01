/*
 * mmitssPerfData.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include "stdheader.h"
#include "SieMmitssConfig.hpp"
#include "udpReceiver.hpp"
#include "mmitssConfig.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

class MmitssPerfData
{
  public:
    MmitssPerfData(const MmitssConfig& cfg, its::ItsFacilityApplication& parent);

    bool start();
    void stop();

    MmitssPerfData(const MmitssPerfData&) = delete;
    MmitssPerfData& operator=(const MmitssPerfData&) = delete;
    virtual ~MmitssPerfData() = default;

  private:
    void onPerfDataReceive(const void* data, size_t len);

    UdpReceiver m_dataReceiver;
    const MmitssConfig& m_config;
    its::ItsFacilityApplication& m_parent;
};

} // namespace MmitssCtrl
} // namespace WaveApp
