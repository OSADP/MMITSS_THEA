/*
 * mmitssServiceControl.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include "stdheader.h"
#include "siesystemd.hpp"
#include "Poco/Dynamic/Struct.h"
#include "stdtypes.h"

namespace WaveApp
{
namespace MmitssCtrl
{

enum MmitssOperatingMode : int
{
    Priority = 1,
    ISIG = 2,
    Passive = 4
};

class MmitssServiceControl
{
  public:
    MmitssServiceControl() = default;

    MmitssServiceControl(const MmitssServiceControl&) = delete;
    MmitssServiceControl& operator=(const MmitssServiceControl&) = delete;
    virtual ~MmitssServiceControl() = default;

    bool start(MmitssOperatingMode mode);
    bool stop();

    bool startService(const std::string& service);
    bool stopService(const std::string& service);

    SERVICE_STATE getStatus(MmitssOperatingMode mode, std::string& status);
    Poco::DynamicStruct getJsonStatus();
    bool completeStop() { return m_completeStop; };

  private:
    bool readServiceState();
    bool getUnit(const std::string& service, siekdbus::UnitInfo& unit) const;

    using Services = siekdbus::UnitList;

    SERVICE_STATE m_state{SERVICE_STATE::UNKNOWN};
    bool m_running{false};
    bool m_completeStop{false};
    Services m_services;
};

} // namespace MmitssCtrl
} // namespace WaveApp
