/*
 * serviceControl.cpp
 *
 *  Created on: May 25, 2017
 *      Author: devel
 */

#include "mmitssServiceControl.hpp"
#include "stdheader.h"
#include "facilityLayer.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

static const std::string TRAJ_AWARE_SRV = "mmitss_trajectory_aware.service";
static const std::string PERF_OBSERVER_SRV = "mmitss_perf_observer.service";
static const std::string PRIO_REQ_SRV = "mmitss_prio_req.service";
static const std::string PRIO_SOLVER_SRV = "mmitss_prio_solver.service";
static const std::string TRAF_CTRL_SRV = "mmitss_traffic_control.service";
static const std::string ISIG_SRV = "mmitss_isig.service";

static const std::vector<std::pair<std::string, int>> MMITSS_SERVICES = {
    std::make_pair(PRIO_SOLVER_SRV, Priority),
    std::make_pair(TRAJ_AWARE_SRV, Passive | Priority | ISIG),
    std::make_pair(PERF_OBSERVER_SRV, Passive | Priority | ISIG),
    std::make_pair(PRIO_REQ_SRV, Priority), std::make_pair(TRAF_CTRL_SRV, Priority | ISIG),
    std::make_pair(ISIG_SRV, ISIG)};

bool MmitssServiceControl::start(MmitssOperatingMode mode)
{
    m_running = true;
    m_completeStop = false;
    ITSAPP_TRACE("start MMITSS services with mode:%d.", mode);
    for (auto& s : MMITSS_SERVICES)
    {
        if (s.second & mode)
        {
            ITSAPP_TRACE("Start MMITSS service: %s", s.first.c_str());
            if (!startService(s.first))
            {
                ITSAPP_WRN("MMITSS service: %s cannot be started", s.first.c_str());
                stop();
                m_running = false;
                return m_running;
            }
        }
    }
    return m_running;
}

bool MmitssServiceControl::stop()
{
    m_running = false;
    for (const auto& s : MMITSS_SERVICES)
    {
        ITSAPP_TRACE("stop MMITSS service: %s", s.first.c_str());
        bool ret = stopService(s.first);
        if (!ret)
        {
            ITSAPP_WRN("Couldn't stop %s", s.first.c_str());
            return false;
        }
    }
    m_completeStop = true;
    return true;
}

bool MmitssServiceControl::startService(const std::string& service)
{
    return siekdbus::systemd_start_unit(service.c_str());
}

bool MmitssServiceControl::MmitssServiceControl::stopService(const std::string& service)
{
    return siekdbus::systemd_stop_unit(service.c_str());
}

bool MmitssServiceControl::getUnit(const std::string& service, siekdbus::UnitInfo& unit) const
{
    for (const auto& u : m_services)
    {
        if (u.id == service)
        {
            unit = u;
            return true;
        }
    }
    return false;
}

Poco::DynamicStruct MmitssServiceControl::getJsonStatus()
{
    Poco::DynamicStruct status;
    int numActive = 0;
    if (readServiceState())
    {
        Poco::DynamicStruct services;
        for (const auto& s : MMITSS_SERVICES)
        {
            siekdbus::UnitInfo unit;
            if (getUnit(s.first, unit))
            {
                services[s.first] =
                    (unit.active_state != "active") ? "inactive" : unit.active_state;
                if (unit.active_state == "active")
                    numActive++;
            }
            else
            {
                services[s.first] = "inactive";
                ITSAPP_DBG("Couldn't get unit state for %s", s.c_str());
            }
        }
        status["services"] = services;
    }
    status["running"] = m_running;
    status["num_active"] = numActive;
    status["num_inactive"] = MMITSS_SERVICES.size() - numActive;
    return status;
}

SERVICE_STATE MmitssServiceControl::getStatus(MmitssOperatingMode mode, std::string& status)
{
    SERVICE_STATE state = SERVICE_STATE::UNKNOWN;
    int numActive = 0;
    if (readServiceState())
    {
        std::string result;
        for (const auto& s : MMITSS_SERVICES)
        {
            siekdbus::UnitInfo unit;
            if (getUnit(s.first, unit))
            {
                status += s.first + std::string(":") + unit.active_state + std::string("\n");
                if (unit.active_state == "active")
                    numActive++;
                else if (m_running && (s.second & mode))
                    state = SERVICE_STATE::CRITICAL;
            }
        }
        if (m_running && (state == SERVICE_STATE::UNKNOWN))
            state = SERVICE_STATE::OK;
    }
    if (state == SERVICE_STATE::UNKNOWN)
    {
        status += std::string("Disabled");
    }
    else if (state == SERVICE_STATE::CRITICAL)
    {
        status += std::string("Error: ") + std::to_string(MMITSS_SERVICES.size() - numActive) +
                  std::string(" services inactive");
    }
    else
    {
        status += std::string("OK ") + std::to_string(numActive) + std::string(" services running");
    }
    m_state = state;
    return m_state;
}

bool MmitssServiceControl::readServiceState()
{
    m_services.clear();
    if (!siekdbus::systemd_list_units(m_services))
    {
        ITSAPP_WRN("Failed to read unit status!");
        return false;
    }
    return true;
}

} // namespace MmitssCtrl
} // namespace WaveApp
