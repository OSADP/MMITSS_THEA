/*
 * mmitssConfig.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "stdheader.h"
#include "stdtypes.h"

#include "mmitssConfig.hpp"
#include "facilityLayer.hpp"
#include "facilityDatabaseAdapter.hpp"

#include <fstream>
#include <sstream>
#include "Poco/File.h"
#include "Poco/Path.h"
#include "Poco/DirectoryIterator.h"
#include "SieMmitssConfig.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

using namespace ::SieMmitss;

std::string toAbsolutePath(const std::string& file)
{
    Poco::Path cfgFile(file);
    if (cfgFile.parent().toString() != Poco::Path(getConfigDir()).toString())
    {
        return cfgFile.makeAbsolute(Poco::Path(getConfigDir())).toString();
    }
    return file;
}

MmitssConfig::MmitssConfig(its::ItsFacilityApplication& parent)
    : m_parent(parent)
{
    m_userCfgFiles[getMapFilePath()] = false;
    m_userCfgFiles[getLanePhaseMapFilePath()] = false;
    m_userCfgFiles[getDsrcRangeFilePath()] = false;
    m_userCfgFiles[getDetNumbersFilePath()] = false;
    m_userCfgFiles[getLaneMovementMapFilePath()] = false;
    m_userCfgFiles[getTestSystemFilePath()] = false;
    m_userCfgFiles[getPriorityConfigurationFilePath()] = false;
    m_userCfgFiles[getLaneNumberFilePath()] = false;
    m_userCfgFiles[getSignalConfigCOPFilePath()] = false;
    m_userCfgFiles[getConfigInfoFilePath()] = false;

    m_autoCfgFiles[getMapNameFilePath()] = false;
    m_autoCfgFiles[getIpInfoFilePath()] = false;
    m_autoCfgFiles[getRsuIdFilePath()] = false;
    m_autoCfgFiles[getInLaneOutLanePhaseMappingFilePath()] = false;

    //    m_autoCfgFiles[getConfigInfoFilePath()] = false;

    Poco::File confDir(getConfigDir());
    if (!confDir.exists())
        confDir.createDirectories();
    if (!confDir.exists())
        ITSAPP_ERROR("Couldn't create MMITSS config dir")
    writeLogId();

    checkConfig();
}

void MmitssConfig::writeLogId()
{
    std::ofstream out;
    out.open(getSlogIdFilePath(), std::ofstream::trunc);
    out << its::appLogId;
    out.close();
}

bool MmitssConfig::updateConfig(MapPtr saeMap)
{
    // MAP
    m_mapStatus.clear();
    std::string nmapName = getMapFilePath();
    if (saeMap != nullptr)
    {
        if (m_mmitssMap.load(saeMap))
        {
            std::string nmap = m_mmitssMap.toNMAP(nmapName);
            if (!nmap.empty())
                nmapName = getConfigDir() + nmapName;
            else
                nmapName = "";
            writeConfigFile(getMapFilePath(), nmap);
            writeConfigFile(getInLaneOutLanePhaseMappingFilePath(), m_mmitssMap.toInOutMapping());
        }
        else
        {
            m_mapStatus = "MAP not MMITSS compliant";
        }
    }
    else if (!m_mmitssMap.loadNmap()) // load nmap file
    {
        ITSAPP_WRN("Failed to load NMAP!");
    }
    writeConfigFile(getMapNameFilePath(), getMapFilePath());

    // IPInfo
    int port = 0;
    std::string address;
    ITSAPP_VERIFY(m_parent.database().read("its/us/asc/config/port", port));
    ITSAPP_VERIFY(m_parent.database().read("its/us/asc/config/addr", address));
    if ((!address.empty()) && (port > 0))
        address += std::string("\n") + std::to_string(port);
    writeConfigFile(getIpInfoFilePath(), address);
    std::string rsuid;
    ITSAPP_VERIFY(m_parent.database().read("system/mgmt_id", rsuid));
    writeConfigFile(getRsuIdFilePath(), rsuid);

    return checkConfig();
}

bool MmitssConfig::writeConfigFile(const std::string& filename, const std::string& content)
{
    bool ret = false;
    Poco::Path cfgFile(filename);
    // store nmap file as map.nmap always
    if (cfgFile.getExtension() == "nmap")
        cfgFile = getMapFilePath();
    std::string cfg = toAbsolutePath(cfgFile.getFileName());
    if ((isUserConfigFile(cfg)) || isAutoConfigFile(cfg))
    {
        if (content.empty())
        {
            std::remove(cfg.c_str());
        }
        else
        {
            std::ofstream out(cfg);
            out << content;
            out.close();
        }
        ret = true;
    }
    checkConfig();
    return ret;
}

std::string MmitssConfig::readConfigFile(const std::string& filename)
{
    try
    {
        std::ifstream ifs(toAbsolutePath(filename));
        std::stringstream str;
        str << ifs.rdbuf();
        return str.str();
    }
    catch (const std::exception& e)
    {
        ITSAPP_WRN("Error reading file %s: %s", filename.c_str(), e.what());
    }
    return "";
}

bool MmitssConfig::checkConfig()
{
    m_isValid = true;
    try
    {
        for (auto& f : m_userCfgFiles)
        {
            m_userCfgFiles.at(f.first) = Poco::File(f.first).exists();
            // ignore test system config file
            if ((f.first != SieMmitss::getTestSystemFilePath()) && !m_userCfgFiles.at(f.first))
            {
                m_isValid = false;
                ITSAPP_WRN("configuration file does not exist: %s", f.first.c_str());
            }
        }
        for (auto& f : m_autoCfgFiles)
        {
            m_autoCfgFiles.at(f.first) = Poco::File(f.first).exists();
            if (!m_autoCfgFiles.at(f.first))
            {
                m_isValid = false;
                ITSAPP_WRN("generated file does not exist: %s", f.first.c_str());
            }
        }
    }
    catch (const std::out_of_range& e)
    {
        ITSAPP_WRN("Out of range: %s", e.what())
        m_isValid = false;
    }

    return m_isValid;
}

Poco::DynamicStruct MmitssConfig::getStatusJson()
{
    Poco::DynamicStruct status;
    try
    {
        for (auto& f : m_userCfgFiles)
        {
            if (f.first == SieMmitss::getMapFilePath())
            {
                bool generated = false;
                ITSAPP_VERIFY(
                    m_parent.database().read("its/us/mmitss/control/generate_nmap", generated));
                status[(generated ? "(generated)" : "") + Poco::Path(f.first).getFileName()] =
                    f.second;
            }
            else if (f.first != SieMmitss::getTestSystemFilePath())
                status[Poco::Path(f.first).getFileName()] = f.second;
        }
        status["(generated)" + Poco::Path(getIpInfoFilePath()).getFileName()] =
            m_autoCfgFiles.at(getIpInfoFilePath());
        if (!m_mapStatus.empty())
        {
            status["map_status"] = m_mapStatus;
        }
    }
    catch (const std::out_of_range& e)
    {
        ITSAPP_WRN("Out of range: %s", e.what())
    }

    return status;
}

Poco::DynamicStruct MmitssConfig::getCurrentConfigFilesJson()
{
    Poco::DynamicStruct result;
    Poco::Dynamic::Array files;

    Poco::DirectoryIterator it(getConfigDir());
    Poco::DirectoryIterator end;
    while (it != end)
    {
        if (it->isFile())
            files.emplace_back(it.name());
        ++it;
    }

    result["curConfigFiles"] = files;
    return result;
}

int MmitssConfig::getSignalPlan()
{
    int sigPlan = -1;
    ITSAPP_VERIFY(m_parent.database().read("its/us/asc/oids/cur_timing_plan", sigPlan));
    return sigPlan;
}

Poco::DynamicStruct MmitssConfig::getConfigFileStatusJson(std::string filename)
{
    Poco::DynamicStruct result;
    Poco::File f(toAbsolutePath(filename));
    if (f.exists())
    {
        Poco::LocalDateTime ldt(f.getLastModified());
        result[filename] =
            Poco::DateTimeFormatter::format(ldt, Poco::DateTimeFormat::SORTABLE_FORMAT);
    }
    return result;
}

std::string MmitssConfig::getStatus() const
{
    std::string status;
    for (const auto& f : m_userCfgFiles)
    {
        status += f.first + std::string(" - ") +
                  (f.second ? std::string("OK") : std::string("NOT FOUND")) + std::string("\n");
    }
    return status;
}

StringList MmitssConfig::getUserConfigFiles() const
{
    StringList result;
    for (const auto& f : m_userCfgFiles)
        result.emplace_back(Poco::Path(f.first).getFileName());
    return result;
}

bool MmitssConfig::isPresent(const std::string& file) const
{
    return ((m_userCfgFiles.find(toAbsolutePath(file)) != m_userCfgFiles.end()) ||
            (m_autoCfgFiles.find(toAbsolutePath(file)) != m_autoCfgFiles.end()));
}

bool MmitssConfig::isUserConfigFile(const std::string& filename)
{
    return (m_userCfgFiles.find(toAbsolutePath(filename)) != m_userCfgFiles.end());
}

bool MmitssConfig::isAutoConfigFile(const std::string& filename)
{
    return (m_autoCfgFiles.find(toAbsolutePath(filename)) != m_userCfgFiles.end());
}

} // namespace MmitssCtrl
} // namespace WaveApp
