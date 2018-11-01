/*
 * mmitssConfig.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include "facilityLayer.hpp"
#include "facilityBase.hpp"
#include "spatMapProxy.hpp"
#include "mmitssMap.hpp"
#include "Poco/DynamicStruct.h"

namespace WaveApp
{
namespace MmitssCtrl
{

class MmitssConfig
{
  public:
    static const int MAX_CONFIG_FILE_SIZE = 65536; // limit config file size to 64kb

    MmitssConfig(its::ItsFacilityApplication& parent);
    MmitssConfig() = delete;

    bool updateConfig(MapPtr saeMap);

    bool isValid() const { return m_isValid; };

    bool writeConfigFile(const std::string& filename, const std::string& content);
    std::string readConfigFile(const std::string& filename);

    std::string getStatus() const;
    Poco::DynamicStruct getStatusJson();
    Poco::DynamicStruct getCurrentConfigFilesJson();
    Poco::DynamicStruct getConfigFileStatusJson(std::string filename);
    int getSignalPlan();

    StringList getUserConfigFiles() const;
    bool isPresent(const std::string& file) const;

    const MmitssMap& getMmitssMap() const { return m_mmitssMap; };

  private:
    using ConfigFiles = std::unordered_map<std::string, bool>;

    static std::string saeMapToNmap(const saeMapData& saeMap, std::string& mapName);

    bool isUserConfigFile(const std::string& filename);
    bool isAutoConfigFile(const std::string& filename);

    void writeLogId();

    bool checkConfig();

    bool m_isValid{false};

    std::string m_mapStatus;
    MmitssMap m_mmitssMap;
    ConfigFiles m_userCfgFiles; // these can be changed/loaded by the user
    ConfigFiles m_autoCfgFiles; // these are auto generated
    its::ItsFacilityApplication& m_parent;
};

} // namespace MmitssCtrl
} // namespace WaveApp
