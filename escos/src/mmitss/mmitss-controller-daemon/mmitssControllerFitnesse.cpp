#include "stdheader.h"

#include "mmitssControllerDaemon.hpp"
#include "cmdLineParser.hpp"
#include "fileUtil.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

void MmitssControllerDaemon::onFitSyntax(std::string& msg) const
{
    NOTUSED(msg);
}

bool MmitssControllerDaemon::onFitReset()
{
    return true;
}

bool MmitssControllerDaemon::onFitCommand(
    std::string& result, const char* command, const char* args)
{
    ITSAPP_TRACE("onFitCommand");

    char* argv[64];
    CmdLineParser parser(args);
    int argc = parser.parse(64, argv);
    if (!std::string(command).compare("loadConfigFile"))
    {
        if (argc < 1)
        {
            result = "Missing filename";
            return false;
        }
        std::string content;
        result = "OK";
        ITSAPP_TRACE("Read file: %s", argv[0]);
        if (!FileUtil::readFile(content, argv[0]))
        {
            result = std::string("Fit: Read file error: ").append(argv[0]);
        }
        ITSAPP_TRACE("Write file: %s, %s", argv[0], content.c_str());

        if (!m_mmitssCfg.writeConfigFile(argv[0], content))
        {
            result = std::string("MMITSS write config file error: ").append(argv[0]);
        }
        return true;
    }
    else if (!std::string(command).compare("checkConfig"))
    {
        result = "OK";
        if (!m_mmitssCfg.isValid())
            result = "ERROR";
        return true;
    }
    else
    {
        result.append("invalid argument");
        return false;
    }
    return true;
}

} // namespace MmitssCtrl
} // namespace WaveApp
