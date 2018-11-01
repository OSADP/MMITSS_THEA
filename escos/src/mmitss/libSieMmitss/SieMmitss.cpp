/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitss.cpp
 *
 *  Created on: May 19, 2017
 *      Author: M.Venus
 */

#include "SieMmitss.hpp"
#include "SieMmitssConfig.hpp"
#include <fstream>

namespace SieMmitss
{

int mmitssAppLogId = -1;

int getAppLogId()
{
    if (mmitssAppLogId != -1)
        return mmitssAppLogId;
    std::fstream logId(getSlogIdFilePath().c_str(), std::ios_base::in);
    logId >> mmitssAppLogId;
    return mmitssAppLogId;
}

} // namespace SieMmitss
