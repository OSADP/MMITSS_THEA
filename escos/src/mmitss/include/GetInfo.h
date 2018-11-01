#pragma once

#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <istream>
#include <math.h>

#include "Signal.h"

#include "SieMmitss.hpp"
#include "SieMmitssConfig.hpp"
#include "Poco/Net/SocketAddress.h"

using namespace std;

#define NumPhases 8

//---Store the file name of the Config file.
//--- For Example: ConfigInfo_Hastings.txt, which has the minGrn and maxGrn etc.
extern int CombinedPhase[8];

std::string get_rsu_id() // DJ add 06/24/2011
{
    fstream fs;
    fs.open(SieMmitss::getRsuIdFilePath().c_str());
    std::string id;
    getline(fs, id);
    fs.close();

    if (id.size() != 0)
    {
        MMITSS_LOG("Current RSU ID %s", id.c_str());
    }
    else
    {
        MMITSS_LOG("Reading RSU ID %s problem", SieMmitss::getRsuIdFilePath().c_str());
    }
    return id;
}

Poco::Net::SocketAddress get_ip_address()
{
    fstream fs;
    fs.open(SieMmitss::getIpInfoFilePath().c_str());

    std::string ip, port;
    getline(fs, ip);
    getline(fs, port);
    fs.close();
    if ((!ip.empty()) && (!port.empty()))
    {
        MMITSS_LOG("Controller IP Address is: %s:%s", ip.c_str(), port.c_str());
    }
    else
    {
        MMITSS_LOG("Reading IPinfo file %s problem", SieMmitss::getIpInfoFilePath().c_str());
        ip.clear();
        port.clear();
    }
    return Poco::Net::SocketAddress(ip, port);
}
