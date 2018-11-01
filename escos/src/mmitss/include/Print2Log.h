/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* Print2Log.h
*  Created by :Jun Ding
*  University of Arizona
*  ATLAS Research Center
*  College of Engineering
*
*  This code was develop under the supervision of Professor Larry Head
*  in the ATLAS Research Center.

*/

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
#include "Array.h"
#include "GetInfo.h"

#include "facilityLayer.hpp"

namespace WaveApp
{
namespace Mmitss
{

using namespace std;

void PrintPlan2Log(const std::string& resultsfile);
void PrintFile2Log(const std::string& resultsfile);
void PrintPhases2Log(PhaseStatus currentPhase); // Print the current phase status

void PrintPhases2Log(PhaseStatus currentPhase)
{
    char tmp_log[128] = "Phase Information: ";
    char tmp_phase[16];
    for (int i = 0; i < NumPhases; i++)
    {
        sprintf(tmp_phase, " %d", currentPhase.phaseColor[i]);
        strcat(tmp_log, tmp_phase);
    }
    ITSAPP_LOG("%s", tmp_log);
}

void PrintFile2Log(const std::string& resultsfile)
{
    fstream fss;
    fss.open(resultsfile, fstream::in);

    if (!fss)
    {
        ITSAPP_WRN("***********Error opening the plan file in order to print to a log file!");
        return;
    }
    string lineread;

    while (!fss.eof())
    {
        getline(fss, lineread);
        ITSAPP_LOG("%s", lineread.c_str());
    }

    fss.close();
}

void PrintPlan2Log(const std::string& resultsfile)
{
    fstream fss;
    fss.open(resultsfile, fstream::in);

    if (!fss)
    {
        ITSAPP_WRN("***********Error opening the plan file in order to print to a log file!");
        return;
    }

    string lineread;

    while (!fss.eof())
    {
        getline(fss, lineread);
        ITSAPP_LOG("%s", lineread.c_str());
    }
    fss.close();
}

} // namespace Mmitss
} // namespace WaveApp
