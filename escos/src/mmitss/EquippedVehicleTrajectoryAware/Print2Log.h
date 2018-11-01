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
#include "LinkedList.h"
#include "GetInfo.h"
#include "SieMmitss.hpp"

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
    MMITSS_LOG("%s", tmp_log);
}

void PrintFile2Log(const std::string& resultsfile)
{
    fstream fss;
    fss.open(resultsfile, fstream::in);

    if (!fss)
    {
        MMITSS_ERROR("***********Error opening the plan file in order to print to a log file!\n");
        exit(1);
    }
    string lineread;

    while (!fss.eof())
    {
        getline(fss, lineread);
        MMITSS_LOG("%s", lineread.c_str());
    }
    fss.close();
}
