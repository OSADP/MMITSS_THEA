//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/* IntLanePhase.cpp
 * Created by Mehdi Zamanipour
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 */

#include "IntLanePhase.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

#include "facilityLayer.hpp"

#define DEFAULT_ARRAY_SIZE 80

bool IntLanePhase::ReadLanePhaseMap(const std::string& filename)
{
    memset(iInLaneOutLanePhase, 0, sizeof(iInLaneOutLanePhase));

    std::ifstream lanePhase_file;
    lanePhase_file.open(filename);

    if (!lanePhase_file)
    {
        ITSAPP_WRN("Error: Open file %s", filename.c_str());
        return false;
    }

    std::string lineread;
    char token_char[DEFAULT_ARRAY_SIZE];
    char temp_char[DEFAULT_ARRAY_SIZE];

    int i = 0;
    for (i = 0; i < 4; i++)
    {
        getline(lanePhase_file, lineread); // Read line by line
        if (lineread.size() && (lineread.size() < DEFAULT_ARRAY_SIZE))
        {
            sscanf(lineread.c_str(), "%s", token_char);
            // token.assign(token_char);
            if (strcmp(token_char, "IntersectionID") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                iIntersectionID = atoi(temp_char);
            }
            else if (strcmp(token_char, "No_Approach") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                iTotalApproaches = atoi(temp_char);
            }
            else if (strcmp(token_char, "No_Phase") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                iTotalPhases = atoi(temp_char);
            }
            else if (strcmp(token_char, "No_Ingress") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                iTotalIngress = atoi(temp_char);
            }
        }
    }

    for (i = 0; i < iTotalIngress; i++)
    {
        getline(lanePhase_file, lineread); // Read line by line
        if (lineread.size() && (lineread.size() < DEFAULT_ARRAY_SIZE))
        {
            sscanf(lineread.c_str(), "%s", token_char);
            if (strcmp(token_char, "Approach") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                iApproach = atoi(temp_char);
                getline(lanePhase_file, lineread);
                sscanf(lineread.c_str(), "%s", token_char);
                if (strcmp(token_char, "No_Lane") == 0)
                {
                    sscanf(lineread.c_str(), "%*s %s", temp_char);
                    iNoRow = atoi(temp_char);
                    for (int j = 0; j < iNoRow; j++)
                    {
                        getline(lanePhase_file, lineread);
                        sscanf(lineread.c_str(), "%s", token_char);
                        iInLane = token_char[2] - 48;
                        sscanf(lineread.c_str(), "%*s %s", token_char);
                        iOutApproach = token_char[0] - 48;
                        iOutLane = token_char[2] - 48;
                        sscanf(lineread.c_str(), "%*s %*s %s", temp_char);
                        iPhase = atoi(temp_char);
                        iInLaneOutLanePhase[iApproach][iInLane][iOutApproach][iOutLane] = iPhase;
                        ITSAPP_WRN("iInLaneOutLanePhase[%d][%d][%d][%d] = %d", iApproach, iInLane,
                            iOutApproach, iOutLane, iPhase);
                    }
                }
            }
        }
        getline(lanePhase_file, lineread); // Read line by line
        if (lineread.size() && (lineread.size() < DEFAULT_ARRAY_SIZE))
        {
            sscanf(lineread.c_str(), "%s", token_char);
        }
    }

    getline(lanePhase_file, lineread); // Read line by line
    if (lineread.size() && (lineread.size() < DEFAULT_ARRAY_SIZE))
    {
        sscanf(lineread.c_str(), "%s", token_char);
        if (strcmp(token_char, "end_map") == 0)
        {
            ITSAPP_TRACE("Read Lane Phase Mapping file successfully!");
        }
    }
    return true;
}
