//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

/* IntLanePhase.h
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

#pragma once

#include <string>

#ifndef MAX_APRROACH
#define MAX_APRROACH 10
#endif
#ifndef MAX_LANE
#define MAX_LANE 10
#endif

class IntLanePhase
{
  public:
    int iIntersectionID;
    int iTotalApproaches;
    int iTotalPhases;
    int iTotalIngress;
    int iApproach;
    int iNoRow;
    int iInLane;
    int iOutLane;
    int iPhase;
    int iOutApproach;
    int iInLaneOutLanePhase[MAX_APRROACH][MAX_LANE][MAX_APRROACH][MAX_LANE];

    // Methods
    bool ReadLanePhaseMap(const std::string& filename);
    int findThePhaseOfInLine(int ilane);
};
