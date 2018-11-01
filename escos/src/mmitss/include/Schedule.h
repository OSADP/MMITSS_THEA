/*NOTICE:  Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/*  Schedule.h
 *  Created by Yihegn Feng on 2/19/15
 *  University of Arizona
 *  ATLAS Research Center
 *  College of Engineering
 *
 *  This code was develop under the supervision of Professor Larry Head
 *  in the ATLAS Research Center.
 *
 *  Revision History:
 *
 *
 */
#pragma once

#include "Mib.h"

enum ScheduleAction : int
{
    ForceOff = 0,
    Omit = 1,
    VehCall = 2,
    Hold = 3,
    PedCall = 4,
    PedClear = 5
};

class Schedule
{
  public:
    double time; // time point for the schedule
    int phase;   // which phase do we operate
    int action;  // FORCE_OFF; HOLD; CALL; OMIT
};
