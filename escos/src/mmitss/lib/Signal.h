//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once

#include "Config.h"
#include "PhaseState.h"

namespace WaveApp
{
namespace Mmitss
{

// enum Color {R=1, G=3, Y=4};
#define numPhases 8 // Number of considering phases

enum PhaseColor : int
{
    Red = 1,
    Green = 3,
    Yellow = 4
};

class PhaseStatus
{
  public:
    PhaseStatus()
    {
        for (int i = 0; i < numPhases; i++)
            phaseColor[i] = Red;
    }
    PhaseStatus(const PhaseStatus&) = default;
    PhaseStatus& operator=(const PhaseStatus&) = default;

    int phaseColor[numPhases];
};

double GetSeconds(); // Get the current time: seconds in float point number

char* GetDate(); // Get the current date as: "Sat May 20 15:21:51 2000"

class Phase // current phase status: PhaseRecord in the Windows code
{
  public:
    PhaseStatus Phase_Status[2]; //[0]: for previous; [1]: for current/new
    double StartTime[numPhases];
    int CurPhase[2];  // current timing phase {0-7} in Ring[2]
    int PrePhase[2];  // timing phase of previous time step, used for all red situation added by YF
                      // 01.17.2014
    int InitPhase[2]; // Real phase should +1: Ring 1:0-3; Ring 2:4-7
    double InitTime[2]; // with InitPhase: for GLPK Solver.
    double GrnElapse[2];
    double ColorTime[numPhases];

  public:
    void UpdatePhase(const PhaseStatus& newPhaseStatus, const PhaseState& CurPhaseStatus);
    // TODO: Check what's the difference is to the one above...
    void UpdatePhaseMPR(
        const PhaseStatus& newPhaseStatus, const PhaseState& CurPhaseStatus, double* globalGmax);
    void Display();
    void RecordPhase(char* filename);
    void FindPhaseNext(int nextPhase[2], int curPhase[2], const PhaseState& curPhaseStatus);

    Phase& operator=(Phase& newPhase);
    Phase(const RsuConfig& cfg);
    Phase() = delete;

  private:
    const RsuConfig& m_cfg;
};

} // namespace Mmitss
} // namespace WaveApp
