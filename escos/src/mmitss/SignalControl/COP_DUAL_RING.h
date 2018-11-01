#pragma once

#include "Mib.h"

namespace WaveApp
{
namespace Mmitss
{

class MmitssSignalControl;

using std::array;

/**
 *  CopDualRing class
 */
class CopDualRing
{
  public:
    CopDualRing(MmitssSignalControl& parent);
    virtual ~CopDualRing() = default;
    CopDualRing(const CopDualRing&) = delete;
    CopDualRing& operator=(const CopDualRing&) = delete;

    int COP_DUAL_RING(int InitPhase[2], int GrnElapse[2], int PassedPhase[2], int Skip_phase[8]);
    // calculate the delay when planning two phases at the first stage
    float f(int ring,
        int phase1,
        int phase2,
        int g1,
        int g2,
        int total_length,
        int Num_Veh[8],
        float Num_Ratio[8]);
    // calculate the delay when planning two phases at the following
    // stages the last parameter is the length of this planning barrier
    float f1(int ring,
        int phase1,
        int phase2,
        int g1,
        int g2,
        int total_length,
        int B_L,
        int stage,
        int Num_Veh[8],
        float Num_Ratio[8]);
    // calculate the delay when planning one phase at the first stage
    float f_r(int ring,
        int phase1,
        int phase2,
        int g1,
        int g2,
        int total_length,
        int Num_Veh[8],
        float Num_Ratio[8]);
    // calculate the delay when planning one phase at the following stages
    float f1_r(int ring,
        int phase1,
        int phase2,
        int g1,
        int g2,
        int total_length,
        int B_L,
        int stage,
        int Num_Veh[8],
        float Num_Ratio[8]);
    // calculate the delay of ring other than phase1 and phase2
    float f_other(
        int ring, int phase1, int phase2, int total_length, int Num_Veh[8], float Num_Ratio[8]);

    float f1_other(int ring,
        int phase1,
        int phase2,
        int total_length,
        int B_L,
        int stage,
        int Num_Veh[8],
        float Num_Ratio[8]);

    int add_missing_phase(int barrier);

  private:
    /*  number of barriers/stages planned  */
    static const int NUM_BARRIERS = 2;
    /*  number of phases  */
    static const int NUM_PHASES = 8;
    /*  total planned time +1  */
    static const int TOTAL_TIME = 120;

    MmitssSignalControl& m_parent;

    const MmitssMib::PhaseState& Ped_Phase_Considered;
    array<int, 8>& MaxGreen;
    array<int, 8>& MinGreen;
    array<array<int, 8>, 131>& ArrivalTable;
    int& delay_type;
    int& queue_warning_P1;
    int& queue_warning_P5;
    array<int, 8>& PedWalk;
    array<int, 8>& PedClr;
    array<array<int, 4>, 2>& opt_sig_plan;
    array<array<int, 4>, 2>& opt_sig_seq;
    array<int, 8>& LaneNo;
    array<double, 8>& red_elapse_time;

    array<int, 8>& yellow;
    array<int, 8>& red;

    /*table of values for barrier level*/
    float v[NUM_BARRIERS][TOTAL_TIME];
    //*table of decisions for barrier level*
    int X[NUM_BARRIERS][TOTAL_TIME];
    // minimum time of each barrier/stage
    int B_Min[NUM_BARRIERS];
    // maximum time of each barrier/stage
    int B_Max[NUM_BARRIERS];

    // planned phase in one barrier, two rings
    int planned_phase[2][2];
    // stage * maximum possible barrier length * ring1 * ring2
    int best_phase_schedule[NUM_BARRIERS][TOTAL_TIME][2][2];
    int best_phase_sequence[NUM_BARRIERS][TOTAL_TIME][2][2];
    // temporary standing queue at each stage
    int Q_temp[TOTAL_TIME][NUM_PHASES];
    // permanent standing queue at each stage
    int Q_perm[NUM_BARRIERS][TOTAL_TIME][NUM_PHASES];
};

} // namespace Mmitss
} // namespace WaveApp
