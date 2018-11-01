#pragma once

#include <array>
#include <vector>

#include "ConnectedVehicle.h"

namespace WaveApp
{
namespace Mmitss
{

using ConnectedVehicleList = std::vector<ConnectedVehicle>;
// Define Parameters for car following model
extern float jam_length;    // minimal headway of a vehicle (AX)
extern float acc_diff_thre; // Acceleration difference threshold
extern float desired_spd;   // vehicle desired speed (ffs)   m/s = 50km/h
extern float lamda;         // parameter for estimating speed
extern float veh_length;    // unified vehicle length

// sorted list by Lane No and distance to stop bar, inserted vehicle will be addd to this list
// extern ConnectedVehicleList sorted_vehlist;
// Sort the vehicle in the list by lane then distance to the stop bar
void sort_list(ConnectedVehicleList& vehListEachPhase);
// determine state of the vehicle based on Wiedemann car following model
// State definition:   1: free flow    2: emergency    3: closing    4: following
int determine_state(
    float cur_pos, float cur_spd, float cur_acc, float lead_pos, float lead_spd, float lead_acc);

float cal_dec(float speed); // Calculate the deceleration rate based on current spd
// phase_est is the current estimated phase
void EVLS_Prio(ConnectedVehicleList& vehListEachPhase,
    ConnectedVehicleList& trackedveh,
    int phase_est,
    long current_time,
    float red_elapse_time,
    int No_lanes,
    float dsrc_range,
    float penetration);

void EVLS_Isig(ConnectedVehicleList& vehListEachPhase,
    ConnectedVehicleList& trackedveh,
    int phase_est,
    long current_time,
    float red_elapse_time,
    int No_lanes,
    float dsrc_range,
    float penetration,
    std::array<std::array<int, 8>, 131>& arrivalTable);

} // namespace Mmitss
} // namespace WaveApp
