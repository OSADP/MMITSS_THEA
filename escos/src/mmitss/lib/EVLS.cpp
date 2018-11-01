/* Copyright 2014 Arizona Board of Regents on behalf of University of Arizona.
 * All information, intellectual, and technical concepts contained herein is and shall
 * remain the proprietary information of Arizona Board of Regents and may be covered
 * by U.S. and Foreign Patents, and patents in process.  Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written permission
 * is obtained from Arizona Board of Regents or University of Arizona.
 */

/* GetInfo.cpp
 * Created by YIHENG FENG
 * University of Arizona
 * ATLAS Research Center
 * College of Engineering
 *
 * This code was develop under the supervision of Professor Larry Head
 * in the ATLAS Research Center.
 *
 * Revision History:
 *
 */

#include "EVLS.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "SieMmitss.hpp"

namespace WaveApp
{
namespace Mmitss
{

// Define Parameters for car following model
float jam_length = 6.56;     // minmum headway of a vehicle (AX)
float acc_diff_thre = 1.96;  // acceleration difference threshold
float desired_spd = 13.8889; // vehicle desired speed (ffs)   m/s = 50km/h
float lamda = 0.162;         // parameter for estimating speed
float veh_length = 4.75;     // unified vehicle length
std::map<std::pair<int, double>, ConnectedVehicle> sorted_vehlist; // sorted list by Lane No and
                                                                   // distance to stop bar, inserted
                                                                   // vehicle will be addd to this
                                                                   // list

void EVLS_Prio(ConnectedVehicleList& vehListEachPhase,
    ConnectedVehicleList& trackedveh,
    int phase_est,
    long current_time,
    float red_elapse_time,
    int No_lanes,
    float dsrc_range,
    float penetration)
{
    NOTUSED(phase_est);

    int i, j;

    ConnectedVehicle temp_veh;

    sorted_vehlist.clear();
    auto veh = sorted_vehlist.begin();

    // other parameters
    // long current_time=1400265597;

    // Note:  always plan at the beginning of the green, so here plan at beginning of 2 and 6, red
    // elapse time of 2 and 6 is the previous red duration
    // int phase_read[8]={1, 3, 1, 1, 1, 3, 1, 1};
    // float red_elapse_time[8]={33.870200, 50.370252, 2.100000, 14.330087, 33.870198, 47.150205,
    // 2.099997, 15.550095};

    // All the vehicle data are from phase 2 now!!

    // Currently it is 50% penetration rate!!
    // int No_lanes=1;  //number of lanes on the approach

    //    MMITSS_LOG("The sorted Vehicle List is:");

    sort_list(vehListEachPhase); // sort list to the correct order

    // print the sorted list

    //    for (const auto& veh_p : sorted_vehlist)
    //    {
    //        MMITSS_LOG(
    //            "%d, %lf, %lf, %lf, %lf", veh_p.second.lane, veh_p.second.stopBarDistance,
    //            veh_p.second.Speed, veh_p.second.acceleration, veh_p.second.time_stop);
    //    }

    // find out how many vehicles in each lane
    int* No_veh_per_lane; // number of vehicle each lane (both equipped and later inserted)
    No_veh_per_lane = new int[No_lanes];

    // lane No and lane mapping
    int* Lane_No_Mapping;
    Lane_No_Mapping = new int[No_lanes];

    int index = 0;
    if (!sorted_vehlist.empty())
    {
        veh = sorted_vehlist.begin();
        int tmp_lane = veh->second.lane, count = 1;
        veh++;
        while (veh != sorted_vehlist.end())
        {
            if (veh->second.lane != tmp_lane)
            {
                No_veh_per_lane[index] = count;
                Lane_No_Mapping[index] = tmp_lane;
                index++;
                tmp_lane = veh->second.lane;
                count = 1;
            }
            else
            {
                count++;
            }
            veh++;
        }
        No_veh_per_lane[index] = count;
        Lane_No_Mapping[index] = tmp_lane;
    }

    //    MMITSS_LOG(
    //        "The lane index mapping to each lane is: %d %d %d", Lane_No_Mapping[0],
    //        Lane_No_Mapping[1], Lane_No_Mapping[2]);
    //    MMITSS_LOG(
    //        "The No. of vehicle per lane is: %d %d %d", No_veh_per_lane[0], No_veh_per_lane[1],
    //        No_veh_per_lane[2]);

    // reduce No_lanes if no vehicle in one lane
    No_lanes = index + 1;

    //    MMITSS_LOG("Reduced lane to: %d", No_lanes);

    // Record how many vehicles are inserted each lane in the slow down region
    int* inserted_each_lane;
    inserted_each_lane = new int[10]; // at most 8 lanes, no need to map the lane id!!

    // Define maximum number of vehicles can be inserted each lane
    int max_insertion;
    if (penetration >= 0.75f)
        max_insertion = 1;
    if (penetration < 0.75f && penetration >= 0.5f)
        max_insertion = 2;
    if (penetration < 0.5f)
        max_insertion = 3;

    int max_q_prop_time = (int)(1.0f / penetration);

    // Step 1: calculate the queuing part for each lane
    float* est_queue_length;
    est_queue_length = new float[No_lanes];

    // number of queued equipped vehicle each lane
    int* No_queue_veh;
    No_queue_veh = new int[No_lanes];

    // Last queued equipped vehicle position
    float* Last_queue_equipped_veh_pos;
    Last_queue_equipped_veh_pos = new float[No_lanes];

    // number of queued vehicle (both equipped and unequipped) each lane
    int* No_queue_veh_all; // The same
    No_queue_veh_all = new int[No_lanes];

    for (i = 0; i < No_lanes; i++)
    {
        No_queue_veh[i] = 0;
    }

    veh = sorted_vehlist.begin();
    for (j = 0; j < No_lanes; j++)
    {
        long time1 =
            current_time - (int)red_elapse_time; // red start time Note: this index is only for
                                                 // phase 6, need to be changed
                                                 // later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        float queue_length1 = 0;
        double queue_speed = 0;
        float queue_length2 = 0;
        long time2 = 0;
        for (i = 0; i < No_veh_per_lane[j]; i++)
        {
            if (veh->second.Speed < 0.1) // stopped vehicle
            {
                queue_length2 = veh->second.stopBarDistance;
                time2 = veh->second.time_stop;
                // Need to deal with the case of over saturation, some vehicle stops before the red
                // beginning at the end of the queue
                if (time2 <= time1)
                    queue_speed = 0;
                else
                    queue_speed = (queue_length2 - queue_length1) / (time2 - time1);
                time1 = time2;
                queue_length1 = queue_length2;
                No_queue_veh[j]++;
            }
            veh++;
        }
        int q_time_after_last_veh;
        if (max_q_prop_time <= current_time - time2)
            q_time_after_last_veh = max_q_prop_time;
        else
            q_time_after_last_veh = current_time - time2;

        est_queue_length[j] = queue_length2 + (float)queue_speed * q_time_after_last_veh;
        Last_queue_equipped_veh_pos[j] = queue_length2;
    }

    //    MMITSS_LOG("Current time is: %ld", current_time);
    //    MMITSS_LOG(
    //        "Estimated queue length of each lane is: %f %f %f\n", est_queue_length[0],
    //        est_queue_length[1], est_queue_length[2]);

    // For queuing part, all we need is the No. of vehicles because all the ETAs of queued vehicle
    // are 0
    /*  Calculate the No. Vehicles in queue based on queue length and add vehicle to sorted_vehlist
     */

    int flag = 0; // This flag is used to identify the position of the inserted vehicle
    for (i = 0; i < No_lanes; i++)
    {
        if (i == 0)
            flag = No_queue_veh[i];
        else // i>0
        {
            flag = 0;
            for (j = 0; j < i; j++)
                flag += No_veh_per_lane[j];
            flag += No_queue_veh[i];
        }
        int No_total_veh_queue = ceil(est_queue_length[i] / jam_length) +
                                 1; //+1 is because of the vehicle at the stop line
        No_queue_veh_all[i] = No_total_veh_queue;
        // Note here the sum of No_queue_veh_all is not equal to the size of sorted_vehlist, because
        // only add vehicle at the last of the queue, not for other gaps!!!!!!!!!!!!!!!!!!!!!!!
        if (est_queue_length[i] - Last_queue_equipped_veh_pos[i] >
            jam_length / 2) // Insert a vehicle at the last of the queue for calculation in the slow
                            // down region
        {
            temp_veh.Speed = 0;
            temp_veh.stopBarDistance = Last_queue_equipped_veh_pos[i] + jam_length;
            temp_veh.lane = Lane_No_Mapping[i];
            temp_veh.acceleration = 0;
            sorted_vehlist[std::make_pair(temp_veh.lane, temp_veh.stopBarDistance)] = temp_veh;
            No_veh_per_lane[i]++;
        }
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Use this No_queue_veh_all value for the No of vehicle for the queuing part for the arrival
    // table
    //    MMITSS_LOG(
    //        "The number of queued vehicle of each lane are (both equipped and unequipped): %d %d
    //        %d", No_queue_veh_all[0], No_queue_veh_all[1], No_queue_veh_all[2]);

    // Step2: Calculate the slow-down part of each lane
    // Define the slow down region
    float slow_region = 50; // 50m for the slow down region temporary, need to change
                            // later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    float ave_queue_length; // average queue length of each lane, since in vissim the queue length
                            // are almost the same for all lanes
    float total_queue_length = 0;
    for (i = 0; i < No_lanes; i++)
    {
        total_queue_length += est_queue_length[i];
    }
    ave_queue_length = total_queue_length / No_lanes;

    if (!sorted_vehlist.empty())
    {
        veh = sorted_vehlist.begin();
        // Assign moving dynamics to leading vehicle
        // We can't insert vehicle for the first vehicle because lack of information
        float speed_pre = veh->second.Speed;
        float acc_pre = veh->second.acceleration;
        float pos_pre = veh->second.stopBarDistance;
        float lane_pre = veh->second.lane;
        veh++;

        while (veh != sorted_vehlist.end())
        {
            if (veh->second.Speed != 0 && veh->second.lane == lane_pre &&
                veh->second.stopBarDistance > (double)ave_queue_length + 3 &&
                veh->second.stopBarDistance <
                    (double)(ave_queue_length + slow_region)) // moving vehicle in the same lane
            {
                // Determine car following state
                int veh_state = determine_state(veh->second.stopBarDistance, veh->second.Speed,
                    veh->second.acceleration, pos_pre, speed_pre, acc_pre);
                // calculate max acc and min acc
                float min_spd =
                    veh->second.Speed <= (double)speed_pre ? (float)veh->second.Speed : speed_pre;
                float max_acc = 3.5f - 3.5f / 40 * min_spd; // m/s
                float min_acc = -20 + 1.5f / 60 * min_spd;

                float delta_x = veh->second.stopBarDistance - (double)pos_pre; // headway
                float AX = jam_length;                                         // min headway
                // float CX=40;
                // float SDV=pow((delta_x-AX)/CX,2);
                // float OPDV=-2.25*SDV;
                float BX = 2.5 * sqrt(min_spd);
                float ABX = AX + 2.5f * (float)sqrt(min_spd);
                float SDX = AX + 3.75f * (float)sqrt(min_spd);
                float delta_v = (float)veh->second.Speed - speed_pre;

                // calculate the desired acceleration based on vehicle state
                float desired_acc;
                if (veh_state == 1) // free flow state
                    desired_acc = std::min(max_acc, desired_spd - (float)veh->second.Speed);
                if (veh_state == 2) // Emergency state
                    desired_acc = 0.5f * (delta_v * delta_v / (AX - delta_x)) + acc_pre +
                                  min_acc * (ABX - delta_x) / BX;
                if (veh_state == 3) // closing state
                {
                    float temp = 0.5f * delta_v * delta_v / (ABX - delta_x) + acc_pre;
                    if (temp >= min_acc)
                        desired_acc = temp;
                    else
                        desired_acc = min_acc;
                }
                if (veh_state == 4) // following state
                    desired_acc = 0;

                // Determin whether to trigger the insertion
                int trigger = 0; // 0: not trigger  1: triggered insertion
                // Trigger one: in free flow region, if acc is two small
                if (veh_state == 1 && desired_acc - (float)veh->second.acceleration > acc_diff_thre)
                    trigger = 1;
                // Trigger two: in closing or following region, and following distance is too long
                if ((veh_state == 3 || veh_state == 4) &&
                    delta_x > 2 * SDX) // here 2 is a parameter for calibration!!!!!!!!!!!!!  modify
                                       // the original model!!!!!!!!!!!!!!!!!!
                    trigger = 1;

                // If triggered insertion
                if (trigger == 1)
                {
                    // Calculate pos, speed and acc of inserted vehicle
                    // speed
                    float tmp_spd = std::max(
                        (float)veh->second.Speed + lamda * (float)veh->second.acceleration, 0.0f);
                    temp_veh.Speed = tmp_spd;

                    // acc
                    // if both of the leading and following vehicle are decelerating
                    if (acc_pre <= 0.0f && veh->second.acceleration <= 0)
                        temp_veh.acceleration = cal_dec(tmp_spd);
                    else
                        temp_veh.acceleration = 0;

                    // position
                    float tmp_delta_x;
                    if (tmp_spd <
                        (float)veh->second.Speed) // the inserted leading vehicle is slower, the
                                                  // current vehicle is in closing region
                    {
                        if (fabs(veh->second.acceleration - temp_veh.acceleration) >
                            0.1) // if the deceleration rate is different
                            tmp_delta_x =
                                7.25 + 3.75 * sqrt(tmp_spd) -
                                0.5 *
                                    pow(std::min((double)-lamda * veh->second.acceleration,
                                            veh->second.Speed),
                                        2) /
                                    (veh->second.acceleration - temp_veh.acceleration);
                        else
                            tmp_delta_x = 7.25 + 3.75 * sqrt(tmp_spd);
                    }
                    else
                        tmp_delta_x = 7.25 + 3.75 * sqrt(tmp_spd);
                    temp_veh.stopBarDistance = veh->second.stopBarDistance - (double)tmp_delta_x;

                    // lane    same lane
                    temp_veh.lane = veh->second.lane;

                    // check the availablity of the inserted vehicle
                    if ((float)temp_veh.stopBarDistance > (pos_pre + jam_length) &&
                        inserted_each_lane[temp_veh.lane] < max_insertion)
                    // if (temp_veh.stopBarDistance>pos_pre+jam_length)
                    {
                        // increase the inserted vehicle number
                        inserted_each_lane[temp_veh.lane]++;

                        // Add the inserted vehicle to the list
                        sorted_vehlist[std::make_pair(temp_veh.lane, temp_veh.stopBarDistance)] =
                            temp_veh;
                        //                        sorted_vehlist.InsertAt(temp_veh);
                        //                        cout<<"Current Vehicle "<<veh->second.lane<<"
                        //                        "<<veh->second.stopBarDistance<<"
                        //                        "<<veh->second.Speed<<"
                        //                        "<<veh->second.acceleration<<"
                        //                        "<<veh->second.time_stop<<endl;
                        //                        int Pos_in_list=sorted_vehlist.CurrentPosition();
                        //                        sorted_vehlist.Reset(Pos_in_list-1);
                    }
                }
            }

            speed_pre = veh->second.Speed;
            acc_pre = veh->second.acceleration;
            pos_pre = veh->second.stopBarDistance;
            lane_pre = veh->second.lane;
            veh++;
        }
    }

    // Step 3: Insert Vehicles in the free flow region
    // Step 3.1 Count No. of veh in free flow region
    int eqp_veh_in_free_region = 0;
    int* eqp_veh_in_fr_lane;
    eqp_veh_in_fr_lane = new int[No_lanes];
    for (i = 0; i < No_lanes; i++)
    {
        eqp_veh_in_fr_lane[i] = 0;
    }

    float total_speed = 0;
    for (const auto& veh_p : sorted_vehlist)
    {
        if ((float)veh_p.second.stopBarDistance >
            ave_queue_length + slow_region) // when vehicles are in free flow region
        {
            eqp_veh_in_free_region++;
            total_speed += (float)veh_p.second.Speed;
            for (i = 0; i < No_lanes; i++)
            {
                if (veh_p.second.lane == Lane_No_Mapping[i])
                    eqp_veh_in_fr_lane[i]++;
            }
        }
    }

    int total_veh_in_free_region = eqp_veh_in_free_region / penetration;
    int veh_ffr_each_lane = total_veh_in_free_region / No_lanes;
    float ave_ffs = total_speed / eqp_veh_in_free_region;

    // Add the identified non connected vehicles into the trackedvehicle
    for (i = 0; i < No_lanes; i++)
    {
        int inserted_No = veh_ffr_each_lane - eqp_veh_in_fr_lane[i];
        for (j = 0; j < inserted_No; j++)
        {
            float distance = (dsrc_range - ave_queue_length - slow_region) / inserted_No;
            // here should come to the end of the list
            temp_veh.lane = Lane_No_Mapping[i];
            temp_veh.stopBarDistance = ave_queue_length + slow_region + distance * j;
            temp_veh.acceleration = 0;
            temp_veh.Speed = ave_ffs;
            trackedveh.emplace_back(temp_veh);
        }
    }

    // print the final vehicle list
    //    MMITSS_LOG("Total No. of Vehicle in the list is %d:", sorted_vehlist.size());

    delete (No_veh_per_lane);
    delete (Lane_No_Mapping);
    delete (inserted_each_lane);
    delete (est_queue_length);
    delete (No_queue_veh);
    delete (No_queue_veh_all);
    delete (eqp_veh_in_fr_lane);
}

void EVLS_Isig(ConnectedVehicleList& vehListEachPhase,
    ConnectedVehicleList& trackedveh,
    int phase_est,
    long current_time,
    float red_elapse_time,
    int No_lanes,
    float dsrc_range,
    float penetration,
    std::array<std::array<int, 8>, 131>& arrivalTable)
{
    NOTUSED(phase_est);

    int i, j;

    ConnectedVehicle temp_veh;

    sorted_vehlist.clear();
    auto veh = sorted_vehlist.begin();

    // other parameters
    // long current_time=1400265597;

    // Note:  always plan at the beginning of the green, so here plan at beginning of 2 and 6, red
    // elapse time of 2 and 6 is the previous red duration
    // int phase_read[8]={1, 3, 1, 1, 1, 3, 1, 1};
    // float red_elapse_time[8]={33.870200, 50.370252, 2.100000, 14.330087, 33.870198, 47.150205,
    // 2.099997, 15.550095};

    // All the vehicle data are from phase 2 now!!

    // Currently it is 50% penetration rate!!
    // int No_lanes=1;  //number of lanes on the approach

    //    MMITSS_LOG("The sorted Vehicle List is:");

    sort_list(vehListEachPhase); // sort list to the correct order

    // print the sorted list

    //    for (const auto& veh_p : sorted_vehlist)
    //    {
    //        MMITSS_LOG(
    //            "%d, %lf, %lf, %lf, %lf", veh_p.second.lane, veh_p.second.stopBarDistance,
    //            veh_p.second.Speed, veh_p.second.acceleration, veh_p.second.time_stop);
    //    }

    // find out how many vehicles in each lane
    int* No_veh_per_lane; // number of vehicle each lane (both equipped and later inserted)
    No_veh_per_lane = new int[No_lanes];

    // lane No and lane mapping
    int* Lane_No_Mapping;
    Lane_No_Mapping = new int[No_lanes];

    int index = 0;
    if (!sorted_vehlist.empty())
    {
        veh = sorted_vehlist.begin();
        int tmp_lane = veh->second.lane, count = 1;
        veh++;
        while (veh != sorted_vehlist.end())
        {
            if (veh->second.lane != tmp_lane)
            {
                No_veh_per_lane[index] = count;
                Lane_No_Mapping[index] = tmp_lane;
                index++;
                tmp_lane = veh->second.lane;
                count = 1;
            }
            else
            {
                count++;
            }
            veh++;
        }
        No_veh_per_lane[index] = count;
        Lane_No_Mapping[index] = tmp_lane;
    }

    //    MMITSS_LOG(
    //        "The lane index mapping to each lane is: %d %d %d", Lane_No_Mapping[0],
    //        Lane_No_Mapping[1], Lane_No_Mapping[2]);
    //    MMITSS_LOG(
    //        "The No. of vehicle per lane is: %d %d %d", No_veh_per_lane[0], No_veh_per_lane[1],
    //        No_veh_per_lane[2]);

    // reduce No_lanes if no vehicle in one lane
    No_lanes = index + 1;

    //    MMITSS_LOG("Reduced lane to: %d", No_lanes);

    // Record how many vehicles are inserted each lane in the slow down region
    int* inserted_each_lane;
    inserted_each_lane = new int[10]; // at most 8 lanes, no need to map the lane id!!

    // Define maximum number of vehicles can be inserted each lane
    int max_insertion;
    if (penetration >= 0.75f)
        max_insertion = 1;
    if (penetration < 0.75f && penetration >= 0.5f)
        max_insertion = 2;
    if (penetration < 0.5f)
        max_insertion = 3;

    int max_q_prop_time = (int)(1.0f / penetration);

    // Step 1: calculate the queuing part for each lane
    float* est_queue_length;
    est_queue_length = new float[No_lanes];

    // number of queued equipped vehicle each lane
    int* No_queue_veh;
    No_queue_veh = new int[No_lanes];

    // Last queued equipped vehicle position
    float* Last_queue_equipped_veh_pos;
    Last_queue_equipped_veh_pos = new float[No_lanes];

    // number of queued vehicle (both equipped and unequipped) each lane
    int* No_queue_veh_all; // The same
    No_queue_veh_all = new int[No_lanes];

    for (i = 0; i < No_lanes; i++)
    {
        No_queue_veh[i] = 0;
    }

    veh = sorted_vehlist.begin();
    for (j = 0; j < No_lanes; j++)
    {
        long time1 =
            current_time - (int)red_elapse_time; // red start time Note: this index is only for
                                                 // phase 6, need to be changed
                                                 // later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        float queue_length1 = 0;
        double queue_speed = 0;
        float queue_length2 = 0;
        long time2 = 0;
        for (i = 0; i < No_veh_per_lane[j]; i++)
        {
            if (veh->second.Speed < 0.1) // stopped vehicle
            {
                queue_length2 = veh->second.stopBarDistance;
                time2 = veh->second.time_stop;
                // Need to deal with the case of over saturation, some vehicle stops before the red
                // beginning at the end of the queue
                if (time2 <= time1)
                    queue_speed = 0;
                else
                    queue_speed = (queue_length2 - queue_length1) / (time2 - time1);
                time1 = time2;
                queue_length1 = queue_length2;
                No_queue_veh[j]++;
            }
            veh++;
        }
        int q_time_after_last_veh;
        if (max_q_prop_time <= current_time - time2)
            q_time_after_last_veh = max_q_prop_time;
        else
            q_time_after_last_veh = current_time - time2;

        est_queue_length[j] = queue_length2 + (float)queue_speed * q_time_after_last_veh;
        Last_queue_equipped_veh_pos[j] = queue_length2;
    }

    //    MMITSS_LOG("Current time is: %ld", current_time);
    //    MMITSS_LOG(
    //        "Estimated queue length of each lane is: %f %f %f\n", est_queue_length[0],
    //        est_queue_length[1], est_queue_length[2]);

    // For queuing part, all we need is the No. of vehicles because all the ETAs of queued vehicle
    // are 0
    /*  Calculate the No. Vehicles in queue based on queue length and add vehicle to sorted_vehlist
     */

    int flag = 0; // This flag is used to identify the position of the inserted vehicle
    for (i = 0; i < No_lanes; i++)
    {
        if (i == 0)
            flag = No_queue_veh[i];
        else // i>0
        {
            flag = 0;
            for (j = 0; j < i; j++)
                flag += No_veh_per_lane[j];
            flag += No_queue_veh[i];
        }
        int No_total_veh_queue = ceil(est_queue_length[i] / jam_length) +
                                 1; //+1 is because of the vehicle at the stop line
        No_queue_veh_all[i] = No_total_veh_queue;
        // Note here the sum of No_queue_veh_all is not equal to the size of sorted_vehlist, because
        // only add vehicle at the last of the queue, not for other gaps!!!!!!!!!!!!!!!!!!!!!!!
        if (est_queue_length[i] - Last_queue_equipped_veh_pos[i] >
            jam_length / 2) // Insert a vehicle at the last of the queue for calculation in the slow
                            // down region
        {
            temp_veh.Speed = 0;
            temp_veh.stopBarDistance = Last_queue_equipped_veh_pos[i] + jam_length;
            temp_veh.lane = Lane_No_Mapping[i];
            temp_veh.acceleration = 0;
            sorted_vehlist[std::make_pair(temp_veh.lane, temp_veh.stopBarDistance)] = temp_veh;
            No_veh_per_lane[i]++;
        }
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Use this No_queue_veh_all value for the No of vehicle for the queuing part for the arrival
    // table
    //    MMITSS_LOG(
    //        "The number of queued vehicle of each lane are (both equipped and unequipped): %d %d
    //        %d", No_queue_veh_all[0], No_queue_veh_all[1], No_queue_veh_all[2]);

    // Step2: Calculate the slow-down part of each lane
    // Define the slow down region
    float slow_region = 50; // 50m for the slow down region temporary, need to change
                            // later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    float ave_queue_length; // average queue length of each lane, since in vissim the queue length
                            // are almost the same for all lanes
    float total_queue_length = 0;
    for (i = 0; i < No_lanes; i++)
    {
        total_queue_length += est_queue_length[i];
    }
    ave_queue_length = total_queue_length / No_lanes;

    if (!sorted_vehlist.empty())
    {
        veh = sorted_vehlist.begin();
        // Assign moving dynamics to leading vehicle
        // We can't insert vehicle for the first vehicle because lack of information
        float speed_pre = veh->second.Speed;
        float acc_pre = veh->second.acceleration;
        float pos_pre = veh->second.stopBarDistance;
        float lane_pre = veh->second.lane;
        veh++;

        while (veh != sorted_vehlist.end())
        {
            if (veh->second.Speed != 0 && veh->second.lane == lane_pre &&
                (float)veh->second.stopBarDistance > ave_queue_length + 3 &&
                (float)veh->second.stopBarDistance <
                    ave_queue_length + slow_region) // moving vehicle in the same lane
            {
                // Determine car following state
                int veh_state = determine_state(veh->second.stopBarDistance, veh->second.Speed,
                    veh->second.acceleration, pos_pre, speed_pre, acc_pre);
                // calculate max acc and min acc
                float min_spd = std::min((float)veh->second.Speed, speed_pre);
                float max_acc = 3.5f - 3.5f / 40 * min_spd; // m/s
                float min_acc = -20 + 1.5f / 60 * min_spd;

                float delta_x = (float)veh->second.stopBarDistance - pos_pre; // headway
                float AX = jam_length;                                        // min headway
                // float CX=40;
                // float SDV=pow((delta_x-AX)/CX,2);
                // float OPDV=-2.25*SDV;
                float BX = 2.5 * sqrt(min_spd);
                float ABX = AX + 2.5f * (float)sqrt(min_spd);
                float SDX = AX + 3.75f * (float)sqrt(min_spd);
                float delta_v = (float)veh->second.Speed - speed_pre;

                // calculate the desired acceleration based on vehicle state
                float desired_acc;
                if (veh_state == 1) // free flow state
                    desired_acc = std::min(max_acc, desired_spd - (float)veh->second.Speed);
                if (veh_state == 2) // Emergency state
                    desired_acc = 0.5f * (delta_v * delta_v / (AX - delta_x)) + acc_pre +
                                  min_acc * (ABX - delta_x) / BX;
                if (veh_state == 3) // closing state
                {
                    desired_acc =
                        std::max(0.5f * delta_v * delta_v / (ABX - delta_x) + acc_pre, min_acc);
                }
                if (veh_state == 4) // following state
                    desired_acc = 0;

                // Determin whether to trigger the insertion
                int trigger = 0; // 0: not trigger  1: triggered insertion
                // Trigger one: in free flow region, if acc is two small
                if (veh_state == 1 && desired_acc - (float)veh->second.acceleration > acc_diff_thre)
                    trigger = 1;
                // Trigger two: in closing or following region, and following distance is too long
                if ((veh_state == 3 || veh_state == 4) &&
                    delta_x > 2 * SDX) // here 2 is a parameter for calibration!!!!!!!!!!!!!  modify
                                       // the original model!!!!!!!!!!!!!!!!!!
                    trigger = 1;

                // If triggered insertion
                if (trigger == 1)
                {
                    // Calculate pos, speed and acc of inserted vehicle
                    // speed
                    float tmp_spd = std::max(
                        (float)veh->second.Speed + lamda * (float)veh->second.acceleration, 0.0f);
                    temp_veh.Speed = tmp_spd;

                    // acc
                    if (acc_pre <= 0 &&
                        veh->second.acceleration <=
                            0) // if both of the leading and following vehicle are decelerating
                        temp_veh.acceleration = cal_dec(tmp_spd);
                    else
                        temp_veh.acceleration = 0;

                    // position
                    float tmp_delta_x;
                    if (tmp_spd <
                        (float)veh->second.Speed) // the inserted leading vehicle is slower, the
                                                  // current vehicle is in closing region
                    {
                        if (fabs(veh->second.acceleration - temp_veh.acceleration) >
                            0.1) // if the deceleration rate is different
                            tmp_delta_x =
                                7.25 + 3.75 * sqrt(tmp_spd) -
                                0.5 *
                                    pow(std::min((double)-lamda * veh->second.acceleration,
                                            veh->second.Speed),
                                        2) /
                                    (veh->second.acceleration - temp_veh.acceleration);
                        else
                            tmp_delta_x = 7.25 + 3.75 * sqrt(tmp_spd);
                    }
                    else
                        tmp_delta_x = 7.25 + 3.75 * sqrt(tmp_spd);
                    temp_veh.stopBarDistance = veh->second.stopBarDistance - (double)tmp_delta_x;

                    // lane    same lane
                    temp_veh.lane = veh->second.lane;

                    // check the availablity of the inserted vehicle
                    if ((float)temp_veh.stopBarDistance > pos_pre + jam_length &&
                        inserted_each_lane[temp_veh.lane] < max_insertion)
                    // if (temp_veh.stopBarDistance>pos_pre+jam_length)
                    {
                        // increase the inserted vehicle number
                        inserted_each_lane[temp_veh.lane]++;

                        // Add the inserted vehicle to the list
                        sorted_vehlist[std::make_pair(temp_veh.lane, temp_veh.stopBarDistance)] =
                            temp_veh;
                        //                        sorted_vehlist.InsertAt(temp_veh);
                        //                        cout<<"Current Vehicle
                        //                        "<<sorted_vehlist.Data().lane<<"
                        //                        "<<sorted_vehlist.Data().stopBarDistance<<"
                        //                        "<<sorted_vehlist.Data().Speed<<"
                        //                        "<<sorted_vehlist.Data().acceleration<<"
                        //                        "<<sorted_vehlist.Data().time_stop<<endl;
                        //                        int Pos_in_list=sorted_vehlist.CurrentPosition();
                        //                        sorted_vehlist.Reset(Pos_in_list-1);
                    }
                }
            }

            speed_pre = veh->second.Speed;
            acc_pre = veh->second.acceleration;
            pos_pre = veh->second.stopBarDistance;
            lane_pre = veh->second.lane;
            veh++;
        }
    }

    // Step 3: Insert Vehicles in the free flow region
    // Step 3.1 Count No. of veh in free flow region
    int eqp_veh_in_free_region = 0;
    int* eqp_veh_in_fr_lane;
    eqp_veh_in_fr_lane = new int[No_lanes];
    for (i = 0; i < No_lanes; i++)
    {
        eqp_veh_in_fr_lane[i] = 0;
    }

    float total_speed = 0;
    for (const auto& veh_p : sorted_vehlist)
    {
        if ((float)veh_p.second.stopBarDistance >
            ave_queue_length + slow_region) // when vehicles are in free flow region
        {
            eqp_veh_in_free_region++;
            total_speed += (float)veh_p.second.Speed;
            for (i = 0; i < No_lanes; i++)
            {
                if (veh_p.second.lane == Lane_No_Mapping[i])
                    eqp_veh_in_fr_lane[i]++;
            }
        }
    }

    int total_veh_in_free_region = eqp_veh_in_free_region / penetration;
    int veh_ffr_each_lane = total_veh_in_free_region / No_lanes;
    float ave_ffs = total_speed / eqp_veh_in_free_region;

    // Add the identified non connected vehicles into the trackedvehicle
    // trackedveh.Reset(); --> MV: This is different to the prio version of that function TBD: check
    // why
    for (i = 0; i < No_lanes; i++)
    {
        int inserted_No = veh_ffr_each_lane - eqp_veh_in_fr_lane[i];
        for (j = 0; j < inserted_No; j++)
        {
            float distance = (dsrc_range - ave_queue_length - slow_region) / inserted_No;
            // here should come to the end of the list
            temp_veh.lane = Lane_No_Mapping[i];
            temp_veh.stopBarDistance = ave_queue_length + slow_region + distance * j;
            temp_veh.acceleration = 0;
            temp_veh.Speed = ave_ffs;
            // trackedveh.InsertRear(temp_veh); --> MV: This is different to the prio version of
            // that function TBD: check why
            trackedveh.emplace_back(temp_veh);
        }
    }

    // print the final vehicle list
    //    MMITSS_LOG("Total No. of Vehicle in the list is %d:", sorted_vehlist.size());

    int total_veh_queue_all_lane = 0;

    for (i = 0; i < No_lanes; i++)
    {
        total_veh_queue_all_lane += No_queue_veh_all[i];
    }
    arrivalTable[0][phase_est - 1] = total_veh_queue_all_lane; // Total number of queued vehicle

    for (const auto& veh_p : sorted_vehlist)
    {
        if (veh_p.second.Speed > 2) // stopped vehicle
        {
            int ETA = ceil(veh_p.second.stopBarDistance / veh_p.second.Speed);
            if (ETA <= 50)
                arrivalTable[ETA][phase_est - 1]++; // increase the value in ETA's row
        }
    }

    delete (No_veh_per_lane);
    delete (Lane_No_Mapping);
    delete (inserted_each_lane);
    delete (est_queue_length);
    delete (No_queue_veh);
    delete (No_queue_veh_all);
    delete (eqp_veh_in_fr_lane);
}

void sort_list(ConnectedVehicleList& vehListEachPhase)
{
    for (const auto& veh : vehListEachPhase)
    {
        sorted_vehlist[std::make_pair(veh.lane, veh.stopBarDistance)] = veh;
    }
}

int determine_state(
    float cur_pos, float cur_spd, float cur_acc, float lead_pos, float lead_spd, float lead_acc)
{
    NOTUSED(cur_acc);
    NOTUSED(lead_acc);

    int state = 0;
    float delta_x = cur_pos - lead_pos; // headway
    float AX = jam_length;              // min headway
    float CX = 40;
    float SDV = pow((delta_x - AX) / CX, 2);
    float OPDV = -2.25f * SDV;
    float ABX_v = std::min(cur_spd, lead_spd); // this speed is used in the calculation of ABX
    float ABX = AX + 2.5f * (float)sqrt(ABX_v);
    float SDX = AX + 3.75f * (float)sqrt(ABX_v);
    float delta_v = cur_spd - lead_spd;

    if (delta_v < OPDV)
        state = 1;
    else
    {
        if (delta_x < ABX)
            state = 2;
        else
        {
            if (delta_v > SDV)
                state = 3;
            else
            {
                if (delta_x > SDX)
                    state = 1;
                else
                    state = 4;
            }
        }
    }
    return state;
}

float cal_dec(float speed) // Calculate the deceleration rate based on current spd
{
    float dec = 0;
    if (speed < 2.78f)
        dec = -0.91;
    if (speed >= 2.78f && speed < 5.56f)
        dec = -1.92;
    if (speed >= 5.56f && speed < 8.33f)
        dec = -1.82;
    if (speed >= 8.33f && speed < 11.11f)
        dec = -1.26;
    if (speed >= 11.11f && speed < 13.89f)
        dec = -0.67;
    return dec;
}

} // namespace Mmitss
} // namespace WaveApp
