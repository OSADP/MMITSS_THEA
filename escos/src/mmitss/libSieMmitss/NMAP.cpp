#include "NMAP.h"
#include <iostream>
#include <string.h>
#include <fstream>
#include "stdlib.h"
#include "math.h"
#include "SieMmitss.hpp"
#include "facilityLayer.hpp"

#define DEFAULT_ARRAY_SIZE 1024

using namespace std;

int MAP::getLaneNum(int laneID) const
{
    for (const auto& appr : intersection.Approaches)
    {
        for (size_t i = 0; i < appr.Lanes.size(); i++)
        {
            if (appr.Lanes[i].ID == laneID)
                return i + 1;
        }
    }
    return -1;
}

int MAP::getLaneID(int approach, int lane) const
{
    if (approach >= (int)intersection.Approaches.size())
        return -1;
    if (lane >= (int)intersection.Approaches[approach].Lanes.size())
        return -1;
    return intersection.Approaches[approach].Lanes[lane].ID;
}

int MAP::getApproach(int laneID) const
{
    if (laneID <= 0)
        return -1;
    for (unsigned int i = 0; i < intersection.Approaches.size(); i++)
    {
        for (const auto& l : intersection.Approaches[i].Lanes)
        {
            if (l.ID == laneID)
                return intersection.Approaches[i].ID;
        }
    }
    return -1;
}

// This function parse information from *.nmap and save to the MAP data structure
bool MAP::ParseIntersection(const char* filename)
{
    // Start parsing the MAP data from file

    // Note: when the approaches number of lane number exceeds 10, there will be problem with
    // token_char[0]-48,
    // because now the number takes 2 positions
    ifstream map_file;
    map_file.open(filename);

    if (!map_file)
    {
        ITSAPP_WRN("Error: Open file %s", filename);
        return false;
    }

    string lineread;
    char token_char[DEFAULT_ARRAY_SIZE];
    char temp_char[DEFAULT_ARRAY_SIZE];
    string token;
    // Read first 6 lines of Intersection information
    int j, k, l;
    for (int i = 0; i < 6; i++)
    {
        getline(map_file, lineread); // Read line by line
        if ((lineread.size() != 0) && (lineread.size() < DEFAULT_ARRAY_SIZE))
        {
            sscanf(lineread.c_str(), "%s", token_char);
            // token.assign(token_char);
            if (strcmp(token_char, "MAP_Name") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                intersection.Map_Name.assign(temp_char);
            }
            else if (strcmp(token_char, "RSU_ID") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                intersection.Rsu_ID.assign(temp_char);
            }
            else if (strcmp(token_char, "IntersectionID") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                intersection.ID = atoi(temp_char);
                ITSAPP_LOG("IntersectionID: %d", intersection.ID);
            }
            else if (strcmp(token_char, "Intersection_attributes") == 0)
            {
                sscanf(lineread.c_str(), "%*s %s", temp_char);
                for (j = 0; j < 8; j++)
                {
                    intersection.Attributes[7 - j] =
                        temp_char[j] -
                        48; // directly use ASCII codes,0 is 48,1 is 49, comply with bit position
                }
            }
            else if (strcmp(token_char, "Reference_point") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf", &intersection.Ref_Lat,
                    &intersection.Ref_Long, &intersection.Ref_Ele);
                MMITSS_LOG("RefPoint: %lf, %lf, %lf", intersection.Ref_Lat, intersection.Ref_Long,
                    intersection.Ref_Ele);
            }
            else if (strcmp(token_char, "No_Approach") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d", &intersection.Appro_No);
            }
        }
        else
        {
            ITSAPP_WRN("ERROR reading NMAP on line %d", i + 1);
            return false;
        }
    }
    // end of reading intersection information

    // initialize approaches
    const int MAX_APPROACHS = 8;
    Approach temp_appro;
    for (int i = 0; i < MAX_APPROACHS; i++)
    {
        intersection.Approaches.push_back(temp_appro);
    }

    // Start to parse the approaches, lanes and lane nodes
    int processedApproaches = 0;
    int i = 0;
    while (processedApproaches < intersection.Appro_No)
    {
        getline(map_file, lineread); // Read line by line
        sscanf(lineread.c_str(), "%s", token_char);
        while (strcmp(token_char, "end_approach")) // if not end_approach
        {
            // The next 3 lines: Read Approach attributes
            for (j = 0; j < 3; j++)
            {
                if (strcmp(token_char, "Approach") == 0)
                {
                    sscanf(lineread.c_str(), "%*s %s", temp_char);
                    i = atoi(temp_char) - 1;
                    intersection.Approaches[i].ID = i + 1;
                    ITSAPP_TRACE("process Approach: %d", i + 1);
                    processedApproaches++;
                    getline(map_file, lineread); // Read line by line
                    sscanf(lineread.c_str(), "%s", token_char);
                }
                else if (strcmp(token_char, "Approach_type") == 0)
                {
                    sscanf(lineread.c_str(), "%*s %s", temp_char);
                    intersection.Approaches[i].type = atoi(temp_char);
                    getline(map_file, lineread); // Read line by line
                    sscanf(lineread.c_str(), "%s", token_char);
                }
                else if (strcmp(token_char, "No_lane") == 0)
                {
                    sscanf(lineread.c_str(), "%*s %s", temp_char);
                    intersection.Approaches[i].Lane_No = atoi(temp_char);
                    // here don't need to read another line
                }
            }
            // End reading approaches attributes

            //////////////////////////////Start reading Lanes
            // first initialize lanes
            Lane temp_lane;
            for (j = 0; j < intersection.Approaches[i].Lane_No; j++)
            {
                intersection.Approaches[i].Lanes.push_back(temp_lane);
            }
            // Read lane information
            for (j = 0; j < intersection.Approaches[i].Lane_No; j++) // j is the loop for lane
            {
                getline(map_file, lineread); // Read line by line
                sscanf(lineread.c_str(), "%s", token_char);
                while (strcmp(token_char, "end_lane")) // if not end_approach
                {
                    // first 6 lines: read lane attributes
                    for (k = 0; k < 6; k++)
                    {
                        if (strcmp(token_char, "Lane") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            intersection.Approaches[i].Lanes[j].Lane_Name = temp_char;
                        }
                        else if (strcmp(token_char, "Lane_ID") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            intersection.Approaches[i].Lanes[j].ID = atoi(temp_char);
                            ITSAPP_TRACE("Approach: %d has lane: %d", i + 1,
                                intersection.Approaches[i].Lanes[j].ID);
                        }
                        else if (strcmp(token_char, "Lane_type") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            intersection.Approaches[i].Lanes[j].Type = atoi(temp_char);
                        }
                        else if (strcmp(token_char, "Lane_attributes") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            for (l = 0; l < 16; l++)
                            {
                                intersection.Approaches[i].Lanes[j].Attributes[15 - l] =
                                    temp_char[l] - 48; // directly use ASCII codes,0 is 48,1 is 49,
                                                       // comply with bit position
                            }
                        }
                        else if (strcmp(token_char, "Lane_width") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            intersection.Approaches[i].Lanes[j].Width = atoi(temp_char);
                        }
                        else if (strcmp(token_char, "No_nodes") == 0)
                        {
                            sscanf(lineread.c_str(), "%*s %s", temp_char);
                            intersection.Approaches[i].Lanes[j].Node_No = atoi(temp_char);
                        }

                        if (k < 5) // the last line doesn't need to read a new line again
                        {
                            getline(map_file, lineread); // Read line by line
                            sscanf(lineread.c_str(), "%s", token_char);
                        }
                    }
                    // Initialize Nodes First
                    LaneNodes temp_node;
                    for (k = 0; k < intersection.Approaches[i].Lanes[j].Node_No; k++)
                    {
                        intersection.Approaches[i].Lanes[j].Nodes.push_back(temp_node);
                    }
                    // Start to read the nodes
                    for (k = 0; k < intersection.Approaches[i].Lanes[j].Node_No;
                         k++) // get the node number
                    {
                        getline(map_file, lineread); // Read line by line
                        sscanf(lineread.c_str(), "%s", token_char);
                        // get element number from token char
                        intersection.Approaches[i].Lanes[j].Nodes[k].index.Approach =
                            token_char[0] - 48; // Token_char is 1.1.1
                        intersection.Approaches[i].Lanes[j].Nodes[k].index.Lane =
                            token_char[2] - 48;
                        intersection.Approaches[i].Lanes[j].Nodes[k].index.Node = k + 1;
                        // get lat, long data
                        sscanf(lineread.c_str(), "%*s %lf %lf",
                            &intersection.Approaches[i].Lanes[j].Nodes[k].Latitude,
                            &intersection.Approaches[i].Lanes[j].Nodes[k].Longitude);
                    }
                    // read connection lanes
                    getline(map_file, lineread); // Read line by line
                    sscanf(lineread.c_str(), "%s %d", token_char,
                        &intersection.Approaches[i].Lanes[j].Connection_No);
                    // Initialize lane connections
                    LaneConnection temp_connection;
                    for (k = 0; k < intersection.Approaches[i].Lanes[j].Connection_No; k++)
                    {
                        intersection.Approaches[i].Lanes[j].Connections.push_back(temp_connection);
                    }
                    // start to read connections
                    for (k = 0; k < intersection.Approaches[i].Lanes[j].Connection_No; k++)
                    {
                        getline(map_file, lineread); // Read line by line
                        sscanf(lineread.c_str(), "%s %d", token_char,
                            &intersection.Approaches[i].Lanes[j].Connections[k].Maneuver);
                        // get approach and lane information from token_char
                        intersection.Approaches[i]
                            .Lanes[j]
                            .Connections[k]
                            .ConnectedLaneName.Approach = token_char[0] - 48; // Token_char is 1.1
                        intersection.Approaches[i].Lanes[j].Connections[k].ConnectedLaneName.Lane =
                            token_char[2] - 48;
                    }

                    getline(map_file, lineread); // Read line by line
                    sscanf(lineread.c_str(), "%s", token_char);
                }
            }
            getline(map_file, lineread); // Read line by line
            sscanf(lineread.c_str(), "%s", token_char);
        }
    }
    // The end line
    getline(map_file, lineread); // Read line by line
    sscanf(lineread.c_str(), "%s", token_char);
    if (strcmp(token_char, "end_map") == 0)
    {
        ITSAPP_TRACE("Parse map file successfully!");
    }
    return true;
}
