//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once

#include <string>
#include <vector>

#include "PhaseState.h"

class RsuConfig
{
  public:
    typedef struct
    {
        int MissPhase[2], MP_Relate[2], MP_Ring[2]; // It is possible that at most one phase missing
        int Ring1No, Ring2No;
        std::vector<int> Phase_Seq_R1;
        std::vector<int> Phase_Seq_R2; // Phase_Seq_R[1][2] {0-3}
        double Yellow[8], Red[8], Gmin[8], Gmax[8];
        double dCoordinationWeight;
        int iCoordinatedPhase[2];
        int iCoordinationCycle;
        double dCoordinationSplit[2];
        int iCoordinationOffset;
        double iTransitWeight;
        double iTruckWeight;

        void printConfgig();

    } RSU_Config;

    RsuConfig(const std::string& configInfo, bool newCfg = false);
    RsuConfig(const std::string& configInfo, const std::string& prioConfig);
    RsuConfig(){};

    const RSU_Config& getConfig() const { return m_cfg; }

    const PhaseState& getCombinedPhases() const { return m_combinedPhases; }

    void RSUConfig2ConfigFile(const std::string& filename, int* PhaseInfo, int Num);
    void PrintRSUConfig() const;
    void PrintRSUConfig2File(const std::string& filename);

    bool ReadInConfig(const std::string& filename);
    bool ReadInConfig(const std::string& configInfo, int New);
    bool ReadInConfig(const std::string& configInfo, const std::string& prioConfig);

  private:
    RSU_Config m_cfg;
    PhaseState m_combinedPhases;
};
