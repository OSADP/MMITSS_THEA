//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

// #include "stdafx.h"
#include "Array.h"
#include "Config.h"

#include "facilityLayer.hpp"

RsuConfig::RsuConfig(const std::string& configInfo, bool newCfg)
{
    if (!newCfg)
        ReadInConfig(configInfo);
    else
        ReadInConfig(configInfo, 1);
}

RsuConfig::RsuConfig(const std::string& configInfo, const std::string& prioConfig)
{
    ReadInConfig(configInfo, prioConfig);
}

void RsuConfig::PrintRSUConfig() const
{
    std::stringstream s;
    for (int i = 0; i < 2; ++i)
    {
        s << "In Ring " << (i + 1) << " The MissPhase is: " << m_cfg.MissPhase[i] + 1
          << "  MP_Relate : " << m_cfg.MP_Relate[i] + 1 << "  Ring " << m_cfg.MP_Ring[i]
          << std::endl;
        ITSAPP_LOG("%s", s.str().c_str());
    }

    ITSAPP_LOG("Ring1 number are: %d,   Details: ", m_cfg.Ring1No);
    PrintArray(m_cfg.Phase_Seq_R1, m_cfg.Ring1No);
    ITSAPP_LOG("Ring2 number are: %d,   Details: ", m_cfg.Ring2No);
    PrintArray(m_cfg.Phase_Seq_R2, m_cfg.Ring2No);

    ITSAPP_LOG("Yellow:  ");
    PrintArray(m_cfg.Yellow, 8);
    ITSAPP_LOG("Red:     ");
    PrintArray(m_cfg.Red, 8);
    ITSAPP_LOG("Gmin:    ");
    PrintArray(m_cfg.Gmin, 8);
    ITSAPP_LOG("Gmax:    ");
    PrintArray(m_cfg.Gmax, 8);

    ITSAPP_LOG("Coordination Weight: %lf", m_cfg.dCoordinationWeight);
    ITSAPP_LOG("Coordinate Phase: ");
    PrintArray(m_cfg.iCoordinatedPhase, 2);
    ITSAPP_LOG("Coordination Cycle: %d", m_cfg.iCoordinationCycle);
    ITSAPP_LOG("Coordination Split phase 1: %lf", m_cfg.dCoordinationSplit[0]);
    ITSAPP_LOG("Coordination Split phase 2: %lf", m_cfg.dCoordinationSplit[1]);
    ITSAPP_LOG("Coordination Offset: %d", m_cfg.iCoordinationOffset);
    ITSAPP_LOG("Transit Weight: %lf", m_cfg.iTransitWeight);
    ITSAPP_LOG("Truck Weight: %lf", m_cfg.iTruckWeight);
}

void RsuConfig::PrintRSUConfig2File(const std::string& filename)
{
    fstream fs_Fileout;
    fs_Fileout.open(filename.c_str(), ios::out);

    int i = 0;

    for (i = 0; i < 2; ++i)
    {
        fs_Fileout << "In Ring " << (i + 1) << " The MissPhase is: " << m_cfg.MissPhase[i] + 1
                   << "  MP_Relate : " << m_cfg.MP_Relate[i] + 1 << "  Ring " << m_cfg.MP_Ring[i]
                   << std::endl;
    }

    fs_Fileout << "Ring 1 number are: " << m_cfg.Ring1No << "  Details: ";

    // PrintArray<int>(configIS.Phase_Seq_R1,configIS.Ring1No);
    for (i = 0; i < m_cfg.Ring1No; ++i)
    {
        fs_Fileout << m_cfg.Phase_Seq_R1[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Ring 2 number are: " << m_cfg.Ring2No << "  Details: ";

    // PrintArray<int>(configIS.Phase_Seq_R2,configIS.Ring2No);
    for (i = 0; i < m_cfg.Ring2No; ++i)
    {
        fs_Fileout << m_cfg.Phase_Seq_R2[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Yellow:  ";
    // PrintArray<double>(configIS.Yellow,8);
    for (i = 0; i < 8; ++i)
    {
        fs_Fileout << m_cfg.Yellow[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Red:     ";
    // PrintArray<double>(configIS.Red,8);
    for (i = 0; i < 8; ++i)
    {
        fs_Fileout << m_cfg.Red[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Gmin:    ";
    // PrintArray<double>(configIS.Gmin,8);
    for (i = 0; i < 8; ++i)
    {
        fs_Fileout << m_cfg.Gmin[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Gmax:    ";
    // PrintArray<double>(configIS.Gmax,8);
    for (i = 0; i < 8; ++i)
    {
        fs_Fileout << m_cfg.Gmax[i] << "\t";
    }
    fs_Fileout << std::endl;

    fs_Fileout.close();
}

void RsuConfig::RSUConfig2ConfigFile(const std::string& filename, int* PhaseInfo, int Num)
{
    // PhaseInfo has the information from the requests.txt, including the requested phases.
    // The Num is the total number of the requested phases, No Repeated case (DOES NOT MATTER).
    // *Important* We should have the assumption that the phase in PhaseInfo should
    // be in the ConfigIS. Phase_Seq_R1&2
    // First to sort the request phase information **Probably not necessary**

    // bubbleSort(PhaseInfo,Num);
    // the element in PhaseInfo should be the truephase, not truephase-1

    int TotalPhase[8] = {0};
    double Yellow[8], Red[8], Gmin[8], Gmax[8];

    for (int i = 0; i < Num; ++i)
    {
        int tempPhase = PhaseInfo[i] - 1;

        if (FindIndexArray(m_cfg.Phase_Seq_R1, m_cfg.Ring1No, tempPhase) != -1 ||
            FindIndexArray(m_cfg.Phase_Seq_R2, m_cfg.Ring2No, tempPhase % 4) != -1)

            TotalPhase[tempPhase] = 1;
    }

    int TotalNum = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (TotalPhase[i] == 1)
        {
            TotalNum++;
            Yellow[i] = m_cfg.Yellow[i];
            Red[i] = m_cfg.Red[i];
            Gmin[i] = m_cfg.Gmin[i];
            Gmax[i] = m_cfg.Gmax[i];
        }
        else
        {
            Yellow[i] = 0;
            Red[i] = 0;
            Gmin[i] = 0;
            Gmax[i] = 0;
        }
    }

    fstream fs_Fileout;
    fs_Fileout.open(filename.c_str(), ios::out);
    fs_Fileout << "Phase_Num " << TotalNum << std::endl;
    fs_Fileout << "Phase_Seq";

    for (int i = 0; i < 8; ++i)
    {
        if (TotalPhase[i] == 1)
            fs_Fileout << " " << (i + 1);
        else
            fs_Fileout << " " << 0;
    }
    fs_Fileout << std::endl;

    fs_Fileout << "Gmin";
    for (int i = 0; i < 8; ++i)
        fs_Fileout << "\t" << Gmin[i];
    fs_Fileout << std::endl;

    fs_Fileout << "Yellow";
    for (int i = 0; i < 8; ++i)
        fs_Fileout << "\t" << Yellow[i];
    fs_Fileout << std::endl;

    fs_Fileout << "Red";
    for (int i = 0; i < 8; ++i)
        fs_Fileout << "\t" << Red[i];
    fs_Fileout << std::endl;

    fs_Fileout << "Gmax";
    for (int i = 0; i < 8; ++i)
        fs_Fileout << "\t" << Gmax[i];
    fs_Fileout << std::endl;

    fs_Fileout.close();
}

bool RsuConfig::ReadInConfig(const std::string& filename)
{
    // *Important* Read the configuration: Find out the Missing phase and MP_Relate
    fstream FileRead;
    FileRead.open(filename.c_str(), ios::in);
    // FileRead.open("ConfigurationInfo1268.txt",ios::in);

    if (!FileRead)
    {
        ITSAPP_WRN("Unable to open file! %s", filename.c_str());
        return false;
    }

    for (int i = 0; i < 2; ++i)
    {
        m_cfg.MissPhase[i] = -1;
        m_cfg.MP_Relate[i] = -1;
        m_cfg.MP_Ring[i] = -1;
    }

    int PhaseNo;
    char TempStr[64];
    string lineread;
    vector<int> P11, P12, P21, P22;

    //----------------- Read in the parameters---------------
    while (!FileRead.eof())
    {
        getline(FileRead, lineread);

        if ((lineread.size() != 0) && (lineread.size() < 64))
        {
            sscanf(lineread.c_str(), "%s", TempStr);
            if (strcmp(TempStr, "Phase_Num") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d ", &PhaseNo);
            }
            else if (strcmp(TempStr, "Phase_Seq") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d %d %d %d %d %d %d %d", &m_combinedPhases[0],
                    &m_combinedPhases[1], &m_combinedPhases[2], &m_combinedPhases[3],
                    &m_combinedPhases[4], &m_combinedPhases[5], &m_combinedPhases[6],
                    &m_combinedPhases[7]);
            }
            else if (strcmp(TempStr, "Yellow") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &m_cfg.Yellow[0],
                    &m_cfg.Yellow[1], &m_cfg.Yellow[2], &m_cfg.Yellow[3], &m_cfg.Yellow[4],
                    &m_cfg.Yellow[5], &m_cfg.Yellow[6], &m_cfg.Yellow[7]);
            }
            else if (strcmp(TempStr, "Red") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &m_cfg.Red[0],
                    &m_cfg.Red[1], &m_cfg.Red[2], &m_cfg.Red[3], &m_cfg.Red[4], &m_cfg.Red[5],
                    &m_cfg.Red[6], &m_cfg.Red[7]);
            }
            else if (strcmp(TempStr, "Gmin") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &m_cfg.Gmin[0],
                    &m_cfg.Gmin[1], &m_cfg.Gmin[2], &m_cfg.Gmin[3], &m_cfg.Gmin[4], &m_cfg.Gmin[5],
                    &m_cfg.Gmin[6], &m_cfg.Gmin[7]);
            }
            else if (strcmp(TempStr, "Gmax") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &m_cfg.Gmax[0],
                    &m_cfg.Gmax[1], &m_cfg.Gmax[2], &m_cfg.Gmax[3], &m_cfg.Gmax[4], &m_cfg.Gmax[5],
                    &m_cfg.Gmax[6], &m_cfg.Gmax[7]);
            }
        }
    }
    FileRead.close();

    //-------------Handle the parameters for non-complete phases case----//
    for (int i = 0; i < 8; ++i)
    {
        if (m_combinedPhases[i] > 0)
        {
            switch (m_combinedPhases[i])
            {
                case 1:
                case 2:
                    P11.push_back(m_combinedPhases[i]);
                    break;
                case 3:
                case 4:
                    P12.push_back(m_combinedPhases[i]);
                    break;
                case 5:
                case 6:
                    P21.push_back(m_combinedPhases[i]);
                    break;
                case 7:
                case 8:
                    P22.push_back(m_combinedPhases[i]);
                    break;
            }
        }
    }

    //--------Here is different from the one within the GeneratePriReqMod--------//
    if (P11.size() == 0)
    {
        P11.push_back(2); // If P11 is missing, put a dummy "6" which means "2" in it
        m_cfg.MissPhase[0] = 1;
        m_cfg.MP_Relate[0] = 5;
        m_cfg.MP_Ring[0] = 0;
    }
    if (P12.size() == 0)
    {
        P12.push_back(4); // If P12 is missing, put a dummy "8" which means "4" in it
        m_cfg.MissPhase[0] = 3;
        m_cfg.MP_Relate[0] = 7;
        m_cfg.MP_Ring[0] = 0;
    }
    if (P21.size() == 0)
    {
        P21.push_back(6); // If P21 is missing, put a dummy "2" which means "6" in it
        m_cfg.MissPhase[1] = 5;
        m_cfg.MP_Relate[1] = 1;
        m_cfg.MP_Ring[1] = 1;
    }
    if (P22.size() == 0)
    {
        P22.push_back(8); // If P21 is missing, put a dummy "4" which means "8" in it
        m_cfg.MissPhase[1] = 7;
        m_cfg.MP_Relate[1] = 3;
        m_cfg.MP_Ring[1] = 1;
    }

    for (vector<int>::iterator it = P12.begin(); it < P12.end(); it++)
    {
        P11.push_back(*it);
    }
    for (vector<int>::iterator it = P22.begin(); it < P22.end(); it++)
    {
        P21.push_back(*it);
    }

    auto it = P11.begin();
    int i = 0;
    int R1No = P11.size();
    int R2No = P21.size();

    // int SP1=StartPhase[0]-1,SP2=StartPhase[1]-1;  // Real phase -1
    std::vector<int> Phase_Seq_R1(R1No, 0); // Real phase-1
    for (it = P11.begin(), i = 0; it < P11.end() && i < R1No; it++, ++i)
    {
        Phase_Seq_R1[i] = *it - 1;
    }

    std::vector<int> Phase_Seq_R2(R2No, 0);
    for (it = P21.begin(), i = 0; it < P21.end() && i < R2No; it++, ++i)
    {
        Phase_Seq_R2[i] = *it - 1;
    }

    m_cfg.Ring1No = R1No;
    m_cfg.Ring2No = R2No;

    m_cfg.Phase_Seq_R1 = std::vector<int>(R1No, 0);
    m_cfg.Phase_Seq_R2 = std::vector<int>(R2No, 0);

    // std::cout << "MissPhase= " << m_cfg.MissPhase << "  MP_Relate:= " << m_cfg.MP_Relate
    //      << " Phase Num:=" << PhaseNo << std::endl;

    // std::cout << "Continue to Parsing...";
    // cin.get();

    for (i = 0; i < 2; ++i)
    {
        if (m_cfg.MP_Relate[i] >= 0)
        {
            // std::cout << "HAVE Missed phase:" << Config.MissPhase << std::endl;
            m_cfg.Yellow[m_cfg.MissPhase[i]] = m_cfg.Yellow[m_cfg.MP_Relate[i]];
            m_cfg.Red[m_cfg.MissPhase[i]] = m_cfg.Red[m_cfg.MP_Relate[i]];
            m_cfg.Gmin[m_cfg.MissPhase[i]] = m_cfg.Gmin[m_cfg.MP_Relate[i]];
            m_cfg.Gmax[m_cfg.MissPhase[i]] = m_cfg.Gmax[m_cfg.MP_Relate[i]];
        }
    }

    for (i = 0; i < R1No; ++i)
    {
        m_cfg.Phase_Seq_R1[i] = Phase_Seq_R1[i];
    }
    for (i = 0; i < R2No; ++i)
    {
        m_cfg.Phase_Seq_R2[i] = Phase_Seq_R2[i] % 4;
    }
    return true;
}

// Read into a RSU_Config RsuConfig_EV from the Temp configuration file.
// The difference is that the phase sequence will not have the missing phase, only have the
// existing phases, which will be used by reading new schedule.
bool RsuConfig::ReadInConfig(const std::string& filename, int New)
{
    RSU_Config& configIS = m_cfg;

    // *Important* Read the configuration: Find out the Missing phase and MP_Relate
    fstream FileRead;
    FileRead.open(filename.c_str(), ios::in);
    // FileRead.open("ConfigurationInfo1268.txt",ios::in);

    if (!FileRead)
    {
        ITSAPP_WRN("Unable to open file! %s", filename.c_str());
        return false;
    }

    for (int i = 0; i < 2; ++i)
    {
        configIS.MissPhase[i] = -1;
        configIS.MP_Relate[i] = -1;
        configIS.MP_Ring[i] = -1;
    }

    int PhaseNo;
    int PhaseSeq[8];
    char TempStr[64];
    string lineread;
    vector<int> P11, P12, P21, P22;

    //----------------- Read in the parameters---------------
    while (!FileRead.eof())
    {
        getline(FileRead, lineread);

        if ((lineread.size() != 0) && (lineread.size() < 64))
        {
            sscanf(lineread.c_str(), "%s", TempStr);
            if (strcmp(TempStr, "Phase_Num") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d ", &PhaseNo);
            }
            else if (strcmp(TempStr, "Phase_Seq") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d %d %d %d %d %d %d %d", &PhaseSeq[0], &PhaseSeq[1],
                    &PhaseSeq[2], &PhaseSeq[3], &PhaseSeq[4], &PhaseSeq[5], &PhaseSeq[6],
                    &PhaseSeq[7]);
            }
            else if (strcmp(TempStr, "Yellow") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Yellow[0],
                    &configIS.Yellow[1], &configIS.Yellow[2], &configIS.Yellow[3],
                    &configIS.Yellow[4], &configIS.Yellow[5], &configIS.Yellow[6],
                    &configIS.Yellow[7]);
            }
            else if (strcmp(TempStr, "Red") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Red[0],
                    &configIS.Red[1], &configIS.Red[2], &configIS.Red[3], &configIS.Red[4],
                    &configIS.Red[5], &configIS.Red[6], &configIS.Red[7]);
            }
            else if (strcmp(TempStr, "Gmin") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Gmin[0],
                    &configIS.Gmin[1], &configIS.Gmin[2], &configIS.Gmin[3], &configIS.Gmin[4],
                    &configIS.Gmin[5], &configIS.Gmin[6], &configIS.Gmin[7]);
            }
            else if (strcmp(TempStr, "Gmax") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Gmax[0],
                    &configIS.Gmax[1], &configIS.Gmax[2], &configIS.Gmax[3], &configIS.Gmax[4],
                    &configIS.Gmax[5], &configIS.Gmax[6], &configIS.Gmax[7]);
            }
        }
    }
    FileRead.close();
    //-------------Handle the parameters for non-complete phases case----//
    for (int i = 0; i < 8; ++i)
    {
        if (PhaseSeq[i] > 0)
        {
            switch (PhaseSeq[i])
            {
                case 1:
                case 2:
                    P11.push_back(PhaseSeq[i]);
                    break;
                case 3:
                case 4:
                    P12.push_back(PhaseSeq[i]);
                    break;
                case 5:
                case 6:
                    P21.push_back(PhaseSeq[i]);
                    break;
                case 7:
                case 8:
                    P22.push_back(PhaseSeq[i]);
                    break;
            }
        }
    }

    //--------Here is different from the one above--------//
    if (P11.size() == 0)
    {
        if (New == 0)
        {
            P11.push_back(2);
        }
        // P11.push_back(2);// If P11 is missing, put a dummy "6" which means "2" in it
        configIS.MissPhase[0] = 1;
        configIS.MP_Relate[0] = 5;
        configIS.MP_Ring[0] = 0;
    }
    if (P12.size() == 0)
    {
        if (New == 0)
        {
            P12.push_back(4);
        }
        // P12.push_back(4);// If P12 is missing, put a dummy "8" which means "4" in it
        configIS.MissPhase[0] = 3;
        configIS.MP_Relate[0] = 7;
        configIS.MP_Ring[0] = 0;
    }
    if (P21.size() == 0)
    {
        if (New == 0)
        {
            P21.push_back(6);
        }
        // P21.push_back(6);// If P21 is missing, put a dummy "2" which means "6" in it
        configIS.MissPhase[1] = 5;
        configIS.MP_Relate[1] = 1;
        configIS.MP_Ring[1] = 1;
    }
    if (P22.size() == 0)
    {
        if (New == 0)
        {
            P22.push_back(8);
        }
        // P22.push_back(8);// If P21 is missing, put a dummy "4" which means "8" in it
        configIS.MissPhase[1] = 7;
        configIS.MP_Relate[1] = 3;
        configIS.MP_Ring[1] = 1;
    }

    for (vector<int>::iterator it = P12.begin(); it < P12.end(); it++)
    {
        P11.push_back(*it);
    }
    for (vector<int>::iterator it = P22.begin(); it < P22.end(); it++)
    {
        P21.push_back(*it);
    }

    int R1No = P11.size();
    int R2No = P21.size();

    // int SP1=StartPhase[0]-1,SP2=StartPhase[1]-1;  // Real phase -1
    std::vector<int> Phase_Seq_R1(R1No, 0); // Real phase-1
    int i = 0;
    for (vector<int>::iterator it = P11.begin(); it < P11.end() && i < R1No; it++, ++i)
    {
        Phase_Seq_R1[i] = *it - 1;
    }

    std::vector<int> Phase_Seq_R2(R2No, 0);
    i = 0;
    for (vector<int>::iterator it = P21.begin(); it < P21.end() && i < R2No; it++, ++i)
    {
        Phase_Seq_R2[i] = *it - 1;
    }

    configIS.Ring1No = R1No;
    configIS.Ring2No = R2No;

    configIS.Phase_Seq_R1 = std::vector<int>(R1No);
    configIS.Phase_Seq_R2 = std::vector<int>(R2No);

    // std::cout << "MissPhase= " << configIS.MissPhase << "  MP_Relate:= " << configIS.MP_Relate
    //      << " Phase Num:=" << PhaseNo << std::endl;

    // std::cout << "Continue to Parsing...";
    // cin.get();

    for (i = 0; i < 2; ++i)
    {
        if (configIS.MP_Relate[i] >= 0)
        {
            // std::cout << "HAVE Missed phase:" << configIS.MissPhase << std::endl;
            configIS.Yellow[configIS.MissPhase[i]] = configIS.Yellow[configIS.MP_Relate[i]];
            configIS.Red[configIS.MissPhase[i]] = configIS.Red[configIS.MP_Relate[i]];
            configIS.Gmin[configIS.MissPhase[i]] = configIS.Gmin[configIS.MP_Relate[i]];
            configIS.Gmax[configIS.MissPhase[i]] = configIS.Gmax[configIS.MP_Relate[i]];
        }
    }

    for (i = 0; i < R1No; ++i)
    {
        configIS.Phase_Seq_R1[i] = Phase_Seq_R1[i];
    }
    for (i = 0; i < R2No; ++i)
    {
        configIS.Phase_Seq_R2[i] = Phase_Seq_R2[i] % 4;
    }
    return true;
}

bool RsuConfig::ReadInConfig(const std::string& configInfo, const std::string& prioConfig)
{
    // *Important* Read the configuration: Find out the Missing phase and MP_Relate
    RSU_Config& configIS = m_cfg;
    fstream FileRead;
    FileRead.open(configInfo, ios::in);

    if (!FileRead)
    {
        ITSAPP_WRN("Unable to open file! %s", configInfo.c_str());
        return false;
    }

    for (int i = 0; i < 2; ++i)
    {
        configIS.MissPhase[i] = -1;
        configIS.MP_Relate[i] = -1;
        configIS.MP_Ring[i] = -1;
    }

    int PhaseNo;
    int PhaseSeq[8];
    char TempStr[255];
    string lineread;
    vector<int> P11, P12, P21, P22;

    //----------------- Read in the parameters---------------
    while (!FileRead.eof())
    {
        getline(FileRead, lineread);

        if (lineread.size() != 0)
        {
            sscanf(lineread.c_str(), "%s", TempStr);
            if (strcmp(TempStr, "Phase_Num") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d ", &PhaseNo);
            }
            else if (strcmp(TempStr, "Phase_Seq") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d %d %d %d %d %d %d %d", &PhaseSeq[0], &PhaseSeq[1],
                    &PhaseSeq[2], &PhaseSeq[3], &PhaseSeq[4], &PhaseSeq[5], &PhaseSeq[6],
                    &PhaseSeq[7]);
            }
            else if (strcmp(TempStr, "Yellow") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Yellow[0],
                    &configIS.Yellow[1], &configIS.Yellow[2], &configIS.Yellow[3],
                    &configIS.Yellow[4], &configIS.Yellow[5], &configIS.Yellow[6],
                    &configIS.Yellow[7]);
            }
            else if (strcmp(TempStr, "Red") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Red[0],
                    &configIS.Red[1], &configIS.Red[2], &configIS.Red[3], &configIS.Red[4],
                    &configIS.Red[5], &configIS.Red[6], &configIS.Red[7]);
            }
            else if (strcmp(TempStr, "Gmin") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Gmin[0],
                    &configIS.Gmin[1], &configIS.Gmin[2], &configIS.Gmin[3], &configIS.Gmin[4],
                    &configIS.Gmin[5], &configIS.Gmin[6], &configIS.Gmin[7]);
            }
            else if (strcmp(TempStr, "Gmax") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &configIS.Gmax[0],
                    &configIS.Gmax[1], &configIS.Gmax[2], &configIS.Gmax[3], &configIS.Gmax[4],
                    &configIS.Gmax[5], &configIS.Gmax[6], &configIS.Gmax[7]);
            }
        }
    }
    FileRead.close();

    // open the priority config file
    fstream FileRead2;
    FileRead2.open(prioConfig, ios::in);
    if (!FileRead2)
    {
        ITSAPP_WRN("Unable to open file! %s", prioConfig.c_str());
        return false;
    }

    double dCoordinationWeight;
    int iCoordinatedPhase[2];
    double iTransitWeight;
    double iTruckWeight;
    int iCoordinationOffset;
    int iCoordinationCycle;
    double dCoordSplit[2];

    //----------------- Read in the parameters---------------
    while (!FileRead2.eof())
    {
        getline(FileRead2, lineread);

        if (lineread.size() != 0)
        {
            sscanf(lineread.c_str(), "%s", TempStr);
            if (strcmp(TempStr, "coordination_weight") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &dCoordinationWeight);
            }
            else if (strcmp(TempStr, "cycle") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d ", &iCoordinationCycle);
            }
            else if (strcmp(TempStr, "offset") == 0)
            {
                sscanf(lineread.c_str(), "%*s %d ", &iCoordinationOffset);
            }

            else if (strcmp(TempStr, "coordinated_phases_number") == 0)
            {
                int iNumber_of_Phases = 0;
                sscanf(lineread.c_str(), "%*s %d ", &iNumber_of_Phases);
                for (int cnt = 0; cnt < iNumber_of_Phases; cnt++)
                {
                    getline(FileRead2, lineread);
                    sscanf(lineread.c_str(), "%s", TempStr);
                    if (strcmp(TempStr, "coordinated_phase1") == 0)
                        sscanf(lineread.c_str(), "%*s %d ", &iCoordinatedPhase[0]);
                    if (strcmp(TempStr, "coordinated_phase2") == 0)
                        sscanf(lineread.c_str(), "%*s %d ", &iCoordinatedPhase[1]);
                }
                for (int cnt = 0; cnt < iNumber_of_Phases; cnt++)
                {
                    getline(FileRead2, lineread);
                    sscanf(lineread.c_str(), "%s", TempStr);
                    if (strcmp(TempStr, "coordinated_phase1_split") == 0)
                        sscanf(lineread.c_str(), "%*s %lf ", &dCoordSplit[0]);

                    if (strcmp(TempStr, "coordinated_phase2_split") == 0)
                        sscanf(lineread.c_str(), "%*s %lf ", &dCoordSplit[1]);
                }
            }
            else if (strcmp(TempStr, "transit_weight") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &iTransitWeight);
            }
            else if (strcmp(TempStr, "truck_weight") == 0)
            {
                sscanf(lineread.c_str(), "%*s %lf ", &iTruckWeight);
            }
        }
    }
    FileRead2.close();

    configIS.dCoordinationWeight = dCoordinationWeight;
    configIS.iCoordinatedPhase[0] = iCoordinatedPhase[0];
    configIS.iCoordinatedPhase[1] = iCoordinatedPhase[1];
    configIS.iTransitWeight = iTransitWeight;
    configIS.iTruckWeight = iTruckWeight;
    configIS.iCoordinationCycle = iCoordinationCycle;
    configIS.iCoordinationOffset = iCoordinationOffset;
    configIS.dCoordinationSplit[0] = dCoordSplit[0];
    configIS.dCoordinationSplit[1] = dCoordSplit[1];

    // Handle the parameters for non-complete phases case
    for (int i = 0; i < 8; ++i)
    {
        if (PhaseSeq[i] > 0 && configIS.Yellow[i] > 0)
        {
            switch (PhaseSeq[i])
            {
                case 1:
                case 2:
                    P11.push_back(PhaseSeq[i]);
                    break;
                case 3:
                case 4:
                    P12.push_back(PhaseSeq[i]);
                    break;
                case 5:
                case 6:
                    P21.push_back(PhaseSeq[i]);
                    break;
                case 7:
                case 8:
                    P22.push_back(PhaseSeq[i]);
                    break;
            }
        }
    }

    //--------Here is different from the one within the GeneratePriReqMod--------//
    if (P11.size() == 0)
    {
        P11.push_back(2); // If P11 is missing, put a dummy "6" which means "2" in it
        configIS.MissPhase[0] = 1;
        configIS.MP_Relate[0] = 5;
        configIS.MP_Ring[0] = 0;
    }
    if (P12.size() == 0)
    {
        P12.push_back(4); // If P12 is missing, put a dummy "8" which means "4" in it
        configIS.MissPhase[0] = 3;
        configIS.MP_Relate[0] = 7;
        configIS.MP_Ring[0] = 0;
    }
    if (P21.size() == 0)
    {
        P21.push_back(6); // If P21 is missing, put a dummy "2" which means "6" in it
        configIS.MissPhase[1] = 5;
        configIS.MP_Relate[1] = 1;
        configIS.MP_Ring[1] = 1;
    }
    if (P22.size() == 0)
    {
        P22.push_back(8); // If P21 is missing, put a dummy "4" which means "8" in it
        configIS.MissPhase[1] = 7;
        configIS.MP_Relate[1] = 3;
        configIS.MP_Ring[1] = 1;
    }

    auto it = P12.begin();
    for (it = P12.begin(); it < P12.end(); it++)
    {
        P11.push_back(*it);
    }
    for (it = P22.begin(); it < P22.end(); it++)
    {
        P21.push_back(*it);
    }

    int i = 0;
    int R1No = P11.size();
    int R2No = P21.size();

    // int SP1=StartPhase[0]-1,SP2=StartPhase[1]-1;  // Real phase -1
    std::vector<int> Phase_Seq_R1(R1No, 0); // Real phase-1
    for (it = P11.begin(), i = 0; it < P11.end() && i < R1No; it++, ++i)
    {
        Phase_Seq_R1[i] = *it - 1;
    }

    std::vector<int> Phase_Seq_R2(R2No, 0);
    for (it = P21.begin(), i = 0; it < P21.end() && i < R2No; it++, ++i)
    {
        Phase_Seq_R2[i] = *it - 1;
    }

    configIS.Ring1No = R1No;
    configIS.Ring2No = R2No;

    configIS.Phase_Seq_R1 = std::vector<int>(R1No, 0);
    configIS.Phase_Seq_R2 = std::vector<int>(R2No, 0);

    for (i = 0; i < 2; ++i)
    {
        if (configIS.MP_Relate[i] >= 0)
        {
            // std::cout<<"HAVE Missed phase:"<<Config.MissPhase<<endl;
            configIS.Yellow[configIS.MissPhase[i]] = configIS.Yellow[configIS.MP_Relate[i]];
            configIS.Red[configIS.MissPhase[i]] = configIS.Red[configIS.MP_Relate[i]];
            configIS.Gmin[configIS.MissPhase[i]] = configIS.Gmin[configIS.MP_Relate[i]];
            configIS.Gmax[configIS.MissPhase[i]] = configIS.Gmax[configIS.MP_Relate[i]];
        }
    }

    for (i = 0; i < R1No; ++i)
    {
        configIS.Phase_Seq_R1[i] = Phase_Seq_R1[i];
    }

    for (i = 0; i < R2No; ++i)
    {
        configIS.Phase_Seq_R2[i] = Phase_Seq_R2[i] % 4;
    }
    return true;
}
