//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#include "Signal.h"

#include <sys/time.h>
#include <cstdio>
#include <ctime>
#include <vector>

#include "facilityLayer.hpp"

namespace WaveApp
{
namespace Mmitss
{

void Phase::FindPhaseNext(int nextPhase[2], int curPhase[2], const PhaseState& curPhaseStatus)
{
    nextPhase[0] = -1; // initial as 1 if there is no phase next
    nextPhase[1] = -1; // initial as 5 if there is no phase next

    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < 4; i++)
        {
            if (curPhaseStatus[i + j * 4] == 2)
            {
                nextPhase[j] = (i + j * 4);
            }

            if (curPhaseStatus[i + j * 4] == 5 || curPhaseStatus[i + j * 4] == 6 ||
                curPhaseStatus[i + j * 4] == 7)
            {
                curPhase[j] = (i + j * 4);
            }
        }
    }
}

double GetSeconds()
{
    struct timeval tv_tt;
    gettimeofday(&tv_tt, NULL);
    return (tv_tt.tv_sec + tv_tt.tv_usec / 1.0e6);
}

char* GetDate()
{
    time_t rawtime = time(NULL);
    return ctime(&rawtime);
}

int FindPhaseIndex(const std::vector<int>& phase, int N, int SP)
// N is the number of the array phase[], SP is the dedired phase to be found in the array
{
    for (long i = 0; i < N; i++)
    {
        if (SP == phase[i])
            return i; // Find the index and return
    }
    // If the SP is not found in the phase[], then return -1;
    //    ITSAPP_TRACE("There is no SP in Phase array!");
    return -1;
}

void Phase::UpdatePhase(const PhaseStatus& newPhaseStatus, const PhaseState& CurPhaseStatus)
{
    /*
     * newPhaseStatus is the status reading from the controller.
     */
    Phase_Status[1] = newPhaseStatus;

    int PhaseNext[2];

    FindPhaseNext(PhaseNext, CurPhase, CurPhaseStatus);

    // m_cfg.PrintRSUConfig();

    // CurPhase[0]=-1;
    // CurPhase[1]=-1;

    for (int i = 0; i < numPhases; i++)
    {
        if (Phase_Status[0].phaseColor[i] == Red && Phase_Status[1].phaseColor[i] == Green)
        { //---- Previous state is Red, and current is Green
            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
            // ITSAPP_LOG("Phase: %d Ring: %d..........Turn to Green!", i+1, indx+1);
        }

        else if (Phase_Status[0].phaseColor[i] == Green && Phase_Status[1].phaseColor[i] == Yellow)
        { //---- Previous state is Green, and current is Yellow

            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
            // ITSAPP_LOG("Phase: %d Ring: %d..........Turn to Yellow!", i+1, indx+1);
        }

        else if (Phase_Status[0].phaseColor[i] == Yellow && Phase_Status[1].phaseColor[i] == Red)
        { //---- Previous state is Yellow, and current is Red
            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
            // ITSAPP_LOG("Phase: %d Ring: %d..........Turn to Red!", i+1, indx+1);
        }
    }

    /*
           if (CurPhase[1-m_cfg.getConfig().MP_Ring]==m_cfg.getConfig().MP_Relate)
       {
           CurPhase[m_cfg.getConfig().MP_Ring]=m_cfg.getConfig().MissPhase;
           StartTime[m_cfg.getConfig().MissPhase]=StartTime[m_cfg.getConfig().MP_Relate];
           ColorTime[m_cfg.getConfig().MissPhase]=ColorTime[m_cfg.getConfig().MP_Relate];
       }
   //*/

    for (int i = 0; i < 2; i++) // 2 rings
    {
        if (CurPhase[1 - m_cfg.getConfig().MP_Ring[i]] == m_cfg.getConfig().MP_Relate[i])
        {
            CurPhase[m_cfg.getConfig().MP_Ring[i]] = m_cfg.getConfig().MissPhase[i];
            StartTime[m_cfg.getConfig().MissPhase[i]] = StartTime[m_cfg.getConfig().MP_Relate[i]];
            ColorTime[m_cfg.getConfig().MissPhase[i]] = ColorTime[m_cfg.getConfig().MP_Relate[i]];
        }
    }

    for (int i = 0; i < numPhases; i++)
    {
        if (Phase_Status[1].phaseColor[i] != Red)
        {
            CurPhase[i / 4] = i;
            PrePhase[i / 4] = CurPhase[i / 4];
        }
        else // added by YF 01.17.2014
        {
            CurPhase[i / 4] = PrePhase[i / 4];
        }

        ColorTime[i] = GetSeconds() - StartTime[i];

        //-----GET the initial phase and time for GLPK solver----//// We use PhaseNext on 2012.4.23
        for (int j = 0; j < 2; j++) //--- 2 Rings-------
        {
            int CP = CurPhase[j]; // {0-7}

            GrnElapse[j] = 0;

            if (CP == m_cfg.getConfig().MissPhase[j]) // TODO: think more
            {
                CP = m_cfg.getConfig().MP_Relate[j];
            }
            if (Phase_Status[1].phaseColor[CP] == Green)
            {
                InitTime[j] = 0;
                InitPhase[j] = CurPhase[j];
                GrnElapse[j] = ColorTime[CP];
            }

            else if (Phase_Status[1].phaseColor[CP] == Yellow)
            {
                if (j == 0)
                {
                    int phase = CP % 4; // consider two rings.

                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R1, m_cfg.getConfig().Ring1No, phase);

                    InitPhase[j] =
                        m_cfg.getConfig().Phase_Seq_R1[(phaseIdx + 1) % m_cfg.getConfig().Ring1No] +
                        4 * j;

                    if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                    {
                        InitPhase[j] = PhaseNext[j];
                    }
                    // InitTime[j]=m_cfg.getConfig().Yellow[InitPhase[j]]-ColorTime[CP]+m_cfg.getConfig().Red[InitPhase[j]];
                    InitTime[j] = std::max(m_cfg.getConfig().Yellow[InitPhase[j]] - ColorTime[CP] +
                                               m_cfg.getConfig().Red[InitPhase[j]],
                        0.0);
                }
                else // j==1
                {
                    int phase = CP % 4; // consider two rings. CP {4-7}

                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R2, m_cfg.getConfig().Ring2No, phase);

                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R2[(phaseIdx + 1) % m_cfg.getConfig().Ring2No] +
                            4 * j;

                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                    }

                    // InitTime[j]=m_cfg.getConfig().Yellow[InitPhase[j]]-ColorTime[CP]+m_cfg.getConfig().Red[InitPhase[j]];
                    InitTime[j] = std::max(m_cfg.getConfig().Yellow[InitPhase[j]] - ColorTime[CP] +
                                               m_cfg.getConfig().Red[InitPhase[j]],
                        0.0);
                }
            }

            else if (Phase_Status[1].phaseColor[CP] == Red)
            {
                if (j == 0)
                {
                    int phase = CP % 4; // consider two rings.
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R1, m_cfg.getConfig().Ring1No, phase);

                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R1[(phaseIdx + 1) % m_cfg.getConfig().Ring1No];

                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                    }

                    InitTime[j] =
                        std::max(m_cfg.getConfig().Red[InitPhase[j]] - ColorTime[CP], 0.0);
                }
                else // j==1
                {
                    int phase = CP % 4; // consider two rings.
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R2, m_cfg.getConfig().Ring2No, phase);
                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R2[(phaseIdx + 1) % m_cfg.getConfig().Ring2No] +
                            4 * j;
                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                    }

                    InitTime[j] =
                        std::max(m_cfg.getConfig().Red[InitPhase[j]] - ColorTime[CP], 0.0);
                }
            }
        }
    }
    /*
    if (CurPhase[1-m_cfg.getConfig().MP_Ring]==m_cfg.getConfig().MP_Relate)
    {
        ColorTime[m_cfg.getConfig().MissPhase]=ColorTime[m_cfg.getConfig().MP_Relate];
    } //*/

    for (int i = 0; i < 2; i++) // 2 rings
    {
        if (CurPhase[1 - m_cfg.getConfig().MP_Ring[i]] == m_cfg.getConfig().MP_Relate[i])
        {
            ColorTime[m_cfg.getConfig().MissPhase[i]] = ColorTime[m_cfg.getConfig().MP_Relate[i]];
        }
    }

    Phase_Status[0] = Phase_Status[1]; // WORK
}

// TODO: Check what's the difference to the one above...
void Phase::UpdatePhaseMPR(
    const PhaseStatus& newPhaseStatus, const PhaseState& CurPhaseStatus, double* globalGmax)
{
    /*
     * newPhaseStatus is the status reading from the controller.
     */
    Phase_Status[1] = newPhaseStatus;
    int PhaseNext[2];
    FindPhaseNext(PhaseNext, CurPhase, CurPhaseStatus);

    for (int i = 0; i < numPhases; i++)
    {
        if (Phase_Status[0].phaseColor[i] == Red && Phase_Status[1].phaseColor[i] == Green)
        { //---- Previous state is Red, and current is Green
            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
        }
        else if (Phase_Status[0].phaseColor[i] == Green && Phase_Status[1].phaseColor[i] == Yellow)
        { //---- Previous state is Green, and current is Yellow
            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
        }
        else if (Phase_Status[0].phaseColor[i] == Yellow && Phase_Status[1].phaseColor[i] == Red)
        { //---- Previous state is Yellow, and current is Red
            StartTime[i] = GetSeconds();
            ColorTime[i] = 0;
        }
    }
    for (int i = 0; i < 2; i++) // 2 rings
    {
        if (CurPhase[1 - m_cfg.getConfig().MP_Ring[i]] == m_cfg.getConfig().MP_Relate[i])
        {
            CurPhase[m_cfg.getConfig().MP_Ring[i]] = m_cfg.getConfig().MissPhase[i];
            StartTime[m_cfg.getConfig().MissPhase[i]] = StartTime[m_cfg.getConfig().MP_Relate[i]];
            ColorTime[m_cfg.getConfig().MissPhase[i]] = ColorTime[m_cfg.getConfig().MP_Relate[i]];
        }
    }
    for (int i = 0; i < numPhases; i++)
    {
        if (Phase_Status[1].phaseColor[i] != Red)
        {
            CurPhase[i / 4] = i;
        }
        ColorTime[i] = GetSeconds() - StartTime[i];
        //-----GET the initial phase and time for GLPK solver----//// We use PhaseNext on 2012.4.23
        for (int j = 0; j < 2; j++) //--- 2 Rings-------
        {
            int CP = CurPhase[j]; // {0-7}
            GrnElapse[j] = 0;
            if (CP == m_cfg.getConfig().MissPhase[j]) // TODO: think more
            {
                CP = m_cfg.getConfig().MP_Relate[j];
            }
            if (Phase_Status[1].phaseColor[CP] == Green)
            {
                InitTime[j] = 0;
                InitPhase[j] = CurPhase[j];
                GrnElapse[j] = ColorTime[CP];
            }
            else if (Phase_Status[1].phaseColor[CP] == Yellow)
            {
                if (j == 0)
                {
                    int phase = CP % 4; // consider two rings.
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R1, m_cfg.getConfig().Ring1No, phase);
                    InitPhase[j] =
                        m_cfg.getConfig().Phase_Seq_R1[(phaseIdx + 1) % m_cfg.getConfig().Ring1No] +
                        4 * j;
                    if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                    {
                        InitPhase[j] = PhaseNext[j];
                    }
                    if (CurPhaseStatus[InitPhase[j]] == 4)
                    {
                        if (phaseIdx + 1 == m_cfg.getConfig().Ring1No)
                        {
                            InitPhase[j] = m_cfg.getConfig().Phase_Seq_R1[0] + 4 * j;
                        }
                        else
                        {
                            InitPhase[j] =
                                m_cfg.getConfig()
                                    .Phase_Seq_R1[(phaseIdx + 2) % m_cfg.getConfig().Ring1No] +
                                4 * j;
                        }
                    }
                    InitTime[j] = std::max(m_cfg.getConfig().Yellow[InitPhase[j]] - ColorTime[CP] +
                                               m_cfg.getConfig().Red[InitPhase[j]],
                        0.0);
                }
                else // j==1
                {
                    int phase = CP % 4; // consider two rings. CP {4-7}
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R2, m_cfg.getConfig().Ring2No, phase);
                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R2[(phaseIdx + 1) % m_cfg.getConfig().Ring2No] +
                            4 * j;

                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                        if (CurPhaseStatus[InitPhase[j]] == 4)
                        {
                            if (phaseIdx + 1 == m_cfg.getConfig().Ring2No)
                            {
                                InitPhase[j] = m_cfg.getConfig().Phase_Seq_R2[0] + 4 * j;
                            }
                            else
                            {
                                InitPhase[j] =
                                    m_cfg.getConfig()
                                        .Phase_Seq_R2[(phaseIdx + 2) % m_cfg.getConfig().Ring2No] +
                                    4 * j;
                            }
                        }
                    }
                    InitTime[j] = std::max(m_cfg.getConfig().Yellow[InitPhase[j]] - ColorTime[CP] +
                                               m_cfg.getConfig().Red[InitPhase[j]],
                        0.0);
                }
            }

            else if (Phase_Status[1].phaseColor[CP] == Red)
            {
                if (j == 0)
                {
                    int phase = CP % 4; // consider two rings.
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R1, m_cfg.getConfig().Ring1No, phase);

                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R1[(phaseIdx + 1) % m_cfg.getConfig().Ring1No];

                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                    }
                    if (CurPhaseStatus[InitPhase[j]] == 4)
                    {
                        if (phaseIdx + 1 == m_cfg.getConfig().Ring1No)
                        {
                            InitPhase[j] = m_cfg.getConfig().Phase_Seq_R1[0] + 4 * j;
                        }
                        else
                        {
                            InitPhase[j] =
                                m_cfg.getConfig()
                                    .Phase_Seq_R1[(phaseIdx + 2) % m_cfg.getConfig().Ring1No] +
                                4 * j;
                        }
                    }
                    InitTime[j] =
                        std::max(m_cfg.getConfig().Red[InitPhase[j]] - ColorTime[CP], 0.0);
                }
                else // j==1
                {
                    int phase = CP % 4; // consider two rings.
                    int phaseIdx = FindPhaseIndex(
                        m_cfg.getConfig().Phase_Seq_R2, m_cfg.getConfig().Ring2No, phase);
                    if (phaseIdx >= 0)
                    {
                        InitPhase[j] =
                            m_cfg.getConfig()
                                .Phase_Seq_R2[(phaseIdx + 1) % m_cfg.getConfig().Ring2No] +
                            4 * j;
                        if (InitPhase[j] != PhaseNext[j] && PhaseNext[j] >= 0)
                        {
                            InitPhase[j] = PhaseNext[j];
                        }
                        if (CurPhaseStatus[InitPhase[j]] == 4)
                        {
                            if (phaseIdx + 1 == m_cfg.getConfig().Ring2No)
                            {
                                InitPhase[j] = m_cfg.getConfig().Phase_Seq_R2[0] + 4 * j;
                            }
                            else
                            {
                                InitPhase[j] =
                                    m_cfg.getConfig()
                                        .Phase_Seq_R2[(phaseIdx + 2) % m_cfg.getConfig().Ring2No] +
                                    4 * j;
                            }
                        }
                    }
                    InitTime[j] =
                        std::max(m_cfg.getConfig().Red[InitPhase[j]] - ColorTime[CP], 0.0);
                }
            }
        }
    }
    /*
    if (CurPhase[1-ConfigIS.MP_Ring]==ConfigIS.MP_Relate)
    {
        ColorTime[ConfigIS.MissPhase]=ColorTime[ConfigIS.MP_Relate];
    }
    */
    for (int i = 0; i < 2; i++) // 2 rings
    {
        if (CurPhase[1 - m_cfg.getConfig().MP_Ring[i]] == m_cfg.getConfig().MP_Relate[i])
        {
            ColorTime[m_cfg.getConfig().MissPhase[i]] = ColorTime[m_cfg.getConfig().MP_Relate[i]];
        }
    }
    Phase_Status[0] = Phase_Status[1]; // WORK

    // populate the global gmax
    // memset(dGlobalGmax, 0.0, sizeof(dGlobalGmax));
    // dGlobalGmax={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 8; i++)
        globalGmax[i] = 0.0;
    for (int i = 0; i < m_cfg.getConfig().Ring1No; i++)
    {
        int k = m_cfg.getConfig().Phase_Seq_R1[i];
        globalGmax[i] = m_cfg.getConfig().Gmax[k];
    }
    for (int i = 0; i < m_cfg.getConfig().Ring2No; i++)
    {
        int k = m_cfg.getConfig().Phase_Seq_R2[i];
        globalGmax[i + 4] = m_cfg.getConfig().Gmax[k + 4];
    }

    ITSAPP_LOG("********Current Signal status:************");
    ITSAPP_LOG("%d\t\t%d\t\t%d\t\t%d\t\t%d\t\t%d\t\t%d\t\t%d\t\t", 1, 2, 3, 4, 5, 6, 7, 8);
    auto f = [](int status) {
        switch (status)
        {
            case 1:
                return "R";
                break;
            case 3:
                return "G";
                break;
            case 4:
                return "Y";
                break;
            default:
                return "N";
        }
    };
    ITSAPP_LOG("%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t", f(Phase_Status[1].phaseColor[0]),
        f(Phase_Status[1].phaseColor[1]), f(Phase_Status[1].phaseColor[2]),
        f(Phase_Status[1].phaseColor[3]), f(Phase_Status[1].phaseColor[4]),
        f(Phase_Status[1].phaseColor[5]), f(Phase_Status[1].phaseColor[6]),
        f(Phase_Status[1].phaseColor[7]));
    ITSAPP_LOG("********Color Time:************");
    ITSAPP_LOG("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t", ColorTime[0], ColorTime[1], ColorTime[2],
        ColorTime[3], ColorTime[4], ColorTime[5], ColorTime[6], ColorTime[7]);

    //~ for (int i=0;i<8;i++)
    //~ cout<< " " <<dGlobalGmax[i]<<endl;
    //~ cout<<endl;
}

Phase::Phase(const RsuConfig& cfg)
    : m_cfg(cfg)
{
    for (int i = 0; i < numPhases; i++)
    {
        Phase_Status[0].phaseColor[i] = 0;
        Phase_Status[1].phaseColor[i] = 0;
        StartTime[i] = GetSeconds();
        ColorTime[i] = 0;
    }
    for (int i = 0; i < 2; i++)
    {
        CurPhase[i] = 0;
        PrePhase[i] = 0;
        InitPhase[i] = 0;
        InitTime[i] = 0;
        // color[i]=R;
    }
}

Phase& Phase::operator=(Phase& newPhase)
{
    for (int i = 0; i < 2; i++)
    {
        Phase_Status[i] = newPhase.Phase_Status[i];
        CurPhase[i] = newPhase.CurPhase[i];

        InitPhase[i] = newPhase.InitPhase[i];
        InitTime[i] = newPhase.InitTime[i];
    }
    for (int i = 0; i < numPhases; i++)
    {
        StartTime[i] = newPhase.StartTime[i];
        ColorTime[i] = newPhase.ColorTime[i];
    }
    return *this;
}

/*
void Phase::RecordPhase(char *filename)
{
fstream fs;
fs.open(filename,fstream::out|fstream::app);

fs<<"At Time: "<<time(NULL)<<"  Current Phase: "<<CurPhase[0]+1<<"\t"<<ColorTime[CurPhase[0]]<<"\t"
<<CurPhase[1]+1<<"\t"<<ColorTime[CurPhase[1]]<<"\t";
cout<<" Current Phase: "<<CurPhase[0]+1<<" "<<CurPhase[1]+1<<"\t"
<<"Initial Phase:\t"<< InitPhase[0]+1<<"  "<< InitPhase[1]+1<<"\n";

for(int i=0;i<numPhases;i++)
{
//fs<<" "<<Phase_Status[1].phaseColor[i]<<" "<<StartTime[i]<<"
"<<Phase_Status[0].phaseColor[i]<<endl;
fs<<"\t"<<Phase_Status[1].phaseColor[i]<<"\t"<<ColorTime[i];
cout<<" "<<Phase_Status[1].phaseColor[i];
}
fs<<endl;

fs.close();
}
*/

void Phase::RecordPhase(char* filename)
{
    FILE* fs;
    fs = fopen(filename, "a+");

    // fs<<"At Time: "<<time(NULL)<<"  Current Phase:
    // "<<CurPhase[0]+1<<"\t"<<ColorTime[CurPhase[0]]<<"\t"
    //    <<CurPhase[1]+1<<"\t"<<ColorTime[CurPhase[1]]<<"\t";

    fprintf(fs, "At %.2f \t CP:\t %d \t %.2f \t  %d \t %.2f ", GetSeconds(), CurPhase[0] + 1,
        ColorTime[CurPhase[0]], (CurPhase[1] + 1), ColorTime[CurPhase[1]]);

    std::cout << " Current Phase: " << CurPhase[0] + 1 << " " << CurPhase[1] + 1 << "\t"
              << "Initial Phase:\t" << InitPhase[0] + 1 << "  " << InitPhase[1] + 1 << "\n";

    for (int i = 0; i < numPhases; i++)
    {
        // fs<<"\t"<<Phase_Status[1].phaseColor[i]<<"\t"<<ColorTime[i];
        fprintf(fs, "\t %d \t %.2f", Phase_Status[1].phaseColor[i], ColorTime[i]);
    }

    for (int j = 0; j < 2; j++)
    {
        fprintf(fs, "\t %d \t %.2f", InitPhase[j] + 1, InitTime[j]);
    }

    for (int j = 0; j < 2; j++)
    {
        fprintf(fs, "\t %d \t %.2f", InitPhase[j] + 1, GrnElapse[j]);
    }

    fprintf(fs, "\n");

    fclose(fs);
}

void Phase::Display()
{
    for (int i = 0; i < numPhases; i++)
    {
        if (Phase_Status[1].phaseColor[i] == Red)
        {
            ITSAPP_LOG("Phase %d: R :%lf:%lf", i + 1, StartTime[i], ColorTime[i]);
        }
        else if (Phase_Status[1].phaseColor[i] == Green)
        {
            ITSAPP_LOG("Phase %d: G :%lf:%lf", i + 1, StartTime[i], ColorTime[i]);
        }
        else if (Phase_Status[1].phaseColor[i] == Yellow)
        {
            ITSAPP_LOG("Phase %d: Y :%lf:%lf", i + 1, StartTime[i], ColorTime[i]);
        }
        else
        {
            ITSAPP_LOG("ERROR POLL PHASE! Multiple color for this phase %d", i + 1);
        }
    }
    ITSAPP_LOG("Current phase: %d & %d", CurPhase[0] + 1, CurPhase[1] + 1);
}

} // namespace Mmitss
} // namespace WaveApp
