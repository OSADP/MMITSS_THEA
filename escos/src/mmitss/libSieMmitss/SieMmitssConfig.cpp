/*
 * mmitssConfig.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "SieMmitssConfig.hpp"
#include "Poco/String.h"
#include "fstream"

namespace SieMmitss
{

static const int MMITSS_CTL_PERF_DATA_PORT = 3344;
static const int MMITSS_TRAJECTORY_AWARE_IN_PORT = 3333;
static const int MMITSS_PERFORMANCE_OBSERVER_PORT = 22222;
static const int MMITSS_PRIO_REQUEST_SRV_PORT = 4444;
static const int MMITSS_TRAFFIC_CONTR_IF_PORT = 44444;
static const int MMITSS_PRIO_SOLVER_PORT = 33333;
static const int MMITSS_SIGNAL_CONTROL_OPT_PORT = 5555;
static const int MMITSS_SIGNAL_CONTROL_TRAJ_PORT = 3334;

static const std::string MMITSS_CONFIG_DIR = "/var/lib/its/us/mmitss/";

static const std::string MMITSS_TEST_IP_FILE = "test_system.txt";

static const std::string MMITSS_RSU_ID_FILE = "rsuid.txt";

static const std::string MMITSS_NMAP_FILE = "map.nmap";
static const std::string MMITSS_LANE_PHASE_MAP_FILE = "Lane_Phase_Mapping.txt";
static const std::string MMITSS_NMAP_NAME_FILE = "nmap_name.txt";
static const std::string MMITSS_DSRC_RANGE_FILE = "DSRC_Range.txt";
static const std::string MMITSS_SLOG_ID_FILE = "logid.txt";

static const std::string MMITSS_CONFIG_INFO_FILE = "ConfigInfo.txt";
static const std::string MMITSS_CONFIG_INFO_EV_FILE = "ConfigInfo_EV.txt";
static const std::string MMITSS_DET_NUMBERS_FILE = "Det_Numbers.txt";
static const std::string MMITSS_IP_INFO_FILE = "IPInfo.txt";
static const std::string MMITSS_LANE_MOVEMENT_FILE = "Lane_Movement_Mapping.txt";
static const std::string MMITSS_PRIO_CONF_FILE = "priorityConfiguration.txt";
static const std::string MMITSS_INOUTLANE_MAPPING_FILE = "InLane_OutLane_Phase_Mapping.txt";
static const std::string MMITSS_REQUESTS_FILE = "requests.txt";
static const std::string MMITSS_REQUESTS_COMBINED_FILE = "requests_combined.txt";
static const std::string MMITSS_REQUESTS_UPDATE_FILE = "requests_update.txt";

static const std::string MMITSS_MOD_FILE = "NewModel.mod";
static const std::string MMITSS_MOD_EV_FILE = "NewModel_EV.mod";
static const std::string MMITSS_PRIO_DATA_FILE = "NewModelData.dat";
static const std::string MMITSS_GLPK_RESULTS_FILE = "Results.txt";
static const std::string MMITSS_LANE_NUMBER_FILE = "Lane_No_MPRSolver.txt";

static const std::string MMITSS_SIGNAL_CFG_COP_FILE = "Signal_Config_COP.txt";

int getMmitssCtlPerfDataPort()
{
    return MMITSS_CTL_PERF_DATA_PORT;
}

int getTrajectoryAwarePort()
{
    return MMITSS_TRAJECTORY_AWARE_IN_PORT;
}

int getPerfDataPort()
{
    return MMITSS_PERFORMANCE_OBSERVER_PORT;
}

int getPrioReqSrvPort()
{
    return MMITSS_PRIO_REQUEST_SRV_PORT;
}

int getPrioSolverPort()
{
    return MMITSS_PRIO_SOLVER_PORT;
}

int getTrafficControllerInterfacePort()
{
    return MMITSS_TRAFFIC_CONTR_IF_PORT;
}

int getSignalControlOPTPort()
{
    return MMITSS_SIGNAL_CONTROL_OPT_PORT;
}

int getSignalControlTrajectoryPort()
{
    return MMITSS_SIGNAL_CONTROL_TRAJ_PORT;
}

std::string getTestSystemSocketAddress()
{
    std::string ipStr;
    std::getline(std::ifstream((MMITSS_CONFIG_DIR + MMITSS_TEST_IP_FILE).c_str()), ipStr);
    return Poco::trim(ipStr);
}

std::string getConfigDir()
{
    return MMITSS_CONFIG_DIR;
}

std::string getTestSystemFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_TEST_IP_FILE;
}

std::string getRsuIdFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_RSU_ID_FILE;
}

std::string getMapFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_NMAP_FILE;
}

std::string getMapNameFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_NMAP_NAME_FILE;
}

std::string getLanePhaseMapFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_LANE_PHASE_MAP_FILE;
}

std::string getDsrcRangeFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_DSRC_RANGE_FILE;
}

std::string getSlogIdFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_SLOG_ID_FILE;
}

std::string getConfigInfoFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_CONFIG_INFO_FILE;
}

std::string getConfigInfoEVFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_CONFIG_INFO_EV_FILE;
}

std::string getDetNumbersFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_DET_NUMBERS_FILE;
}

std::string getIpInfoFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_IP_INFO_FILE;
}

std::string getLaneMovementMapFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_LANE_MOVEMENT_FILE;
}

std::string getPriorityConfigurationFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_PRIO_CONF_FILE;
}

std::string getInLaneOutLanePhaseMappingFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_INOUTLANE_MAPPING_FILE;
}

std::string getRequestsFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_REQUESTS_FILE;
}

std::string getRequestsCombinedFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_REQUESTS_COMBINED_FILE;
}

std::string getRequestsUpdateFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_REQUESTS_UPDATE_FILE;
}

std::string getModFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_MOD_FILE;
}

std::string getModEVFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_MOD_EV_FILE;
}

std::string getGlpkResultsFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_GLPK_RESULTS_FILE;
}

std::string getPriorityDataFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_PRIO_DATA_FILE;
}

std::string getLaneNumberFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_LANE_NUMBER_FILE;
}

std::string getSignalConfigCOPFilePath()
{
    return MMITSS_CONFIG_DIR + MMITSS_SIGNAL_CFG_COP_FILE;
}

} // namespace SieMmitss
