/*
 * mmitssConfig.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include <string>

namespace SieMmitss
{

int getMmitssCtlPerfDataPort();
int getTrajectoryAwarePort();
int getPerfDataPort();
int getPrioReqSrvPort();
int getPrioSolverPort();
int getTrafficControllerInterfacePort();
// the port that the optimal phase timing plane sends to SignalControl (COP)
int getSignalControlOPTPort();
int getSignalControlTrajectoryPort();

std::string getTestSystemSocketAddress();

std::string getConfigDir();
std::string getSlogIdFilePath();
std::string getTestSystemFilePath();

std::string getRsuIdFilePath();
// Vehicle trajectory Aware files
std::string getMapFilePath();
std::string getMapNameFilePath();
std::string getLanePhaseMapFilePath();
std::string getDsrcRangeFilePath();
// Performance Observer files
std::string getConfigInfoFilePath();
std::string getDetNumbersFilePath();
std::string getIpInfoFilePath();
std::string getLaneMovementMapFilePath();
// Priority Request Server files
std::string getPriorityConfigurationFilePath();
std::string getInLaneOutLanePhaseMappingFilePath();
std::string getRequestsFilePath();
//*** Add a file to restore the combined requests: requests_combined.txt BY DJ 2012.2.10
//*** The files will be modified and broadcast, and finally will be used by mpr_solver
std::string getRequestsCombinedFilePath();
// This file stores reqeusts with SplitPhase: used for updating the request list.
std::string getRequestsUpdateFilePath();
std::string getConfigInfoEVFilePath();

std::string getModFilePath();
std::string getModEVFilePath();
std::string getGlpkResultsFilePath();
std::string getPriorityDataFilePath();
std::string getLaneNumberFilePath();

std::string getSignalConfigCOPFilePath();

} // namespace SieMmitss
