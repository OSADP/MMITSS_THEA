//******************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with
// rights granted for USDOT OSADP distribution with the Apache 2.0 open source
// license.
//
//******************************************************************************

#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <istream>
#include <math.h>

#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include "Signal.h"
#include "Array.h"

#include "Mib.h"

#include "facilityLayer.hpp"

namespace WaveApp
{
namespace Mmitss
{

static const int MAX_RESULT_ITEMS = 50;
static const int MAX_ITEMS = 50;

void PackEventList(std::vector<byte>& tmp_event_data)
{
    byte* pByte; // pointer used (by cast)to get at each byte
                 // of the shorts, longs, and blobs
    unsigned short tempUShort;
    long tempLong;
    // header 2 bytes
    tmp_event_data.emplace_back(0xFF);
    tmp_event_data.emplace_back(0xFF);
    tmp_event_data.emplace_back(0xFF);
    // MSG ID: 0x03 for signal event data send to Signal Control Interface
    tmp_event_data.emplace_back(0x03);
    // No. events in R1
    int numberOfPhase = 4;
    int tempTime = 0;
    int tempCmd = 3;
    tempUShort = (unsigned short)numberOfPhase;
    pByte = (byte*)&tempUShort;
    tmp_event_data.emplace_back(*(pByte + 1));
    tmp_event_data.emplace_back(*(pByte + 0));
    // Events in R1
    for (int iii = 0; iii < 4; iii++)
    {
        // Time
        tempLong = (long)(tempTime);
        pByte = (byte*)&tempLong;
        tmp_event_data.emplace_back(*(pByte + 3));
        tmp_event_data.emplace_back(*(pByte + 2));
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
        // phase
        tempUShort = (unsigned short)iii + 1;
        pByte = (byte*)&tempUShort;
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
        // action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte*)&tempUShort;
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
    }

    tempUShort = (unsigned short)numberOfPhase;
    pByte = (byte*)&tempUShort;
    tmp_event_data.emplace_back(*(pByte + 1));
    tmp_event_data.emplace_back(*(pByte + 0));
    // Events in R
    for (int iii = 0; iii < 4; iii++)
    {
        // Time
        tempLong = (long)(tempTime);
        pByte = (byte*)&tempLong;
        tmp_event_data.emplace_back(*(pByte + 3));
        tmp_event_data.emplace_back(*(pByte + 2));
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
        // phase
        tempUShort = (unsigned short)iii + 5;
        pByte = (byte*)&tempUShort;
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
        // action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte*)&tempUShort;
        tmp_event_data.emplace_back(*(pByte + 1));
        tmp_event_data.emplace_back(*(pByte + 0));
    }
    tmp_event_data.emplace_back(0);
}

MmitssMib::MmitssMib(const SocketAddress& controllerAddress)
    : m_controllerAddress(controllerAddress)
{
}

int MmitssMib::GetSignalColor(int PhaseStatusNo)
{
    int ColorValue = Red;

    switch (PhaseStatusNo)
    {
        case 2:
        case 3:
        case 4:
        case 5:
            ColorValue = Red;
            break;
        case 6:
        case 11:
            ColorValue = Yellow;
            break;
        case 7:
        case 8:
            ColorValue = Green;
            break;
        default:
            ColorValue = 0;
    }
    return ColorValue;
}

// WILL use the global parameter phase_read
void MmitssMib::PhaseTimingStatusRead()
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;

    init_snmp("ASC"); // Initialize the SNMP library

    snmp_sess_init(&session); // Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    // session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
    session.version = SNMP_VERSION_1; // for ASC/3 software  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */
    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("ASC", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;
    //---#define CUR_TIMING_PLAN     "1.3.6.1.4.1.1206.3.5.2.1.22.0"      // return the current
    // timing plan
    char ctemp[50];
    for (int i = 1; i <= 8; i++)
    {
        sprintf(ctemp, "%s%d", PHASE_STA_TIME2_ASC, i);
        if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit
                                                       // as enabled or not: "1"  enable; "0" not
                                                       // used
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }
        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        int out[MAX_RESULT_ITEMS];
        int i = 0;
        // for(vars = response->variables; vars; vars = vars->next_variable)
        // print_variable(vars->name, vars->name_length, vars);
        /* manipuate the information ourselves */
        for (vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }
        //****** GET the results from controller *************//
        for (i = 0; i < 8; i++)
        {
            m_phaseRead.phaseColor[i] = GetSignalColor(out[i]);
            m_curPhaseStatus[i] = out[i]; // NOT converted to GYR
        }
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    /*
     * Clean up:    *  1) free the response.   *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);
    snmp_close(ss);
    SOCK_CLEANUP;
}

int MmitssMib::CurTimingPlanRead()
{
    // USING the standard oid : ONLY read timing plan 1.
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;
    int currentTimePlan = 0; // return value

    init_snmp("RSU"); // Initialize the SNMP library

    snmp_sess_init(&session); // Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    // session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
    session.version = SNMP_VERSION_1; // for ASC/3 software  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */

    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("RSU", &session);
        SOCK_CLEANUP;
        // exit(1);
        return -1;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    //---#define CUR_TIMING_PLAN     "1.3.6.1.4.1.1206.3.5.2.1.22.0"      // return the current
    // timing plan

    char ctemp[50];

    sprintf(ctemp, "%s", CUR_TIMING_PLAN); // WORK
    // sprintf(ctemp,"%s",PHASE_MIN_GRN_ASC_TEST); // WORK
    // sprintf(ctemp,"%s%d.%d",PHASE_MIN_GRN_ASC,2,1);        //  WORK

    if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as
                                                   // enabled or not: "1"  enable; "0" not used
    {
        ITSAPP_WRN("SNMP parse oid error");
        snmp_perror(ctemp);
        SOCK_CLEANUP;
        // exit(1);
        return -1;
    }

    snmp_add_null_var(pdu, anOID, anOID_len);

    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        int out[MAX_RESULT_ITEMS];
        int i = 0;
        //  for(vars = response->variables; vars; vars = vars->next_variable)
        //     print_variable(vars->name, vars->name_length, vars);

        /* manipuate the information ourselves */
        for (vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }

        // CUR_TIMING_PLAN_NO=out[0];
        currentTimePlan = out[0];
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    /*
     * Clean up:    *  1) free the response.   *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);

    snmp_close(ss);

    SOCK_CLEANUP;

    return currentTimePlan;
}

void MmitssMib::IntersectionPedConfigRead(int CurTimingPlanNo, const std::string& ConfigOutFile)
{
    NOTUSED(CurTimingPlanNo);

    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;
    int i;

    /*
     * Initialize the SNMP library
     */
    init_snmp("RSU");

    /*
     * Initialize a "session" that defines who we're going to talk to
     */
    snmp_sess_init(&session); /* set up defaults */
    // char *ip = m_rampmeterip.GetBuffer(m_rampmeterip.GetLength());
    // char *port = m_rampmeterport.GetBuffer(m_rampmeterport.GetLength());
    session.peername = strdup(m_controllerAddress.toString().c_str());
    /* set the SNMP version number */
    // session.version = SNMP_VERSION_2c; //for ASC intersection
    session.version = SNMP_VERSION_1; // for ASC/3 software  /* set the SNMP version number */
    // session.version = SNMP_VERSION_3;  // for Rampmeter

    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */

    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("RSU", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    //// Phase options: last ".X" is phase, the last bit of return result is "0", the phase is not
    /// enabled.
    //#define PHASE_ENABLED       "1.3.6.1.4.1.1206.4.2.1.1.2.1.21."
    ////------The following from ASC3------------//
    //#define PHASE_MIN_GRN_ASC     "1.3.6.1.4.1.1206.3.5.2.1.2.1.9."   // need last "x.p" x is the
    // timing plan number,p is the phase number: x get from CUR_TIMING_PLAN
    //#define PHASE_MAX_GRN_ASC     "1.3.6.1.4.1.1206.3.5.2.1.2.1.15."
    //#define PHASE_RED_CLR_ASC     "1.3.6.1.4.1.1206.3.5.2.1.2.1.19."
    //#define PHASE_YLW_XGE_ASC     "1.3.6.1.4.1.1206.3.5.2.1.2.1.18."
    char ctemp[50];

    for (i = 1; i <= 8; i++)
    {
        // Ped Walk time, only read for plan 1 NTCIP standard command
        sprintf(ctemp, "%s%d", PED_WALK, i);

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }
        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++)
    {
        // Ped Clearance time , only read for plan 1 NTCIP standard command
        sprintf(ctemp, "%s%d", PED_CLEAR, i);
        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }
        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++)
    {
        sprintf(ctemp, "%s%d", PHASE_ENABLED, i);

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit
                                                       // as enabled or not: "1"  enable; "0" not
                                                       // used
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        std::vector<int> out(MAX_ITEMS, 0);
        //        for(vars = response->variables; vars; vars = vars->next_variable)
        //            print_variable(vars->name, vars->name_length, vars);

        /* manipuate the information ourselves */
        for (i = 0, vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }
        // FOR ASC INTERSECTIONS_ Draw the lights
        // UpdateIntersectionStatus(out[0],out[1]);
        // cout<<"Minimum Green Time for Phase 1 is:\t"<<out[0]<<" ****** "<<out[1]<<endl;
        // *** int PhaseSeq[8],MinGrn[8],Yellow[8],RedClr[8],GrnMax[8];***//

        int Result[3][8]; //*** Sequence:
        // MinGrn[8],Yellow[8],RedClr[8],GrnMax[8],PhaseSeq[8],PedWalk[8],PedClr[8];***//
        // Yiheng added for row 6 and 7 for ped information 10/24/2014
        /*
        Result[0][8]  ----> MinGrn[8]
        Result[1][8]  ----> Yellow[8]
        Result[2][8]  ----> RedClr[8]
        Result[3][8]  ----> GrnMax[8]
        Result[4][8]  ----> PhaseSeq[8]
        Result[5][8]  ---->
        */
        for (i = 0; i < 3; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                Result[i][j] = out[j + i * 8];
            }
        }

        int PedWalk[8], PedClr[8];

        for (i = 0; i < 8; i++)
        {
            //*** Last bit is 1, odd number, the '&' is 0:
            if (Result[2][i] & 1) //*** Be careful here: it is bit "&" not "&&". ***//
            {
                PedWalk[i] = Result[0][i];
                PedClr[i] = Result[1][i];
            }
            else
            {
                PedWalk[i] = 0;
                PedClr[i] = 0;
            }
        }

        fstream fs_config;
        // TODO:,char *ConfigOutFile
        // fs_config.open(ConfigFile,ios::out);
        fs_config.open(ConfigOutFile, ios::out | ios::app);

        // Added by Yiheng for Ped info
        fs_config << "PedWalk";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << PedWalk[i];
        fs_config << endl;

        fs_config << "PedClr";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << PedClr[i];
        fs_config << endl;

        fs_config.close();
    }
    else
    {
        /*
         * FAILURE: print what went wrong!
         */

        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    // Because of the number of item limitation, here need to send again.

    /*
     * Clean up:
     *  1) free the response.
     *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);

    snmp_close(ss);

    SOCK_CLEANUP;
}

void MmitssMib::IntersectionConfigRead(int CurTimingPlanNo, const std::string& ConfigOutFile)
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;
    int i;

    /*
     * Initialize the SNMP library
     */
    init_snmp("RSU");

    /*
     * Initialize a "session" that defines who we're going to talk to
     */
    snmp_sess_init(&session); /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    /* set the SNMP version number */
    // session.version = SNMP_VERSION_2c; //for ASC intersection
    session.version = SNMP_VERSION_1; // for ASC/3 software  /* set the SNMP version number */
    // session.version = SNMP_VERSION_3;  // for Rampmeter

    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */

    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("RSU", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    //// Phase options: last ".X" is phase, the last bit of return result is "0", the phase is not
    /// enabled.
    //#define PHASE_ENABLED       "1.3.6.1.4.1.1206.4.2.1.1.2.1.21."
    ////------The following from ASC3------------//
    //#define PHASE_MIN_GRN_ASC   "1.3.6.1.4.1.1206.3.5.2.1.2.1.9."   // need last "x.p" x is the
    // timing plan number,p is the phase number: x get from CUR_TIMING_PLAN
    //#define PHASE_MAX_GRN_ASC   "1.3.6.1.4.1.1206.3.5.2.1.2.1.15."
    //#define PHASE_RED_CLR_ASC   "1.3.6.1.4.1.1206.3.5.2.1.2.1.19."
    //#define PHASE_YLW_XGE_ASC   "1.3.6.1.4.1.1206.3.5.2.1.2.1.18."
    char ctemp[50];

    for (i = 1; i <= 8; i++) // PHASE_MIN_GRN_ASC
    {
        sprintf(ctemp, "%s%d.%d", PHASE_MIN_GRN_ASC, CurTimingPlanNo, i); //  in seconds

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++) // PHASE_YLW_XGE_ASC
    {
        sprintf(ctemp, "%s%d.%d", PHASE_YLW_XGE_ASC, CurTimingPlanNo,
            i); // in tenth of seconds, for example if 35 should be 3.5 seconds

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++) // PHASE_RED_CLR_ASC
    {
        sprintf(ctemp, "%s%d.%d", PHASE_RED_CLR_ASC, CurTimingPlanNo,
            i); // in tenth of seconds, for example if 35 should be 3.5 seconds

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++) // PHASE_MAX_GRN_ASC
    {
        sprintf(ctemp, "%s%d.%d", PHASE_MAX_GRN_ASC, CurTimingPlanNo, i); // in seconds

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len))
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    for (i = 1; i <= 8; i++)
    {
        sprintf(ctemp, "%s%d", PHASE_ENABLED, i);

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit
                                                       // as enabled or not: "1"  enable; "0" not
                                                       // used
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            // exit(1);
            return;
        }

        snmp_add_null_var(pdu, anOID, anOID_len);
    }

    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        int out[MAX_RESULT_ITEMS];
        //  for(vars = response->variables; vars; vars = vars->next_variable)
        //     print_variable(vars->name, vars->name_length, vars);

        /* manipuate the information ourselves */
        for (i = 0, vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }
        // FOR ASC INTERSECTIONS_ Draw the lights
        // UpdateIntersectionStatus(out[0],out[1]);
        // cout<<"Minimum Green Time for Phase 1 is:\t"<<out[0]<<" ****** "<<out[1]<<endl;
        // *** int PhaseSeq[8],MinGrn[8],Yellow[8],RedClr[8],GrnMax[8];***//

        int Result[5][8]; //*** Sequence: MinGrn[8],Yellow[8],RedClr[8],GrnMax[8],PhaseSeq[8],;***//
        /*
        Result[0][8] --> MinGrn[8]
        Result[1][8] --> Yellow[8]
        Result[2][8] --> RedClr[8]
        Result[3][8] --> GrnMax[8]
        Result[4][8] --> PhaseSeq[8]
        */
        for (i = 0; i < 5; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                Result[i][j] = out[(i * 8) + j];
            }
        }
        int phaseSeq[8];
        int MinGrn[8];
        int GrnMax[8];
        float Yellow[8];
        float RedClr[8];

        int TotalNo = 0;

        for (i = 0; i < 8; i++)
        {
            //*** Last bit is 1, odd number, the '&' is 0:
            if (Result[4][i] & 1) //*** Be careful here: it is bit "&" not "&&". ***//
            {
                phaseSeq[i] = i + 1;

                MinGrn[i] = Result[0][i];
                Yellow[i] = float(Result[1][i] / 10.0);
                RedClr[i] = float(Result[2][i] / 10.0);
                GrnMax[i] = Result[3][i];

                TotalNo++;
            }
            else
            {
                phaseSeq[i] = 0;
                MinGrn[i] = 0;
                Yellow[i] = 0;
                RedClr[i] = 0;
                GrnMax[i] = 0;
            }
        }

        fstream fs_config;
        // TODO:,char *ConfigOutFile
        // fs_config.open(ConfigFile,ios::out);
        fs_config.open(ConfigOutFile, ios::out);

        fs_config << "Phase_Num " << TotalNo << endl;

        fs_config << "Phase_Seq";
        for (i = 0; i < 8; i++)
            fs_config << " " << phaseSeq[i];
        fs_config << endl;

        fs_config << "Gmin";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << MinGrn[i];
        fs_config << endl;

        fs_config << "Yellow";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << Yellow[i];
        fs_config << endl;

        fs_config << "Red";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << RedClr[i];
        fs_config << endl;

        fs_config << "Gmax";
        for (i = 0; i < 8; i++)
            fs_config << "\t" << GrnMax[i];
        fs_config << endl;

        fs_config.close();
    }
    else
    {
        /*
         * FAILURE: print what went wrong!
         */

        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    /*
     * Clean up:
     *  1) free the response.
     *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);

    snmp_close(ss);

    SOCK_CLEANUP;
}

void MmitssMib::IntersectionPhaseControl(int phase_control, int Total, char YES)
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;
    char tmp_int[16];

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    //    netsnmp_variable_list *vars;
    int status;
    int failures = 0;

    sprintf(tmp_int, "%d", Total);

    ITSAPP_LOG("CMD Applied to Phase: %d %d", Total, YES); //<<buffer;

    /*
     * Initialize the SNMP library
     */
    init_snmp("RSU");

    /*
     * Initialize a "session" that defines who we're going to talk to
     */
    snmp_sess_init(&session); /* set up defaults */

    session.peername = strdup(m_controllerAddress.toString().c_str());

    /* set the SNMP version number */
    // session.version = SNMP_VERSION_2c;  //For Intersection Control
    session.version = SNMP_VERSION_1; // for ASC/3 software  /* set the SNMP version number */

    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */

    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("RSU", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to SET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_SET);
    anOID_len = MAX_OID_LEN;
    if (Hold == phase_control)
    {
        if (!snmp_parse_oid(MIB_PHASE_HOLD, anOID, &anOID_len))
        {
            ITSAPP_WRN("Error on parse oid: %s", MIB_PHASE_HOLD);
            snmp_perror(MIB_PHASE_HOLD);
            failures++;
        }
        ITSAPP_LOG("HOLD control! Number (%d), AT time:[%.2f] ", Total, GetSeconds());
    }
    else if (ForceOff == phase_control)
    {
        if (!snmp_parse_oid(MIB_PHASE_FORCEOFF, anOID, &anOID_len))
        {
            ITSAPP_WRN("Error on parse oid: %s", MIB_PHASE_FORCEOFF);
            snmp_perror(MIB_PHASE_FORCEOFF);
            failures++;
        }
        ITSAPP_LOG("FORCEOFF control! Number (%d), AT time:[%.2f] ", Total, GetSeconds());
    }
    else if (Omit == phase_control)
    {
        if (!snmp_parse_oid(MIB_PHASE_OMIT, anOID, &anOID_len))
        {
            ITSAPP_WRN("Error on parse oid: %s", MIB_PHASE_OMIT);
            snmp_perror(MIB_PHASE_OMIT);
            failures++;
        }
        ITSAPP_LOG("OMIT control! Number (%d), AT time:[%.2f] ", Total, GetSeconds());
    }
    else if (VehCall == phase_control)
    {
        if (!snmp_parse_oid(MIB_PHASE_VEH_CALL, anOID, &anOID_len))
        {
            ITSAPP_WRN("Error on parse oid: %s", MIB_PHASE_VEH_CALL);
            snmp_perror(MIB_PHASE_VEH_CALL);
            failures++;
        }
        ITSAPP_LOG("VEH CALL to ASC controller! Number (%d), AT time:%ld  ", Total, time(NULL));
    }

    // snmp_add_var() return 0 if success
    if (snmp_add_var(pdu, anOID, anOID_len, 'i', tmp_int))
    {
        switch (phase_control)
        {
            case ForceOff:
                snmp_perror(MIB_PHASE_FORCEOFF);
                break;
            case Omit:
                snmp_perror(MIB_PHASE_OMIT);
                break;
            case VehCall:
                snmp_perror(MIB_PHASE_VEH_CALL);
                break;
            case Hold:
                snmp_perror(MIB_PHASE_HOLD);
                break;
        }
        failures++;
    }

    if (failures)
    {
        ITSAPP_WRN("SNMP error");
        snmp_close(ss);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        //------SUCCESS: Print the result variables
        // int i =0;
        //        for(vars = response->variables; vars; vars = vars->next_variable)
        //            print_variable(vars->name, vars->name_length, vars);

        /* manipuate the information ourselves */
        /*for(vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char *sp = (char *)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int *aa;
                aa =(int *)vars->val.integer;
                printf("value #%d is NOT a string! Ack!", count++);
            }
        }*/
    }
    else
    {
        // FAILURE: print what went wrong!
        if (status == STAT_SUCCESS)
        {
            ITSAPP_WRN("Error in packet. Reason: %s", snmp_errstring(response->errstat));
        }
        else if (status == STAT_TIMEOUT)
        {
            ITSAPP_WRN("Timeout: No response from %s.", session.peername);
        }
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }
    //------Clean up:1) free the response. 2) close the session.
    if (response)
        snmp_free_pdu(response);
    snmp_close(ss);

    SOCK_CLEANUP;
}

void MmitssMib::DetReadVolume(const std::vector<int>& detNumber, int totNum)
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;

    init_snmp("ASC"); // Initialize the SNMP library

    snmp_sess_init(&session); // Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    // session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
    session.version =
        SNMP_VERSION_1; // for ASC/3 Software in VISSIM  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */

    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("ASC", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    char ctemp[50];

    // Reading all the available Detectors
    for (int i = 0; i < totNum; i++)
    {
        if (detNumber[i] > 0) // It is added to get only real detectors' values
        {
            sprintf(ctemp, "%s%d", SYS_DET_VOL, detNumber[i]);
            if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last
                                                           // bit as enabled or not: "1"  enable;
                                                           // "0" not used
            {
                ITSAPP_WRN("SNMP parse oid error");
                snmp_perror(ctemp);
                SOCK_CLEANUP;
                // exit(1);
                return;
            }
            snmp_add_null_var(pdu, anOID, anOID_len);
        }
    }
    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */
    // cout<<"I am here "<<response->errstat<<" "<<SNMP_ERR_NOERROR<<endl;
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        // int *out = new int[MAX_ITEMS];
        int i = 0;
        // for(vars = response->variables; vars; vars = vars->next_variable)
        //    print_variable(vars->name, vars->name_length, vars);

        /* manipulate the information ourselves */
        for (vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                m_out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }

        // cout<<"The VOLUME output of the detector information is: "<<out[0]<<" "<<out[1]<<"
        // "<<out[2]<<" "<<out[3]<<" "<<out[4]<<" "<<out[5]<<" "<<out[6]<<" "<<out[7]<<"
        // "<<out[8]<<" "<<out[9]<<" "<<out[10]<<" "<<out[11]<<endl;

        // logfile<<out[0]<<" "<<out[1]<<" "<<out[2]<<" "<<out[3]<<" "<<out[4]<<" "<<out[5]<<"
        // "<<out[6]<<" "<<out[7]<<" "<<out[8]<<" "<<out[9]<<endl;
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    /*
     * Clean up:    *  1) free the response.   *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);

    snmp_close(ss);

    SOCK_CLEANUP;
}

void MmitssMib::DetReadOccupancy(std::vector<int>& occupancy)
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;

    oid anOID[MAX_OID_LEN];
    size_t anOID_len;

    netsnmp_variable_list* vars;
    int status;
    //    int count=1;

    init_snmp("ASC"); // Initialize the SNMP library

    snmp_sess_init(&session); // Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    // session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
    session.version =
        SNMP_VERSION_1; // for ASC/3 Software in VISSIM  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);

    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */
    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("ASC", &session);
        SOCK_CLEANUP;
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    char ctemp[50];

    // we have 8 detectors numbered from 1 to 8 in the VISSIM model
    for (int i = 1; i <= 8; i++)
    {
        sprintf(ctemp, "%s%d", SYS_DET_OCC, i);

        if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit
                                                       // as enabled or not: "1"  enable; "0" not
                                                       // used
        {
            ITSAPP_WRN("SNMP parse oid error");
            snmp_perror(ctemp);
            SOCK_CLEANUP;
            return;
        }
        snmp_add_null_var(pdu, anOID, anOID_len);
    }
    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);

    /*
     * Process the response.
     */

    // cout<<"I am here "<<response->errstat<<" "<<SNMP_ERR_NOERROR<<endl;
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        int* out = new int[MAX_ITEMS];
        int i = 0;
        // for(vars = response->variables; vars; vars = vars->next_variable)
        //  print_variable(vars->name, vars->name_length, vars);

        /* manipulate the information ourselves */
        for (vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }

        // cout<<"I am here 1"<<endl;
        // cout<<"\n"<<endl;
        // cout<<"The VOLUME output of the detector information is: "<<out[0]<<" "<<out[1]<<"
        // "<<out[2]<<" "<<out[3]<<" "<<out[4]<<" "<<out[5]<<" "<<out[6]<<" "<<out[7]<<endl;
        // logfile<<out[0]<<" "<<out[1]<<" "<<out[2]<<" "<<out[3]<<" "<<out[4]<<" "<<out[5]<<"
        // "<<out[6]<<" "<<out[7]<<endl;
        for (int j = 0; j < 8; j++)
        {
            occupancy[j] = out[j];
        }
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            ITSAPP_WRN("Session error");
            snmp_sess_perror("snmpdemoapp", ss);
        }
    }

    /*
     * Clean up:
     * 1) free the response.
     * 2) close the session.
     */
    if (response)
        snmp_free_pdu(response);
    snmp_close(ss);
    SOCK_CLEANUP;
}

void MmitssMib::PedStatusRead()
{
    netsnmp_session session, *ss;
    netsnmp_pdu* pdu;
    netsnmp_pdu* response;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len;
    netsnmp_variable_list* vars;
    int status;
    init_snmp("ASC");         // Initialize the SNMP library
    snmp_sess_init(&session); // Initialize a "session" that defines who we're going to talk to
    /* set up defaults */
    session.peername = strdup(m_controllerAddress.toString().c_str());
    // session.version = SNMP_VERSION_2c; //for ASC intersection  /* set the SNMP version number */
    session.version = SNMP_VERSION_1; // for ASC intersection  /* set the SNMP version number */
    /* set the SNMPv1 community name used for authentication */
    session.community = (u_char*)"public";
    session.community_len = strlen((const char*)session.community);
    SOCK_STARTUP;
    ss = snmp_open(&session); /* establish the session */
    if (!ss)
    {
        ITSAPP_WRN("SNMP open error");
        snmp_sess_perror("ASC", &session);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }

    /*
     * Create the PDU for the data for our request.
     *   1) We're going to GET the system.sysDescr.0 node.
     */
    pdu = snmp_pdu_create(SNMP_MSG_GET);
    anOID_len = MAX_OID_LEN;

    //---#define CUR_TIMING_PLAN     "1.3.6.1.4.1.1206.3.5.2.1.22.0"      // return the current
    // timing plan

    char ctemp[50];
    sprintf(ctemp, "%s", WALK_GROUP);              // Check Ped phase status
    if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as
                                                   // enabled or not: "1"  enable; "0" not used
    {
        ITSAPP_WRN("SNMP parse oid error");
        snmp_perror(ctemp);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }
    snmp_add_null_var(pdu, anOID, anOID_len);
    sprintf(ctemp, "%s", PED_CALL);                // Check ped call
    if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as
                                                   // enabled or not: "1"  enable; "0" not used
    {
        ITSAPP_WRN("SNMP parse oid error");
        snmp_perror(ctemp);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }
    snmp_add_null_var(pdu, anOID, anOID_len);
    sprintf(ctemp, "%s", PEDCLEAR_GROUP);          // Check clear call
    if (!snmp_parse_oid(ctemp, anOID, &anOID_len)) // Phase sequence in the controller: last bit as
                                                   // enabled or not: "1"  enable; "0" not used
    {
        ITSAPP_WRN("SNMP parse oid error");
        snmp_perror(ctemp);
        SOCK_CLEANUP;
        // exit(1);
        return;
    }
    snmp_add_null_var(pdu, anOID, anOID_len);
    /*
     * Send the Request out.
     */
    status = snmp_synch_response(ss, pdu, &response);
    /*
     * Process the response.
     */
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {
        /*
         * SUCCESS: Print the result variables
         */
        int out[MAX_RESULT_ITEMS];
        int i = 0;
        // for(vars = response->variables; vars; vars = vars->next_variable)
        //     print_variable(vars->name, vars->name_length, vars);

        /* manipuate the information ourselves */
        for (vars = response->variables; vars; vars = vars->next_variable)
        {
            if (vars->type == ASN_OCTET_STR)
            {
                char* sp = (char*)malloc(1 + vars->val_len);
                memcpy(sp, vars->val.string, vars->val_len);
                sp[vars->val_len] = '\0';
                // printf("value #%d is a string: %s\n", count++, sp);
                free(sp);
            }
            else
            {
                int* aa;
                aa = (int*)vars->val.integer;
                out[i++] = *aa;
                // printf("value #%d is NOT a string! Ack!. Value = %d \n", count++,*aa);
            }
        }

        // Out[0] is ped status, Out[1] is ped call;
        m_pedInfo[0] = out[0];
        m_pedInfo[1] = out[1];
        m_pedInfo[2] = out[2];
    }
    else
    {
        if (status == STAT_SUCCESS)
            fprintf(stderr, "Error in packet\nReason: %s\n", snmp_errstring(response->errstat));
        else if (status == STAT_TIMEOUT)
            fprintf(stderr, "Timeout: No response from %s.\n", session.peername);
        else
        {
            snmp_sess_perror("SNMP session error", ss);
            ITSAPP_WRN("SNMP session error");
        }
    }

    /*
     * Clean up:    *  1) free the response.   *  2) close the session.
     */
    if (response)
        snmp_free_pdu(response);

    snmp_close(ss);

    SOCK_CLEANUP;
}

void MmitssMib::SendPedCall(int pedReqPhase)
{
    NOTUSED(pedReqPhase);

    /*if  (pedReqPhase==28)
        pedReqPhase=2;
    else if (pedReqPhase==20)
        pedReqPhase=8;
    else if (pedReqPhase==4)
        pedReqPhase=4;
    else if (pedReqPhase==12)
        pedReqPhase=6;
    else
        pedReqPhase=1;
    m_pedReqFlagForClean[pedReqPhase-1]=1;
    std::vector<byte> eventData;

    //Pack_Event_List(tmp_event_data, size);
    byte*   pByte;      // pointer used (by cast)to get at each byte
                           // of the shorts, longs, and blobs
    unsigned short   tempUShort;
    long    tempLong;
    //header 2 bytes
    eventData.emplace_back(0xFF);
    eventData.emplace_back(0xFF);
    //MSG ID: 0x03 for signal event data send to Signal Control Interface
    eventData.emplace_back(0x03);

    int numberOfPhase;
    int tempTime=0;
    int tempCmd=4; // Traffic Interface will consider 4 as PED_CALL

    // if the ped call is for phases in the first ring
    if (pedReqPhase==2 || pedReqPhase ==4)
    {
        //number of the phases in first ring
        numberOfPhase=1;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Time
        tempLong = (long)(tempTime);
        pByte = (byte* ) &tempLong;
        eventData.emplace_back((byte) *(pByte + 3));
        eventData.emplace_back((byte) *(pByte + 2));
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //phase
        tempUShort = (unsigned short) pedReqPhase ;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //number of the phases in second ring
        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R
    }

    // if the ped call is for phases in the second ring
    if (pedReqPhase==6 || pedReqPhase ==8)
    {
        //number of the phases in first ring
        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R

        numberOfPhase=1;
        //number of the phases in second ring
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Time
        tempLong = (long)(tempTime);
        pByte = (byte* ) &tempLong;
        eventData.emplace_back((byte) *(pByte + 3));
        eventData.emplace_back((byte) *(pByte + 2));
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //phase
        tempUShort = (unsigned short) pedReqPhase ;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));

        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R
    }
    eventData.emplace_back(0);
    try
    {
        m_ctrlIfSocket.sendBytes(eventData.data(), eventData.size());
        ITSAPP_LOG(" PED CALL SENT TO SignalControllerInterface FOR PHASE %d", pedReqPhase);
    }
    catch(const Poco::Exception& e)
    {
        ITSAPP_LOG("Failed to send data to SignalControllerInterface: %s", e.displayText().c_str());
    }*/
}

void MmitssMib::SendClearCommands()
{
    /*std::vector<byte> eventData;
    PackEventList(eventData);
    try
    {
        m_ctrlIfSocket.sendBytes(eventData.data(), eventData.size());
        ITSAPP_LOG(" The Event List sent to SignalControllerInterface to delete all previous
    commands, The size is %d",
                eventData.size());
    }
    catch(const Poco::Exception& e)
    {
        ITSAPP_LOG("Failed to send data to SignalControllerInterface: %s", e.displayText().c_str());
    }*/
}

void MmitssMib::SendClearPedCall(int ped_phase)
{
    NOTUSED(ped_phase);

    /*std::vector<byte> eventData;
    //Pack_Event_List(tmp_event_data, size);
    byte*   pByte;      // pointer used (by cast)to get at each byte
                           // of the shorts, longs, and blobs
    unsigned short   tempUShort;
    long    tempLong;
    //header 2 bytes
    eventData.emplace_back(0xFF);
    eventData.emplace_back(0xFF);
    //MSG ID: 0x03 for signal event data send to Signal Control Interface
    eventData.emplace_back(0x03);

    int numberOfPhase;
    int tempTime=0;
    int tempCmd=5; // Traffic Interface will consider 5 as PED_Clear

    // if the ped call is for phases in the first ring
    if (ped_phase==2 || ped_phase ==4)
    {
        //number of the phases in first ring
        numberOfPhase=1;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Time
        tempLong = (long)(tempTime);
        pByte = (byte* ) &tempLong;
        eventData.emplace_back((byte) *(pByte + 3));
        eventData.emplace_back((byte) *(pByte + 2));
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //phase
        tempUShort = (unsigned short) ped_phase ;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //number of the phases in second ring
        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R
    }

    // if the ped call is for phases in the second ring
    if (ped_phase==6 || ped_phase ==8)
    {
        //number of the phases in first ring
        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R
        numberOfPhase=1;
        //number of the phases in second ring
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Time
        tempLong = (long)(tempTime);
        pByte = (byte* ) &tempLong;
        eventData.emplace_back((byte) *(pByte + 3));
        eventData.emplace_back((byte) *(pByte + 2));
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //phase
        tempUShort = (unsigned short) ped_phase ;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //action
        tempUShort = (unsigned short)tempCmd;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));

        numberOfPhase=0;
        tempUShort = (unsigned short)numberOfPhase;
        pByte = (byte* ) &tempUShort;
        eventData.emplace_back((byte) *(pByte + 1));
        eventData.emplace_back((byte) *(pByte + 0));
        //Events in R
    }
    eventData.emplace_back(0);
    try
    {
        m_ctrlIfSocket.sendBytes(eventData.data(), eventData.size());
        ITSAPP_LOG(" PED CLEAR SENT TO SignalControllerInterface FOR PHASE %d", ped_phase);
    }
    catch(const Poco::Exception& e)
    {
        ITSAPP_LOG("Failed to send data to SignalControllerInterface: %s", e.displayText().c_str());
    }*/
}

int MmitssMib::CheckPedPhaseInfo()
{
    int i;
    int Ped_Phase_on[8] = {0};
    int Ped_Phase_call[8] = {0};
    int Ped_Clear_on[8] = {0};
    int ped_status = 0;
    if (m_pedInfo[0] != 0 || m_pedInfo[1] != 0 || m_pedInfo[2] != 0)
        ped_status = 1;
    for (i = 7; i >= 0; i--)
    {
        if (m_pedInfo[0] - pow(2.0, i * 1.0) >= 0)
        {
            Ped_Phase_on[i] = 1;
            m_pedInfo[0] -= (int)pow(2.0, i * 1.0);
        }
    }
    for (i = 7; i >= 0; i--)
    {
        if (m_pedInfo[1] - pow(2.0, i * 1.0) >= 0)
        {
            Ped_Phase_call[i] = 1;
            m_pedInfo[1] -= (int)pow(2.0, i * 1.0);
        }
    }
    for (i = 7; i >= 0; i--)
    {
        if (m_pedInfo[2] - pow(2.0, i * 1.0) >= 0)
        {
            Ped_Clear_on[i] = 1;
            m_pedInfo[2] -= (int)pow(2.0, i * 1.0);
        }
    }
    for (i = 0; i < 8; i++)
    {
        if (Ped_Phase_call[i] != 0)
            m_pedPhaseConsidered[i] = 1;
        else if (Ped_Phase_on[i] != 0)
            m_pedPhaseConsidered[i] = 2;
        else if (Ped_Clear_on[i] != 0)
            m_pedPhaseConsidered[i] = 3;
        else
            m_pedPhaseConsidered[i] = 0;
    }
    for (int kk = 0; kk < 8; kk++)
    {
        if (m_pedPhaseConsidered[kk] > 0)
            ITSAPP_LOG("PED state is %d and the phase is %d", m_pedPhaseConsidered[kk], kk + 1);
    }

    // check if we should clear the previous ped call from Savari ped app
    /*for (i=0;i<8;i++)
    {
        if (Ped_Phase_on[i]==1 && m_pedReqFlagForClean[i]==1) // means if the ped status is on ped
    walking time and a ped resuest from ped app is being receiveid, then send a clear command t0o
    controller
        {
            SendClearPedCall(i+1);
            m_pedReqFlagForClean[i]=0;
        }
    }*/

    return ped_status;
}

} // namespace Mmitss
} // namespace WaveApp
