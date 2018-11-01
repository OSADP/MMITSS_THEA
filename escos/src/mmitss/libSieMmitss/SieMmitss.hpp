/*
 * SIEMENS Copyright 2017 header
 */

/*
 * SieMmitss.hpp
 *
 *  Created on: May 15, 2017
 *      Author: M. Venus
 */

#pragma once

#include "syslog.h"

/* priority codes */
#define LOG_EMERG 0   /* system is unusable */
#define LOG_ALERT 1   /* action must be taken immediately */
#define LOG_CRIT 2    /* critical conditions */
#define LOG_ERR 3     /* error conditions */
#define LOG_WARNING 4 /* warning conditions */
#define LOG_NOTICE 5  /* normal but significant condition */
#define LOG_INFO 6    /* informational */
#define LOG_DEBUG 7   /* debug-level messages */

#define MAX_UNACK_REQ 5

#define MMITSS_LOG(fmt, args...) \
    do \
    { \
        ::syslog(SieMmitss::getAppLogId() << 3 | LOG_INFO, "%s : " fmt "\n", __func__, ##args); \
    } while (false);
#define MMITSS_WRN(fmt, args...) \
    do \
    { \
        ::syslog(SieMmitss::getAppLogId() << 3 | LOG_WARNING, "%s : " fmt "\n", __func__, ##args); \
    } while (false);
#define MMITSS_ERROR(fmt, args...) \
    do \
    { \
        ::syslog(SieMmitss::getAppLogId() << 3 | LOG_ERR, "%s : " fmt "\n", __func__, ##args); \
    } while (false);

#define NOTUSED(x) (void)(x)

namespace SieMmitss
{
int getAppLogId();
}
