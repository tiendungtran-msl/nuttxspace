/****************************************************************************
 * apps/uav/lib/utils/debug.hpp
 *
 * Debug macros cho driver và modules
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>
#include <debug.h>
#include <syslog.h>
#include <stdio.h>

/* Color codes cho serial output (ANSI escape) */
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_BLUE    "\x1b[34m"
#define ANSI_RESET   "\x1b[0m"

/* Debug macros - chỉ hoạt động khi CONFIG_DEBUG_FEATURES=y */
#ifdef CONFIG_DEBUG_FEATURES

  #define DRIVER_ERR(fmt, ...)   syslog(LOG_ERR, ANSI_RED "[ERROR] " fmt ANSI_RESET "\n", ##__VA_ARGS__)
  #define DRIVER_WARN(fmt, ...)  syslog(LOG_WARNING, ANSI_YELLOW "[WARN] " fmt ANSI_RESET "\n", ##__VA_ARGS__)
  #define DRIVER_INFO(fmt, ...)  syslog(LOG_INFO, ANSI_GREEN "[INFO] " fmt ANSI_RESET "\n", ##__VA_ARGS__)

  #ifdef CONFIG_DEBUG_VERBOSE
    #define DRIVER_DEBUG(fmt, ...) syslog(LOG_DEBUG, ANSI_BLUE "[DEBUG] " fmt ANSI_RESET "\n", ##__VA_ARGS__)
  #else
    #define DRIVER_DEBUG(fmt, ...) ((void)0)
  #endif

#else
  /* Khi không có DEBUG, các macro này không làm gì */
  #define DRIVER_ERR(fmt, ...)   ((void)0)
  #define DRIVER_WARN(fmt, ...)  ((void)0)
  #define DRIVER_INFO(fmt, ...)  ((void)0)
  #define DRIVER_DEBUG(fmt, ...) ((void)0)
#endif
