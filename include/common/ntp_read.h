/**
 * <ntp_read.h>
 *
 * @brief       Code for reading the NTP timer offset in order to beaglebones.
 *              Only used if "log_ntp" flag is 'true' in settings file and
 *              if NTP server is running on HP laptop and
 *              BeagleBoneBlue is running NTP client and both laptop and BeagleBoneBlue are connected to A2sys router
 *
 * 
 * @author     Prince Kuevor 
 * @date       05/21/2020 (MM/DD/YYYY)
 * 
 * 
 * @addtogroup  NTP_READ
 * @{
 */

#ifndef __NTP_READ__
#define __NTP_READ__

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTP_COUNTER_THRESH 20*1e6 //Number of microseconds between each update of the NTP offset 
#define NTP_PATH_SIZE 1024
#define NTP_ERR_NUM 9999.9999

/**
 * @brief   Offset read from NTP daemon
 */
extern double ntp_offset;

double get_ntp_offset();

#ifdef __cplusplus
}
#endif

#endif //__NTP_READ__