#ifndef __XBEE_PACKET__5Q__
#define __XBEE_PACKET__5Q__
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                           SELECT PACKET HERE                             //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#include <xbee_packet_t_v2.h>
//#include "xbee_packet_t_v4.h"

#include <vector>

#define _XBEE_USE_TWO_START_BYTES_

#define NUM_START_BYTES 2
#define XBEE_START_BYTE1 0x81
#define XBEE_START_BYTE2 0xA1

void printXBeeMsg_5q(xbee_packet_t &msg);

void printXBeeMsg_5q(std::vector<xbee_packet_t> &msg);



#endif // __XBEE_PACKET__5Q__