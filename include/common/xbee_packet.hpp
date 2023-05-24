#ifndef __XBEE_PACKET__
#define __XBEE_PACKET__
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                           SELECT PACKET HERE                             //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#include <xbee_packet_t_pg.h>
//#include "xbee_packet_t_v4.h"

#include <vector>

#define _XBEE_USE_TWO_START_BYTES_

#define NUM_START_BYTES 2
#define XBEE_START_BYTE1 0x81
#define XBEE_START_BYTE2 0xA1

void printXBeeMsg(xbee_packet_t &msg);

void printXBeeMsg(std::vector<xbee_packet_t> &msg);

void printXBeeMsg_simple(std::vector<xbee_packet_t> &msg);

int set_xbee_dest_addr(int XBEE_portID, int xbeeAddr);


#endif