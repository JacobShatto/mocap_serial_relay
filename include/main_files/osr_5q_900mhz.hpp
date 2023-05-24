#ifndef __OSR_5Q_900MHZ__
#define __OSR_5Q_900MHZ__

#include <getopt.h>
#include <optitrack_channels.h>
#include <serial.h>
#include <timestamp.h>
#include <optitrack.hpp>

#include <vector>

#include <errno.h>  //Errors for read/write
#include <stdio.h>
#include <unistd.h>  // read / write / sleep

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <cmath>
#include <fstream>  // std::ifstream
#include <iostream>

// Below for PRId64
#include <inttypes.h>
#include <cinttypes>

#include <stdint.h>

/// Helper Functions
#include <Quaternion.h>
#include <crc16.h>
#include <nonBlockingCLI.h>
#include <simpleMatrixStuff.h>
#include <xbee_packet_5q.hpp>



void send_serial_xbees(std::vector<int>  fd, std::vector<char* > serialPacket, int packetLength);
void printCLIforStatesFakeData();
void initTerminalPrinting();
void printToTerminal(std::vector<xbee_packet_t> &msg);
FILE* initLoggingXBeeGCS(int numQuads);
void logXBeeGCS(FILE *fp, std::vector<xbee_packet_t> xb_msg, int numQuads);


#endif  //__OSR_5Q_900MHZ__