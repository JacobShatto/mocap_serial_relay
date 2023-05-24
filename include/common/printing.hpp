#ifndef PRINTING_HPP
#define PRINTING_HPP

#include <serial.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <xbee_packet.hpp>

void initLogging(FILE *fp);

void initLoggingSimpleOptirelay(FILE *fp, int numQuads);

FILE *initLoggingXBeeGCS(int numQuads, bool usingWindVane);

// Print CLI for States
void printCLIforStates();

void printCLIHeaders_simple(int numQuads);

// Printf Headers
void printCLIHeaders(int numQuads);

void logXBeeMsgs(FILE *fp, std::vector<xbee_packet_t> xb_msg, int numQuads,
                 std::vector<std::vector<float>> w, int fd_w,
                 bool usingWindVane, char &wind);

void log_xbee_msg_quali(FILE *fp, xbee_packet_t &xb_msg, 
                        uint32_t time32_us, uint64_t time64_us, double ntp_offset, 
                        float roll, float pitch, float yaw);

#endif  // PRINTING_HPP