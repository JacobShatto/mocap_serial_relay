#include "xbee_packet_5q.hpp"
#include <vector>
#include <stdio.h>
#include <unistd.h>   // read / write / sleep
#include <string.h>   //memcpy

void printXBeeMsg_5q(xbee_packet_t &msg) {
  printf("\r");


  // XYZ
  if (msg.x < 0)
    printf("%7.6f |", msg.x);
  else
    printf(" %7.6f |", msg.x);
  if (msg.y < 0)
    printf("%7.6f |", msg.y);
  else
    printf(" %7.6f |", msg.y);
  if (msg.z < 0)
    printf("%7.6f |", msg.z);
  else
    printf(" %7.6f |", msg.z);

  fflush(stdout);
}

void printXBeeMsg_5q(std::vector<xbee_packet_t> &msg) {
  printf("\r");

  // Position
  printf("%+07.3f|", msg[1].x);
  printf("%+07.3f|", msg[1].y);
  printf("%+07.3f|", msg[1].z);

  fflush(stdout);
}