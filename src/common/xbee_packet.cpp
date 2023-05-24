#include "xbee_packet.hpp"
#include <vector>
#include <stdio.h>
#include <unistd.h>   // read / write / sleep
#include <string.h>   //memcpy

void printXBeeMsg(xbee_packet_t &msg) {
  printf("\r");

  // Time
  if (msg.time < 1000000)
    printf("   %u |", msg.time);
  else if (msg.time < 10000000)
    printf("  %u |", msg.time);
  else if (msg.time < 100000000)
    printf(" %u |", msg.time);
  else
    printf("%u |", msg.time);

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

  // Quaternion
  if (msg.qx < 0)
    printf("%7.6f |", msg.qx);
  else
    printf(" %7.6f |", msg.qx);
  if (msg.qy < 0)
    printf("%7.6f |", msg.qy);
  else
    printf(" %7.6f |", msg.qy);
  if (msg.qz < 0)
    printf("%7.6f |", msg.qz);
  else
    printf(" %7.6f |", msg.qz);
  if (msg.qw < 0)
    printf("%7.6f |", msg.qw);
  else
    printf(" %7.6f |", msg.qw);

  // Tracking Valid
  printf("   %d   |", msg.trackingValid);

  // State
  printf("   %d   |", msg.state);

  // // Desired Position
  // if (msg.x_d < 0)
  //   printf("%7.6f |", msg.x_d);
  // else
  //   printf(" %7.6f |", msg.x_d);
  // if (msg.y_d < 0)
  //   printf("%7.6f |", msg.y_d);
  // else
  //   printf(" %7.6f |", msg.y_d);
  // if (msg.z_d < 0)
  //   printf("%7.6f |", msg.z_d);
  // else
  //   printf(" %7.6f |", msg.z_d);

  fflush(stdout);
}

void printXBeeMsg(std::vector<xbee_packet_t> &msg) {
  printf("\r");

  // Time
  printf("%011u|", msg[1].time);

  // State
  printf("   %d   |", msg[1].state);

  // Valid Tracking
  for (unsigned i = 1; i < msg.size(); i++) {
    if (msg[i].trackingValid) {
      printf("       Y       |");
    } else {
      printf("       N       |");
    }
  }

  // Position
  printf("%+07.3f|", msg[1].x);
  printf("%+07.3f|", msg[1].y);
  printf("%+07.3f|", msg[1].z);

  // Quaternion
  printf("%+07.4f|", msg[1].qw);
  printf("%+07.4f|", msg[1].qx);
  printf("%+07.4f|", msg[1].qy);
  printf("%+07.4f|", msg[1].qz);


  // // Desired Position (print last 2 quads)
  // int index = msg.size() - 1;
  // if (index > 1) {
  //   printf("%7.3f  |", msg[index - 1].x_d);
  //   printf("%7.3f  |", msg[index - 1].y_d);
  //   printf("%7.3f  |", msg[index - 1].z_d);
  // }
  // printf("%7.3f  |", msg[index].x_d);
  // printf("%7.3f  |", msg[index].y_d);
  // printf("%7.3f  |", msg[index].z_d);

  

  fflush(stdout);
}

void printXBeeMsg_simple(std::vector<xbee_packet_t> &msg) {
  printf("\r");

  // Time
  if (msg[1].time < 1000000)
    printf("   %u |", msg[1].time);
  else if (msg[1].time < 10000000)
    printf("  %u |", msg[1].time);
  else if (msg[1].time < 100000000)
    printf(" %u |", msg[1].time);
  else
    printf("%u |", msg[1].time);

  // Valid Tracking
  for (unsigned i = 1; i < msg.size(); i++) {
    if (msg[i].trackingValid) {
      printf("  Y |");
    } else {
      printf("  N |");
    }
  }

  fflush(stdout);
}


int set_xbee_dest_addr(int XBEE_portID, int xbeeAddr)
{
  //Useful strings and buffers
  char config_mode[] = "+++";
  char dest_addr_read[] = "ATDL\r\n";
  char cmd_null[] = "ATCN\r\n";
  char xbee_resp[] = "OK";

  char dest_addr_cmd[80];
  char resp[10];
  char dest_addr[10];

  size_t numBytesRead;


  printf("Programming XBee...\n");
  
  sprintf(dest_addr_cmd, "ATDL %d\r\n", xbeeAddr);
  if(!write(XBEE_portID,config_mode,sizeof(config_mode)-1))
  {
    printf("Could not write to XBee\n");
    return -1;
  }
  usleep(1E5);
  numBytesRead = read(XBEE_portID,resp,sizeof(resp));
  if(numBytesRead == 0 || !strcmp(resp,xbee_resp))
  {
    printf("Received Incorrect Response\n");
    return -2;
  }

  if(write(XBEE_portID,dest_addr_cmd,sizeof(dest_addr_cmd)-1))
  {
    printf("Could not write to XBee\n");
    return -1;
  }
  usleep(1E5);
  numBytesRead = read(XBEE_portID,resp,sizeof(resp));
  if(numBytesRead == 0 || !strcmp(resp,xbee_resp))
  {
    printf("Received Incorrect Response\n");
    return -2;
  }

  if(write(XBEE_portID,dest_addr_read,sizeof(dest_addr_read)-1))
  {
    printf("Could not write to XBee\n");
    return -1;
  }
  usleep(1E5);
  numBytesRead = read(XBEE_portID,resp,sizeof(resp));
  
  sprintf(dest_addr,"%d",xbeeAddr);
  if(numBytesRead == 0 || !strcmp(resp,dest_addr))
  {
    printf("Received Incorrect Response\n");  
    return -2;
  }
    
  if(write(XBEE_portID,cmd_null,sizeof(cmd_null)-1))
  {
    printf("Could not write to XBee\n");
    return -1;
  }
  usleep(1E5);
  numBytesRead = read(XBEE_portID,resp,sizeof(resp));
  if(numBytesRead == 0 || !strcmp(resp,xbee_resp))
  {
    printf("Recieved Incorrect Response\n");
    return -2;
  }
  printf("RESP:%s\n",resp);

  return 0;
}