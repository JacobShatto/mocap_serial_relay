//================================================================================
// NOTE: This code is adapted from PacketClient.cpp, which remains in this
// folder. The license for the code is located in that file and duplicated
// below:
//
//=============================================================================
// Copyright © 2014 NaturalPoint, Inc. All Rights Reserved.
//
// This software is provided by the copyright holders and contributors "as is"
// and any express or implied warranties, including, but not limited to, the
// implied warranties of merchantability and fitness for a particular purpose
// are disclaimed. In no event shall NaturalPoint, Inc. or contributors be
// liable for any direct, indirect, incidental, special, exemplary, or
// consequential damages (including, but not limited to, procurement of
// substitute goods or services; loss of use, data, or profits; or business
// interruption) however caused and on any theory of liability, whether in
// contract, strict liability, or tort (including negligence or otherwise)
// arising in any way out of the use of this software, even if advised of the
// possibility of such damage.
//=============================================================================

#include <float.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optitrack.hpp>

// Linux socket headers
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define MAX_NAMELENGTH 256

// NATNET message ids
#define NAT_PING 0
#define NAT_PINGRESPONSE 1
#define NAT_REQUEST 2
#define NAT_RESPONSE 3
#define NAT_REQUEST_MODELDEF 4
#define NAT_MODELDEF 5
#define NAT_REQUEST_FRAMEOFDATA 6
#define NAT_FRAMEOFDATA 7
#define NAT_MESSAGESTRING 8
#define NAT_UNRECOGNIZED_REQUEST 100
#define UNDEFINED 999999.9999

#define MAX_PACKETSIZE \
  100000  // max size of packet (actual packet size is dynamic)
#define VERBOSE 0

std::string guess_optitrack_network_interface(void) {
  ifaddrs* ifAddrStruct = 0;
  ifaddrs* ifa = 0;
  void* tmpAddrPtr = 0;

  std::string interfaceAddress;
  char ipAddress[128];

  getifaddrs(&ifAddrStruct);
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
      continue;
    }
    if (ifa->ifa_addr->sa_family == AF_INET) {  // IP4
      tmpAddrPtr = &((sockaddr_in*)ifa->ifa_addr)->sin_addr;
      // char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, ipAddress, INET_ADDRSTRLEN);
      printf("%s IP Address %s\n", ifa->ifa_name, ipAddress);

      // If the name starts with a 'w', then it is probably a wireless device
      if (ifa->ifa_name && ifa->ifa_name[0] == 'w') {
        interfaceAddress = ipAddress;
        break;
        // Else if it isn't the loopback device, then it's our best guess
      } else if (ifa->ifa_name && !strncasecmp(ifa->ifa_name, "lo", 2)) {
        interfaceAddress = ipAddress;
      }
    }
  }

  freeifaddrs(ifAddrStruct);
  printf(
      "[guess_optitrack_network_interface] detected interface address as %s\n",
      interfaceAddress.c_str());
  return interfaceAddress;
}

SOCKET create_optitrack_data_socket_ipv6(const std::string& interfaceIp,
                                         unsigned short port) {
  SOCKET dataSocket = socket(AF_INET6, SOCK_DGRAM, 0);

  // Prince: This variable is currently unused and causing a compiler warning
  // However, I don't want to change the function prototype
  port = port;

  // allow multiple clients on same machine to use address/port
  int value = 1;
  int retval = setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value,
                          sizeof(value));
  if (retval < 0) {
    close(dataSocket);
    return -1;
  }

  sockaddr_in6 socketAddr;
  memset(&socketAddr, 0, sizeof(socketAddr));

  socketAddr.sin6_family = AF_INET6;
  socketAddr.sin6_port = htons(PORT_DATA);
  socketAddr.sin6_addr = in6addr_any;

  if (bind(dataSocket, (sockaddr*)&socketAddr, sizeof(sockaddr)) < 0) {
    printf("[create_optitrack_data_socket] bind failed.\n");
    return -1;
  } else {
    printf(
        "[create_optitrack_data_socket] bound to socket successfully on port "
        "%d\n",
        PORT_DATA);
  }

  /*
      // join multicast group
      in_addr interfaceAddress;
      interfaceAddress.s_addr = inet_addr(interfaceIp.c_str());

      in_addr multicastAddress;
      multicastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);

      ip_mreq Mreq;
      Mreq.imr_multiaddr = multicastAddress;
      Mreq.imr_interface = interfaceAddress;
  */

  // join multicast group
  in6_addr interfaceAddress;
  inet_pton(AF_INET6, interfaceIp.c_str(), &interfaceAddress);

  in6_addr multicastAddress;
  inet_pton(AF_INET6, MULTICAST_ADDRESS_6, &multicastAddress);

  struct ipv6_mreq mreq6;
  memcpy(&mreq6.ipv6mr_multiaddr, &multicastAddress, sizeof(struct in6_addr));

  memcpy(&mreq6.ipv6mr_multiaddr, &interfaceAddress, sizeof(struct in6_addr));

  // mreq6.ipv6mr_interface= //interfaceAddress;
  /*
          r1= setsockopt(dataSocket, IPPROTO_IPV6, IPV6_MULTICAST_LOOP,
                        &loopBack, sizeof(loopBack));
          if (r1<0)
                  perror("joinGroup:: IPV6_MULTICAST_LOOP:: ");
  */

  /*
          int r2= setsockopt(dataSocket, IPPROTO_IPV6, IPV6_MULTICAST_HOPS,
                        &mcastTTL, sizeof(mcastTTL));
          if (r2<0)
                  perror("joinGroup:: IPV6_MULTICAST_HOPS::  ");
  */

  int r3 = setsockopt(dataSocket, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq6,
                      sizeof(mreq6));
  if (r3 < 0) perror("joinGroup:: IPV6_ADD_MEMBERSHIP:: ");

  /*
      retval = setsockopt(dataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
     (char*)&Mreq, sizeof(Mreq)); if (retval < 0) {
          printf("[create_optitrack_data_socket] join failed.\n");
          return -1;
      } else {
          printf("[create_optitrack_data_socket] joined multicast group at
     address %s\n", MULTICAST_ADDRESS);
      }
  */

  // create a 1MB buffer
  int optval = 0x100000;
  socklen_t optvalSize = 4;
  setsockopt(dataSocket, SOL_SOCKET, SO_RCVBUF, (char*)&optval, optvalSize);
  getsockopt(dataSocket, SOL_SOCKET, SO_RCVBUF, (char*)&optval, &optvalSize);
  if (optval != 0x100000) {
    printf("[create_optitrack_data_socket] ReceiveBuffer size = %d\n", optval);
  } else {
    printf(
        "[create_optitrack_data_socket] Increased receive buffer size to %d\n",
        optval);
  }

  return dataSocket;
}

SOCKET create_optitrack_data_socket(const std::string& interfaceIp,
                                    unsigned short port) {
  SOCKET dataSocket = socket(AF_INET, SOCK_DGRAM, 0);

  // Prince: This variable is currently unused and causing a compiler warning
  // However, I don't want to change the function prototype
  port = port;

  // allow multiple clients on same machine to use address/port
  int value = 1;
  int retval = setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value,
                          sizeof(value));
  if (retval < 0) {
    close(dataSocket);
    return -1;
  }

  sockaddr_in socketAddr;
  memset(&socketAddr, 0, sizeof(socketAddr));
  socketAddr.sin_family = AF_INET;
  socketAddr.sin_port = htons(PORT_DATA);
  socketAddr.sin_addr.s_addr = INADDR_ANY;
  if (bind(dataSocket, (sockaddr*)&socketAddr, sizeof(sockaddr)) < 0) {
    printf("[create_optitrack_data_socket] bind failed.\n");
    return -1;
  } else {
    printf(
        "[create_optitrack_data_socket] bound to socket successfully on port "
        "%d\n",
        PORT_DATA);
  }

  // join multicast group
  in_addr interfaceAddress;
  interfaceAddress.s_addr = inet_addr(interfaceIp.c_str());

  in_addr multicastAddress;
  multicastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);

  ip_mreq Mreq;
  Mreq.imr_multiaddr = multicastAddress;
  Mreq.imr_interface = interfaceAddress;
  retval = setsockopt(dataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&Mreq,
                      sizeof(Mreq));
  if (retval < 0) {
    printf("[create_optitrack_data_socket] join failed.\n");
    return -1;
  } else {
    printf(
        "[create_optitrack_data_socket] joined multicast group at address %s\n",
        MULTICAST_ADDRESS);
  }

  // create a 1MB buffer
  int optval = 0x100000;
  socklen_t optvalSize = 4;
  setsockopt(dataSocket, SOL_SOCKET, SO_RCVBUF, (char*)&optval, optvalSize);
  getsockopt(dataSocket, SOL_SOCKET, SO_RCVBUF, (char*)&optval, &optvalSize);
  if (optval != 0x100000) {
    printf("[create_optitrack_data_socket] ReceiveBuffer size = %d\n", optval);
  } else {
    printf(
        "[create_optitrack_data_socket] Increased receive buffer size to %d\n",
        optval);
  }

  return dataSocket;
}

std::vector<optitrack_message_t> parse_optitrack_packet_into_messages(
    const char* packet, int size) {
  // Prince: This variable is currently unused and causing a compiler warning
  // However, I don't want to change the function prototype
  size = size;

  std::vector<optitrack_message_t> messages;

  int major = 3;
  int minor = 0;

  const char* ptr = packet;
  if (VERBOSE) {
    printf("Begin Packet\n-------\n");
  }

  // message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2);
  ptr += 2;
  if (VERBOSE) {
    printf("Message ID : %d\n", MessageID);
  }

  // size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2);
  ptr += 2;
  if (VERBOSE) {
    printf("Byte count : %d\n", nBytes);
  }
  if (MessageID == NAT_FRAMEOFDATA) {  // FRAME OF MOCAP DATA packet
    // frame number
    int frameNumber = 0;
    memcpy(&frameNumber, ptr, 4);
    ptr += 4;
    if (VERBOSE) {
      printf("Frame # : %d\n", frameNumber);
    }

    // number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0;
    memcpy(&nMarkerSets, ptr, 4);
    ptr += 4;
    if (VERBOSE) {
      printf("Marker Set Count : %d\n", nMarkerSets);
    }

    for (int i = 0; i < nMarkerSets; i++) {
      // Markerset name
      char szName[256];
      strcpy(szName, ptr);
      int nDataBytes = (int)strlen(szName) + 1;
      ptr += nDataBytes;
      // printf("Model Name: %s\n", szName);

      // marker data
      int nMarkers = 0;
      memcpy(&nMarkers, ptr, 4);
      ptr += 4;
      // printf("Marker Count : %d\n", nMarkers);

      for (int j = 0; j < nMarkers; j++) {
        float x = 0;
        memcpy(&x, ptr, 4);
        ptr += 4;
        float y = 0;
        memcpy(&y, ptr, 4);
        ptr += 4;
        float z = 0;
        memcpy(&z, ptr, 4);
        ptr += 4;
        // printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n",j,x,y,z);
      }
    }

    // unidentified markers
    int nOtherMarkers = 0;
    memcpy(&nOtherMarkers, ptr, 4);
    ptr += 4;
    if (VERBOSE) {
      printf("Unidentified Marker Count : %d\n", nOtherMarkers);
    }
    for (int j = 0; j < nOtherMarkers; j++) {
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;
      if (VERBOSE) {
        printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, x, y, z);
      }
    }

    // rigid bodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4);
    ptr += 4;
    if (VERBOSE) {
      printf("Rigid Body Count : %d\n", nRigidBodies);
    }
    for (int j = 0; j < nRigidBodies; j++) {
      optitrack_message_t msg;
      // rigid body position/orientation
      memcpy(&(msg.id), ptr, 4);
      ptr += 4;
      memcpy(&(msg.x), ptr, 4);
      ptr += 4;
      memcpy(&(msg.y), ptr, 4);
      ptr += 4;
      memcpy(&(msg.z), ptr, 4);
      ptr += 4;
      memcpy(&(msg.qx), ptr, 4);
      ptr += 4;
      memcpy(&(msg.qy), ptr, 4);
      ptr += 4;
      memcpy(&(msg.qz), ptr, 4);
      ptr += 4;
      memcpy(&(msg.qw), ptr, 4);
      ptr += 4;

      if (VERBOSE) {
        printf("ID : %d\n", msg.id);
        printf("pos: [%3.2f,%3.2f,%3.2f]\n", msg.x, msg.y, msg.z);
        printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", msg.qx, msg.qy, msg.qz,
               msg.qw);
      }

      // NatNet version 2.0 and later
      if (major >= 2) {
        // Mean marker error
        float fError = 0.0f;
        memcpy(&fError, ptr, 4);
        ptr += 4;
      }

      // NatNet version 2.6 and later
      if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
        // params
        short params = 0;
        memcpy(&params, ptr, 2);
        ptr += 2;
        bool bTrackingValid =
            params &
            0x01;  // 0x01 : rigid body was successfully tracked in this frame
        msg.trackingValid = bTrackingValid;
      }
      messages.push_back(msg);

    }  // Go to next rigid body

  } else {
    printf("Unrecognized Packet Type: %d\n", MessageID);
  }

  return messages;
}

void toEulerAngle(const optitrack_message_t& msg, double& roll, double& pitch,
                  double& yaw) {
  // from wikipedia quaternion entry
  double ysqr = msg.qy * msg.qy;

  // roll (x-axis rotation)
  double t0 = +2.0 * (msg.qw * msg.qx + msg.qy * msg.qz);
  double t1 = +1.0 - 2.0 * (msg.qx * msg.qx + ysqr);
  roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (msg.qw * msg.qy - msg.qz * msg.qx);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (msg.qw * msg.qz + msg.qx * msg.qy);
  double t4 = +1.0 - 2.0 * (ysqr + msg.qz * msg.qz);
  yaw = std::atan2(t3, t4);
}
