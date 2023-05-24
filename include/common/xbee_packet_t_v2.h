#include <stdint.h>

#ifndef __XBEE_PACKET_V2__
#define __XBEE_PACKET_V2__

// XBee Packet (simple version)
struct __attribute__((packed)) xbee_packet_t {
  float x;               ///< x-position in meters
  float y;               ///< y-position in meters
  float z;               ///< z-position in meters
  float yaw;             ///< yaw angle in radians
};

// 2 for START & STOP bytes
// 3 for START, STOP, and 8-bit CRC
// 4 for START, STOP, and 16-bit checksum (Flecther 16)
#define NUM_FRAMING_BYTES 4

// Actual Packet Being Sent
#define _MOCAP_DATA_LENGTH sizeof(xbee_packet_t)

// Num bytes in optitrack message (INCLUDING START, STOP and optional CRC)
#define _MOCAP_PACKET_LENGTH _MOCAP_DATA_LENGTH + NUM_FRAMING_BYTES

#endif  //__XBEE_PACKET_V2__