#include <stdint.h>

#ifndef __XBEE_PACKET_PG__
#define __XBEE_PACKET_PG__

// PG (Prince-Glen) XBee Packet (simple version)
struct __attribute__((packed)) xbee_packet_t {
  uint32_t time;         ///< Unique id for the rigid body being described
  float x;               ///< x-position in the Optitrack frame
  float y;               ///< y-position in the Optitrack frame
  float z;               ///< z-position in the Optitrack frame
  float qx;              ///< qx of quaternion
  float qy;              ///< qy of quaternion
  float qz;              ///< qz of quaternion
  float qw;              ///< qw of quaternion
  int8_t trackingValid;  ///< boolean - TRUE if tracking is valid
  int16_t state;         ///< State transitions from keyboard input
  uint8_t deadByte;      ///< Dead byte to ensure backwards compatibility. 
};

// 2 for START & STOP bytes
// 3 for START, STOP, and 8-bit CRC
// 4 for START, STOP, and 16-bit checksum (Flecther 16)
#define NUM_FRAMING_BYTES 4

// Actual Packet Being Sent
#define _MOCAP_DATA_LENGTH sizeof(xbee_packet_t)

// Num bytes in optitrack message (INCLUDING START, STOP and optional CRC)
#define _MOCAP_PACKET_LENGTH _MOCAP_DATA_LENGTH + NUM_FRAMING_BYTES

#endif  //__XBEE_PACKET_PG__