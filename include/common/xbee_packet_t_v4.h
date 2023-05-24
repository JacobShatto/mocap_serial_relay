#include <stdint.h>

#ifndef __XBEE_PACKET_V4__
#define __XBEE_PACKET_V4__

// v4 XBee Packet (simple version)
struct __attribute__((packed)) xbee_packet_t {
  uint32_t time;         ///< Unique id for the rigid body being described
  float x;               ///< x-position in the Optitrack frame
  float y;               ///< y-position in the Optitrack frame
  float z;               ///< z-position in the Optitrack frame
  float qx;              ///< qx of quaternion
  float qy;              ///< qy of quaternion
  float qz;              ///< qz of quaternion
  float qw;              ///< qw of quaternion
  int8_t trackingValid;  // (bool) of whether or not tracking was valid (0 or 1)
};

#define NUM_FRAMING_BYTES 2                       // START & STOP (CRC if 3)
#define _MOCAP_DATA_LENGTH sizeof(xbee_packet_t)  // Actual Packet Being Sent
#define _MOCAP_PACKET_LENGTH \
  _MOCAP_DATA_LENGTH +       \
      NUM_FRAMING_BYTES  // Number of bytes in an optitrack message (INCLUDING
                         // START, STOP and CRC)

#endif  //__XBEE_PACKET_V4__