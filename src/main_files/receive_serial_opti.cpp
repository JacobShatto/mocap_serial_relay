#include <serial.h>
#include <unistd.h>  // read / write
#include <iostream>

#include <stdint.h>
#include <cstdlib>  //one of these two is for memcpy
#include <cstring>
//#include <inttypes.h>
#include <string>  //stoi

// Below for PRId64
#include <cinttypes>

#include <xbee_packet.hpp>

using std::cout;
using std::endl;

/**
 * optitrack_message_t defines the data for a rigid body transmitted by the
 * Optitrack server. A rigid body has a unique id, a 3D position, and a
 * quaternion defining the orientation.
 */

void getData(xbee_packet_t& msg);

unsigned char c = 'D';
xbee_packet_t msg;

// Serial Packet Variables
char serialPacket[_MOCAP_PACKET_LENGTH];
char* ptr = serialPacket;
int fd;
char startByte1 = XBEE_START_BYTE1;
char startByte2 = XBEE_START_BYTE2;

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "Usage: " << endl;
    cout << " ./receiveSerial <Serial Port> " << endl << endl;
    cout << " ./receiveSerial <Serial Port> <Baudrate> " << endl << endl;
    cout << " <Serial Port> = /dev/ttyUSB0, etc..." << endl;
    cout << " <Baudrate> = 9600, 19200, etc ..." << endl;

    return 1;
  }

  int baudRate = 115200;

  if (argc == 3) {
    baudRate = std::stoi(argv[2]);
  }

  fd = serial_open(argv[1], baudRate, 0);  // blocking == 0 now,

  if (fd == -1) {
    cout << "Failed to open Serial Port" << endl;
    return 1;
  }

  // Begin Printing
  printf("\n");

  printf("   Time   |");
  printf("    X     |");
  printf("    Y     |");
  printf("    Z     |");
  printf("    qx    |");
  printf("    qy    |");
  printf("    qz    |");
  printf("    qw    |");
  printf(" valid |");
  printf(" state |");
  // printf("   X_d    |");
  // printf("   Y_d    |");
  // printf("   Z_d    |");

  printf("\n");

  xbee_packet_t theMsg;
  while (1) {
    getData(theMsg);
  }

  return 0;
}

void getData(xbee_packet_t& msg) {
  while (read(fd, ptr, 1) > 0) {
    // if the first Byte is wrong keep looking
    if ((ptr == serialPacket) && (ptr[0] != startByte1)) {
      // printf("Looking for First Byte!\n");
      continue;
    }

    ptr++;

    // Once we have all of the Bytes check to make sure first and last are good
    if ((ptr - serialPacket) == _MOCAP_PACKET_LENGTH) {
      // printf("Full Packet!\n");

      // Error Check Packet 0xFF
      if ((serialPacket[0] != startByte1) ||
          (serialPacket[1] != startByte2)) {
        printf("Packet Error!\n");
        printf("startByte2 = %u\n",
               (unsigned char)serialPacket[1]);
        // printf("Packet = ");
        for (unsigned i = 0; i < _MOCAP_PACKET_LENGTH; i++) {
          printf("%2X, ", (unsigned char)serialPacket[i]);
        }
        printf("\n");
        // Don't take packet, there's an error
      } else {
        // Alternatively to make it faster...
        // ptr = serialPacket + 1;
        // memcpy(&msg, ptr, sizeof(msg));

        // full packet, parse it
        ptr = serialPacket + 1;
        memcpy(&msg, ptr, _MOCAP_PACKET_LENGTH - NUM_FRAMING_BYTES);

        // Print to terminal
        printf("\r");

        // Time
        // printf("  %d |",msg.time);
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

      ptr = serialPacket;
    }
  }
}
