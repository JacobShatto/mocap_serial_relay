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

#include <xbee_packet_5q.hpp>

using std::cout;
using std::endl;

/**
 * optitrack_message_t defines the data for a rigid body transmitted by the
 * Optitrack server. A rigid body has a unique id, a 3D position, and a
 * quaternion defining the orientation.
 */

void getData(xbee_packet_t& msg);
int XBEE_getData(xbee_packet_t& msg);

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
  printf("    X     |");
  printf("    Y     |");
  printf("    Z     |");
  printf("   yaw    |");
  printf("\n");

  xbee_packet_t theMsg;
  theMsg.x = 0;
  theMsg.y = 0;
  theMsg.z = 0;
  theMsg.yaw = 0;

  while (1) {
    // 1) Get Data
    XBEE_getData(theMsg);

    // 2) Print to terminal
    printf("\r");

    // XYZ
    printf("%7.6f |", theMsg.x);
    printf("%7.6f |", theMsg.y);
    printf("%7.6f |", theMsg.z);
    printf("%7.6f |", theMsg.yaw);

    fflush(stdout);
  }

  return 0;
}

// void getData(xbee_packet_t& msg) {
//   while (read(fd, ptr, 1) > 0) {
//     // if the first Byte is wrong keep looking
//     if ((ptr == serialPacket) && (ptr[0] != XBEE_START_BYTE1)) {
//       // printf("Looking for First Byte!\n");
//       continue;
//     }

//     ptr++;

//     // Once we have all of the Bytes check to make sure first and last are good
//     if ((ptr - serialPacket) == _MOCAP_PACKET_LENGTH) {
//       // printf("Full Packet!\n");

//       // Error Check Packet 0xFF
//       if ((serialPacket[0] != XBEE_START_BYTE1) ||
//           (serialPacket[1] != XBEE_START_BYTE2)) {
//         printf("Packet Error!\n");
//         printf("startByte2 = %u\n",
//                (unsigned char)serialPacket[1]);
//         // printf("Packet = ");
//         for (unsigned i = 0; i < _MOCAP_PACKET_LENGTH; i++) {
//           printf("%2X, ", (unsigned char)serialPacket[i]);
//         }
//         printf("\n");
//         // Don't take packet, there's an error
//       } else {
//         // Alternatively to make it faster...

//         // full packet, parse it
//         ptr = serialPacket + NUM_START_BYTES;
//         memcpy(&msg, ptr, _MOCAP_DATA_LENGTH);

//         // Print to terminal
//         printf("\r");

//         // XYZ
//         printf("%7.6f |", msg.x);
//         printf("%7.6f |", msg.y);
//         printf("%7.6f |", msg.z);
//         printf("%7.6f |", msg.yaw);

//         fflush(stdout);
//       }

//       ptr = serialPacket;
//     }
//   }
// }



int XBEE_getData(xbee_packet_t& msg)
{
    // Init Packet Pointers
    static unsigned char serialPacket[_MOCAP_PACKET_LENGTH];
    static unsigned char* dataPacket = serialPacket + NUM_START_BYTES;
    static unsigned char* ptr = serialPacket;
    static uint8_t chksm0 = 0;
    static uint8_t chksm1 = 0;

     while (read(fd, ptr, 1) > 0) {
        // 1) if the first Byte is wrong keep looking
        if( (ptr == serialPacket) && (ptr[0] != XBEE_START_BYTE1) ) {
            ptr = serialPacket;
            // printf("First XBee Byte NOT correct! %02x\n",ptr[0] );
            continue;
        }

        // 2) if the second Byte is wrong keep looking
        if( (ptr == serialPacket + 1) && (ptr[0] != XBEE_START_BYTE2) ) {
            ptr = serialPacket;
            // printf("Second XBee Byte NOT correct!\n");
            continue;
        }

        // 3) Reset the checksum on the third byte
        if(ptr == serialPacket + 2) {
            chksm0 = 0;
            chksm1 = 0;
        } 

        // 4) Compute the checksum on the data bytes
        if ((ptr >= serialPacket + 2) && (ptr < serialPacket + _MOCAP_DATA_LENGTH + NUM_START_BYTES) ) {
            chksm0 += ptr[0];
            chksm1 += chksm0;
            // printf("chksm0: %02x, chksm1: %02x \n", chksm0, chksm1);
        }

        // 5) And then increment the ptr to keep reading in bytes
        ptr++;	

        // 6) Once we have all of the Bytes, Process them!
        if( (ptr-serialPacket) == _MOCAP_PACKET_LENGTH) {
            ptr = serialPacket;

            uint16_t checksum_calculated = (uint16_t) ((chksm1 << 8) | chksm0);
            uint16_t checksum_received = 0;
            // uint16_t checksum_with_function = fletcher16(dataPacket, OPTI_DATA_LENGTH);

            memcpy(&checksum_received, &dataPacket[_MOCAP_DATA_LENGTH], sizeof(checksum_received));
            // printf("Checksum calculated: %u; Checksum Recieved: %u \n", checksum_calculated, checksum_received);
            
            if (checksum_received == checksum_calculated)
            {
                memcpy(&msg, dataPacket, _MOCAP_DATA_LENGTH);

            } else 
            {
                return -1;
            }
        }	
    }
    return 0;
}