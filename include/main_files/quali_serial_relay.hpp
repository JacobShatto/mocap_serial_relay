#include <getopt.h>
#include <timestamp.h>
#include <serial.h>

// Qualisys
#include <RTProtocol.h>
#define QTM_RT_SERVER_BASE_PORT 22222

#include <vector>
#include <string>
#include <unordered_map>

#include <cmath>
#include <cfloat>

#include <errno.h>	//Errors for read/write
#include <unistd.h> // read / write / sleep
#include <stdio.h>

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <fstream>      // std::ifstream
#include <cmath>

#include <crc16.h>      //fletcher16

#include <nonBlockingCLI.h>

// Below for PRId64
#include <cinttypes>
#include <inttypes.h>

#include <stdint.h>

#include "xbee_packet.hpp"
#include "Quaternion.h"

typedef enum Euler_Rotation_Sequence
{
    //Negative values are valid Euler rotations, but not all axes are different
    yaw_roll_yaw     = -1,
    roll_pitch_roll  = -2,
    pitch_yaw_pitch  = -3,
    yaw_pitch_yaw    = -4,
    roll_yaw_roll    = -5,
    pitch_roll_pitch = -6,

    //Positive values are "Proper/Classic" Euler angles (aka Tait-Bryan Angles)
    roll_pitch_yaw = 1,
    pitch_yaw_roll = 2,
    yaw_roll_pitch = 3,
    roll_yaw_pitch = 4,
    yaw_pitch_roll = 5,
    pitch_roll_yaw = 6,

    //Zero is reserved for invalid sequences
    invalid_rotation_sequence = 0
} Euler_Rotation_Sequence;

 
 // Get settings from QTM
void getQTMSettings(CRTProtocol &poRTProtocol) {
    if (poRTProtocol.Read6DOFSettings())
    {
        int nBodies = poRTProtocol.Get6DOFBodyCount();

        printf("There %s %d 6DOF %s\n\n", nBodies == 1 ? "is" : "are", nBodies, nBodies == 1 ? "body" : "bodies");

        CRTProtocol::SPoint sPoint;

        for (int iBody = 0; iBody < nBodies; iBody++)
        {
            printf("Body #%d\n", iBody);
            printf("  Name:  %s\n",   poRTProtocol.Get6DOFBodyName(iBody));
            printf("  Color: %.6X\n", poRTProtocol.Get6DOFBodyColor(iBody));
            printf("  Euler: %s -> %s -> %s\n", poRTProtocol.Get6DOFBodyFirstEuler(iBody), poRTProtocol.Get6DOFBodySecondEuler(iBody), poRTProtocol.Get6DOFBodyThirdEuler(iBody));
            for (unsigned int iPoint = 0; iPoint < poRTProtocol.Get6DOFBodyPointCount(iBody); iPoint++)
            {
                poRTProtocol.Get6DOFBodyPoint(iBody, iPoint, sPoint);
                printf("  Point: X = %9f  Y = %9f  Z = %9f\n", sPoint.fX, sPoint.fY, sPoint.fZ);
            }
            printf("\n");
        }
    }
}

Euler_Rotation_Sequence select_euler_rotation_sequence(const char * first_axis, const char * second_axis, const char * third_axis);

bool arrange_euler_angles(Euler_Rotation_Sequence seq, 
                         float &fRoll, float &fPitch, float &fYaw, 
                         float fAng1, float fAng2, float fAng3);
