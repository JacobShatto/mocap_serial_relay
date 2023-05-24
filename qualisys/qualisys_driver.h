#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/serial.h"

// Qualisys
#include "qualisys/RTProtocol.h"
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

// Below for PRId64
#include <cinttypes>
#include <inttypes.h>

#include <stdint.h>

#include "xbee_packet.h"
#include "Quaternion.h"
 
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
            for (unsigned int iPoint = 0; iPoint < poRTProtocol.Get6DOFBodyPointCount(iBody); iPoint++)
            {
                poRTProtocol.Get6DOFBodyPoint(iBody, iPoint, sPoint);
                printf("  Point: X = %9f  Y = %9f  Z = %9f\n", sPoint.fX, sPoint.fY, sPoint.fZ);
            }
            printf("\n");
        }
    }
}
