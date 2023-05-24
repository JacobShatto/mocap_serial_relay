// Updated code (Feb 2019):  Use more robust serial message structure with Fletcher-16

#include "qsr_5q_900mhz.hpp"
// #include "printing.hpp"
#include "signal.h" //For SIGINT and SIGETERM

//This function is used to handle SIGINT (ctrl + c) and other terminal signals to exit cleanly
static bool keep_running = true;

//This enum keeps track of which rotation sequence we have from Qualisys
// Euler_Rotation_Sequence euler_sequence = invalid_rotation_sequence;

void signal_handler(int signum)
{
    //Note: Stackoverflow recommends NOT adding print statements inside the signal handler
    //since printf is not reentrant. rc_pilot violates this rule and I (Prince) don't know enough about
    //this to risk it.
    switch (signum)
    {
    //SIGINT is |ctrl + c|
    case SIGINT:
        keep_running = 0;
        break;
    //SIGTERM
    case SIGTERM:
        keep_running = 0;
        break;
    }
}


int main(int argc, char **argv)
{
    const char *kInterfaceArg = "interface";
    const char *kSerialBaudArg = "baudrate";
    const char *kSerialPort1 = "serialPort1";
    const char *kSerialPort2 = "serialPort2";
    const char *kSerialPort3 = "serialPort3";
    const char *kSerialPort4 = "serialPort4";
    const char *kSerialPort5 = "serialPort5";
    const char *kSerialPort6 = "serialPort6";
    const char *kUsingWreckingBall = "usingWreckingBall";
    const char *kNumberOfQuads = "numberOfQuads";
    const char *kTestingFakeData = "TestingFakeData";
    const char* kNoSerialOutput = "noSerialOutput";


    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.254.1", "Local network interface for connecting to Optitrack network");
    getopt_add_int(gopt, 'b', kSerialBaudArg, "57600", "Serial baudrate for communication via XBee");
    getopt_add_string(gopt, '1', kSerialPort1, "/dev/ttyS1", "Serial port used to send the XBee packets out (#1)");
    getopt_add_string(gopt, '2', kSerialPort2, "/dev/ttyS2", "Serial port used to send the XBee packets out (#2)");
    getopt_add_string(gopt, '3', kSerialPort3, "/dev/ttyS4", "Serial port used to send the XBee packets out (#3)");
    getopt_add_string(gopt, '4', kSerialPort4, "/dev/ttyUSB0", "Serial port used to send the XBee packets out (#4)");
    getopt_add_string(gopt, '5', kSerialPort5, "/dev/ttyUSB1", "Serial port used to send the XBee packets out (#5)");
    getopt_add_string(gopt, '6', kSerialPort6, "/dev/ttyUSB2", "Serial port used to send the XBee packets out (#6)");
    getopt_add_bool(gopt, 'W', kUsingWreckingBall, 0, "Grab Wrecking Ball Mocap data and send it");
    getopt_add_int(gopt, 'n', kNumberOfQuads, "1", "Number of quadrotors you are commanding");
    getopt_add_bool(gopt, 'T', kTestingFakeData, 0, "Send fake, hardcoded data instead of optitrack for testing");
    getopt_add_bool(gopt, 's', kNoSerialOutput, 0, "Don't Actually send serial data or open ports");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help"))
    {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int baudRate = getopt_get_int(gopt, kSerialBaudArg);
    bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
    int numQuads = getopt_get_int(gopt, kNumberOfQuads);     
    bool usingWreckingBall = getopt_get_bool(gopt, kUsingWreckingBall);
    bool noSerialOutput = getopt_get_bool(gopt, kNoSerialOutput);
    bool debugging = false;


    std::vector<std::string> serialPort;
    serialPort.push_back(std::string());
    serialPort.push_back(getopt_get_string(gopt, kSerialPort1));   
    serialPort.push_back(getopt_get_string(gopt, kSerialPort2));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort3));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort4));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort5)); 
    serialPort.push_back(getopt_get_string(gopt, kSerialPort6)); 

    // Qualisys Rigid Body Labels
    std::unordered_map<std::string, int> bodyLabels2VehicleNum;
    bodyLabels2VehicleNum["Quad1"] = 1;
    bodyLabels2VehicleNum["Quad2"] = 2;
    bodyLabels2VehicleNum["Quad3"] = 3;
    bodyLabels2VehicleNum["Quad4"] = 4;
    if ( (numQuads == 4) && (usingWreckingBall) ) {
        // bodyLabels2VehicleNum["Quad5"] = 5;
        bodyLabels2VehicleNum["WreckingBall"] = 5;
    } else {
        bodyLabels2VehicleNum["Quad5"] = 5;
        bodyLabels2VehicleNum["WreckingBall"] = 6;
    }
    
     //consider the wb an extra quad
    if (usingWreckingBall) {
        numQuads++;
    }

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                       Initialize Variables                               //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
	// Serial Packet Variables
    std::vector<char* > serialPacket(numQuads+1,NULL);
    std::vector<char* > dataPacket(numQuads+1,NULL);
    for (int i=1; i < numQuads+1; i++) {
        serialPacket[i] = new char[_MOCAP_PACKET_LENGTH];
        serialPacket[i][0] = XBEE_START_BYTE1; 
        serialPacket[i][1] = XBEE_START_BYTE2;  
        dataPacket[i] = serialPacket[i] + NUM_START_BYTES;
    }
    std::vector<xbee_packet_t> xb_msg(numQuads+1,xbee_packet_t());

    // Grab Initial Time
    int64_t init_time64_u = utime_now();
    int64_t time64_u = utime_now();
    uint32_t time_u = (uint32_t) (time64_u - init_time64_u);

    // State variable for coordination of hardcoded waypoints on board quads
    int8_t state = 0;

    // Default Desired Position Variables
    float x_d = 0;
    float y_d = 0;
    float z_d = 0;

//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//                                                                                  //
//                          Open the Serial Ports                                   //
//                                                                                  //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////
    std::vector<int> fd(numQuads+1,0);

    if (noSerialOutput) {
        printf("Not opening Serial Ports! \n");
    }
    else {
        for (int i=1; i < numQuads+1; i++) {
            fd[i] = serial_open(serialPort[i].c_str(),baudRate,0); 	// non-blocking reads
            if(fd[i] == -1) {
                printf("Failed to open Serial Port%d: %s\n", i, serialPort[i].c_str());
                return 0;
            } else {
                printf("Successfully opened Serial Port%d: %s\n", i, serialPort[i].c_str());
            }
        }
    }

    // Writing to Logfile
    FILE *fp_logging = initLoggingXBeeGCS(numQuads);

    // Grab Initial Time
    uint64_t time64_us = (uint64_t)utime_now();
    uint32_t time32_us = (uint32_t)time64_us;


    bool computeWeightsFlag = false;

    // set_conio_terminal_mode();  // Set Terminal for non-blocking

    // Wind Vane Variables
    // bool usingWindVane = false;
    // char wind;
    // int fd_w;
    /* 
     fd_w = serial_open("/dev/ttyUSB0",9600,0); 	// non-blocking reads
     if(fd_w == -1) {
     usingWindVane = false;
     }
  */

//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//                                                                                  //
//                         Send Fake Data for Testing                               //
//                                                                                  //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////
    if (testingFakeData)
    {
        // Send data at "60 Hz" = 16.67 ms =  16,670 us
        // Send data at "10 Hz" = 100 ms = 100,000 us
        unsigned int microseconds = 16670;

        for (int i=1; i < numQuads+1; i++)
        {
            xb_msg[i].yaw = 0;
        }

        float xTest = 0;
        float yTest = 0; 
        float zTest = 0;

        printCLIforStatesFakeData();
        initTerminalPrinting();

        while (1)
        {
            usleep(microseconds);
            time64_us = utime_now();
            time32_us = (uint32_t)time64_us;

            // Update State from Keyboard Input
            if (updateState(state, computeWeightsFlag, xTest, yTest, zTest))
                return 1;

            // Construct Serial Message
            for (int i=1; i < numQuads+1; i++) {
                // Construct Serial Message
                xb_msg[i].x = xTest;
                xb_msg[i].y = yTest;
                xb_msg[i].z = zTest;
                memcpy(dataPacket[i], &(xb_msg[i]), _MOCAP_DATA_LENGTH);

                // Obtain CRC
                fletcher16(dataPacket[i], _MOCAP_DATA_LENGTH, dataPacket[i] + _MOCAP_DATA_LENGTH);               
            }

            // Send Serial Message
            if (!noSerialOutput) {
                send_serial_xbees(fd, serialPacket, _MOCAP_PACKET_LENGTH);
            }

            // Print to Terminal
            printToTerminal(xb_msg);

            // Write to file
            logXBeeGCS(fp_logging, xb_msg, numQuads);
        }
    }

    // Qualisys code
    CRTProtocol poRTProtocol;
    printf("Trying to connect to Qualisys Network at %s:%d... \n", (char *)interface.data(), QTM_RT_SERVER_BASE_PORT);

    if (poRTProtocol.Connect((char *)interface.data(), QTM_RT_SERVER_BASE_PORT, 0, 1, 7) == false)
    {
        fprintf(stderr, "Unable to connect to Qualisys Network at %s:%d... \n",
                (char *)interface.data(), QTM_RT_SERVER_BASE_PORT);
        return -1;
    }

    //Get QTM Settings (prints to terminal list of rigid bodies received from QTM)
    getQTMSettings(poRTProtocol);

    //Not sure what this does exactly...
    poRTProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL, CRTProtocol::Component6dEuler);

    //Variables used to store data received from Qualisys Network
    CRTPacket::EPacketType eType;
    unsigned int nCount;
    CRTPacket *pRTPacket;
    float fX, fY, fZ, fAng1, fAng2, fAng3, fRoll, fPitch, fYaw;
    // bool                   bKeyAbort  = false;

    //Euler to quaternion
    float deg2rad = 3.14159265 / 180;
    Quat quat;
    Euler_Rotation_Sequence euler_sequence = select_euler_rotation_sequence(poRTProtocol.Get6DOFBodyFirstEuler(0),
                                                                            poRTProtocol.Get6DOFBodySecondEuler(0),
                                                                            poRTProtocol.Get6DOFBodyThirdEuler(0));
    if (euler_sequence == invalid_rotation_sequence)
    {
        printf("Improper Euler Angle Rotation Sequence: %s -> %s -> %s\n",
               poRTProtocol.Get6DOFBodyFirstEuler(0), poRTProtocol.Get6DOFBodySecondEuler(0), poRTProtocol.Get6DOFBodyThirdEuler(0));
        return -1;
    }
    if (euler_sequence < 0)
    {
        printf("Given Euler Rotation sequence is not 'Proper/Classic' (aka Tait-Bryan): %s -> %s -> %s\n",
               poRTProtocol.Get6DOFBodyFirstEuler(0), poRTProtocol.Get6DOFBodySecondEuler(0), poRTProtocol.Get6DOFBodyThirdEuler(0));
        return -1;
    }

    initTerminalPrinting();

    //Initialize signal handler for SIGNINT and SIGTERM before we start while(1)
    //This is needed to read NTP offset one last time before program ends
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);



    //Main while-loop: continues until SIGINT is received
    while (keep_running)
    {
        // Grab Time
        time64_us = utime_now();
        time32_us = (uint32_t)time64_us;

        // Update UAV State from Keyboard Input
        if (updateState(state, computeWeightsFlag, x_d, y_d, z_d))
            return 1;

        //Attempt to receive rigid body data from Qualisys network (using socket)
        if (poRTProtocol.ReceiveRTPacket(eType, true, 500000))
        {
            switch (eType)
            {

            case CRTPacket::PacketError:
                // sHeader.nType 0 indicates an error
                fprintf(stderr, "Error when streaming frames: %s\n", poRTProtocol.GetRTPacket()->GetErrorString());
                break;

            case CRTPacket::PacketData:
                // Data received
                pRTPacket = poRTProtocol.GetRTPacket();
                nCount = pRTPacket->Get6DOFEulerBodyCount();

                if (nCount == 0)
                {
                    break;
                }

                for (unsigned int bodyID = 0; bodyID < nCount; bodyID++)
                {
                    char* label = (char*)poRTProtocol.Get6DOFBodyName(bodyID);
                    char  emptyString[] = "";
                    if (label == NULL)
                    {
                        label = emptyString;
                    }

                    pRTPacket->Get6DOFEulerBody(bodyID, fX, fY, fZ, fAng1, fAng2, fAng3);

                    if (std::isnan(fX) || std::isnan(fY) || std::isnan(fZ) ||
                        std::isnan(fAng1) || std::isnan(fAng2) || std::isnan(fAng3))
                    {
                        continue;
                    }

                    arrange_euler_angles(euler_sequence, fRoll, fPitch, fYaw, fAng1, fAng2, fAng3);

                    // Convert Euler to Quaternion
                    quat = EulerToQuat(fRoll * deg2rad, fPitch * deg2rad, fYaw * deg2rad);

                    int ind;
                    std::string label_str = (std::string) label;
                    if (bodyLabels2VehicleNum.count(label_str) == 0) {
                        if (debugging) {
                            printf("Label Not Found!\n");
                        }
                        continue;
                    } else {
                        ind = bodyLabels2VehicleNum[label_str];
                        if (debugging) {
                            printf("Label Found! %s: #%d\n",label,ind);
                        }
                    }
                    
                    if ( ind <= numQuads) {
                        // Construct XBee Packet
                        xb_msg[ind].x = fX / 1000;      // mm -> m
                        xb_msg[ind].y = fY / 1000;      // mm -> m
                        xb_msg[ind].z = fZ / 1000;      // mm -> m
                        xb_msg[ind].yaw = fAng3*deg2rad;    // deg -> rad
            
                        // Construct Serial Message
                        memcpy(dataPacket[ind], &(xb_msg[ind]), _MOCAP_DATA_LENGTH);

                        // Obtain CRC
                        fletcher16(dataPacket[ind], _MOCAP_DATA_LENGTH, dataPacket[ind] + _MOCAP_DATA_LENGTH);
                    }

                   

                }  // for (bodyID... )

                // Send Serial Messages to All!
                if (!noSerialOutput) {
                    send_serial_xbees(fd, serialPacket, _MOCAP_PACKET_LENGTH);
                }

                // Print to Terminal
                printToTerminal(xb_msg);

                // Write to file
                logXBeeGCS(fp_logging, xb_msg, numQuads);
                break;


            default:
                break;
            } //switch(eType)
        }     //if(received packet)
    }         //while(keep_running)

    poRTProtocol.StreamFramesStop();
    poRTProtocol.Disconnect(); // Disconnect from the server

    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);
    fclose(fp_logging);
    return 0;
}

void send_serial_xbees(std::vector<int>  fd, std::vector<char* > serialPacket, int packetLength) {
    // Send the Data
    // std::cout << std::endl << std::endl;
    for (int i=1; i < serialPacket.size(); i++) {
        int cm1_val = i;
        int cm2_val = i + 3;
        bool send_cm1 = cm1_val < serialPacket.size();
        bool send_cm2 = cm2_val < serialPacket.size();

        if (send_cm1) {
            // std::cout << "Sending cm1! #" << cm1_val << std::endl;
            if(write(fd[cm1_val],serialPacket[cm1_val],packetLength) > 0) {} 
            else printf("Error Serial Port fdCM1: %d \n",errno); 
        }
        if (send_cm2) {
            // std::cout << "Sending cm2! #" << cm2_val << std::endl;
            if(write(fd[cm2_val],serialPacket[cm2_val],packetLength) > 0) {} 
            else printf("Error Serial Port fdCM2: %d \n",errno); 
        }

        if (send_cm1 || send_cm2) {
            fsync(fd[cm1_val]);
            fsync(fd[cm2_val]);
            // unsigned int sleep_time = CHAR_DELAY_US * (0); 
            // std::cout << "sleeping for " << sleep_time << " us" << std::endl;
            // unsigned int time1 = utime_now();
            // usleep(sleep_time);
            // unsigned int time2 = utime_now();
            // std::cout << "Actually slept for " << time2 - time1 << " us" << std::endl;
        }

    }
}

Euler_Rotation_Sequence select_euler_rotation_sequence(const char *first_axis, const char *second_axis, const char *third_axis)
{
    if (!strcmp("Yaw", first_axis) && !strcmp("Roll", second_axis) && !strcmp("Yaw", third_axis))
    {
        return yaw_roll_yaw;
    }
    else if (!strcmp("Roll", first_axis) && !strcmp("Pitch", second_axis) && !strcmp("Roll", third_axis))
    {
        return roll_pitch_roll;
    }
    else if (!strcmp("Pitch", first_axis) && !strcmp("Yaw", second_axis) && !strcmp("Pitch", third_axis))
    {
        return pitch_yaw_pitch;
    }
    else if (!strcmp("Yaw", first_axis) && !strcmp("Pitch", second_axis) && !strcmp("Yaw", third_axis))
    {
        return yaw_pitch_yaw;
    }
    else if (!strcmp("Roll", first_axis) && !strcmp("Yaw", second_axis) && !strcmp("Roll", third_axis))
    {
        return roll_yaw_roll;
    }
    else if (!strcmp("Pitch", first_axis) && !strcmp("Roll", second_axis) && !strcmp("Pitch", third_axis))
    {
        return pitch_roll_pitch;
    }
    else if (!strcmp("Roll", first_axis) && !strcmp("Pitch", second_axis) && !strcmp("Yaw", third_axis))
    {
        return roll_pitch_yaw;
    }
    else if (!strcmp("Pitch", first_axis) && !strcmp("Yaw", second_axis) && !strcmp("Roll", third_axis))
    {
        return pitch_yaw_roll;
    }
    else if (!strcmp("Yaw", first_axis) && !strcmp("Roll", second_axis) && !strcmp("Pitch", third_axis))
    {
        return yaw_roll_pitch;
    }
    else if (!strcmp("Roll", first_axis) && !strcmp("Yaw", second_axis) && !strcmp("Pitch", third_axis))
    {
        return roll_yaw_pitch;
    }
    else if (!strcmp("Yaw", first_axis) && !strcmp("Pitch", second_axis) && !strcmp("Roll", third_axis))
    {
        return yaw_pitch_roll;
    }
    else if (!strcmp("Pitch", first_axis) && !strcmp("Roll", second_axis) && !strcmp("Yaw", third_axis))
    {
        return pitch_roll_yaw;
    }

    return invalid_rotation_sequence;
}

bool arrange_euler_angles(Euler_Rotation_Sequence seq,
                          float &fRoll, float &fPitch, float &fYaw,
                          float fAng1, float fAng2, float fAng3)
{
    switch (seq)
    {
    //"Proper/Classic" Euler rotations (aka Tair-Bryan) will be assigned values
    case roll_pitch_yaw:
        fRoll = fAng1;
        fPitch = fAng2;
        fYaw = fAng3;
        break;
    case pitch_yaw_roll:
        fPitch = fAng1;
        fYaw = fAng2;
        fRoll = fAng3;
        break;
    case yaw_roll_pitch:
        fYaw = fAng1;
        fRoll = fAng2;
        fPitch = fAng3;
        break;
    case roll_yaw_pitch:
        fRoll = fAng1;
        fYaw = fAng2;
        fPitch = fAng3;
        break;
    case yaw_pitch_roll:
        fYaw = fAng1;
        fPitch = fAng2;
        fRoll = fAng3;
        break;
    case pitch_roll_yaw:
        fPitch = fAng1;
        fRoll = fAng2;
        fYaw = fAng3;
        break;
    default:
        fRoll = fPitch = fYaw = 0;
        printf("ERROR in 'arrange_euler_angles': Given Euler Sequence must have three different axes\n");
        return 1;
        break;
    }
    return 0;
}

// Print CLI for States
void printCLIforStatesFakeData() {
    printf("\nControl Position Test Data as Follows: \n");
    printf(" @: Z++ \n");
    printf(" X: Z-- \n");
    printf(" W: X++ \n");
    printf(" A: Y-- \n");
    printf(" S: X-- \n");
    printf(" D: Y++ \n");
    printf(" O: X=Y=Z=0 \n");
}


void initTerminalPrinting() 
{    


    printf("\n");

    // Position
    printf("     x    |");
    printf("     y    |");
    printf("     z    |");

    // Yaw
    printf("   yaw    |");

	printf("\n");
}

void printToTerminal(std::vector<xbee_packet_t> &msg)
{
    // Print Quad 1's info regardless of numQuads

    printf("\r");

    printf("%7.6f, ", msg[1].x);
    printf("%7.6f, ", msg[1].y);
    printf("%7.6f, ", msg[1].z);
    printf("%7.6f, ", msg[1].yaw);

    fflush(stdout);
}


FILE* initLoggingXBeeGCS(int numQuads) {
    char * fName = "mocap_logfile.csv";
    FILE* fp = fopen(fName, "w");

	// Data
    for (int i=1; i < numQuads+1; i++) {
        fprintf(fp, "Quad %d, ", i);
	    fprintf(fp, "x, ");
	    fprintf(fp, "y, ");
	    fprintf(fp, "z, ");
        fprintf(fp, "yaw, ");
    }

	// End of Line
	fprintf(fp,"\n");

    return fp;
}

void logXBeeGCS(FILE *fp, std::vector<xbee_packet_t> xb_msg, int numQuads) {
    if (fp == NULL) {
        printf("Issue Writing to Log, fp is NULL\n");
        return;
    }
    
    for (int i=1; i < numQuads+1; i++) {
        fprintf(fp, " , ");
        fprintf(fp, "%7.6f, ",xb_msg[i].x); 
        fprintf(fp, "%7.6f, ",xb_msg[i].y); 
        fprintf(fp, "%7.6f, ",xb_msg[i].z); 
        fprintf(fp, "%7.6f, ",xb_msg[i].yaw);
    }

    // End of Line
    fprintf(fp,"\n");
}

