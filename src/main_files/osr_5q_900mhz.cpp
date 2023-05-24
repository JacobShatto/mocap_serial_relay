#include <getopt.h>
#include <osr_5q_900mhz.hpp>
#include "signal.h" //For SIGINT and SIGTERM

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                          SIGNAL HANDLER                                  //
//            (i.e. what do do when you catch a ctrl+C)                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

// Keep streaming mocap data until this bool is false
static bool keep_running = true;

//This function is used to handle SIGINT (ctrl + c) and other terminal signals to exit cleanly
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
    const char *kTransform = "transform";
    const char *kSerialPort1 = "serialPort1";
    const char *kSerialPort2 = "serialPort2";
    const char *kSerialPort3 = "serialPort3";
    const char *kSerialPort4 = "serialPort4";
    const char *kSerialPort5 = "serialPort5";
    const char *kSerialPort6 = "serialPort6";
    const char *kNumberOfQuads = "numberOfQuads";
    const char *kTestingFakeData = "TestingFakeData";
    const char *kUsingWreckingBall = "usingWreckingBall";
    const char* kNoSerialOutput = "noSerialOutput";

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.1.131","Local network interface for connecting to Optitrack network");
    getopt_add_int(gopt, 'b', kSerialBaudArg, "57600","Serial baudrate for communication via XBee");
    getopt_add_bool(gopt, 't', kTransform, 0,"Transform data from Y-Up to NED Frame");
    getopt_add_string(gopt, '1', kSerialPort1, "/dev/ttyS1","Serial port used to send the XBee packets out (#1)");
    getopt_add_string(gopt, '2', kSerialPort2, "/dev/ttyS2","Serial port used to send the XBee packets out (#2)");
    getopt_add_string(gopt, '3', kSerialPort3, "/dev/ttyS4","Serial port used to send the XBee packets out (#3)");
    getopt_add_string(gopt, '4', kSerialPort4, "/dev/ttyUSB0","Serial port used to send the XBee packets out (#4)");
    getopt_add_string(gopt, '5', kSerialPort5, "/dev/ttyUSB1","Serial port used to send the XBee packets out (#5)");
    getopt_add_string(gopt, '6', kSerialPort6, "/dev/ttyUSB2","Serial port used to send the XBee packets out (#6)");
    getopt_add_int(gopt, 'n', kNumberOfQuads, "1","Number of quadrotors you are commanding");
    getopt_add_bool(gopt, 'T', kTestingFakeData, 0,"Send fake, hardcoded data instead of optitrack for testing");
    getopt_add_bool(gopt, 'W', kUsingWreckingBall, 0, "Grab Wrecking Ball Mocap data and send it");
    getopt_add_bool(gopt, 's', kNoSerialOutput, 0, "Don't Actually send serial data or open ports");
   

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help"))
    {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int baudRate = getopt_get_int(gopt, kSerialBaudArg);
    bool transform = getopt_get_bool(gopt, kTransform);
    int numQuads = getopt_get_int(gopt, kNumberOfQuads);
    bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
    bool usingWreckingBall = getopt_get_bool(gopt, kUsingWreckingBall);
    bool noSerialOutput = getopt_get_bool(gopt, kNoSerialOutput);

    std::vector<std::string> serialPort;
    serialPort.push_back(std::string());
    serialPort.push_back(getopt_get_string(gopt, kSerialPort1));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort2));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort3));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort4));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort5));
    serialPort.push_back(getopt_get_string(gopt, kSerialPort6));

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

    // Quat Transformation
    Quat Q_rotx_90; // Rot about x by 90 degrees
    Q_rotx_90.w = 0.707107;
    Q_rotx_90.x = -0.707107;
    Q_rotx_90.y = 0;
    Q_rotx_90.z = 0;
    Quat Q_rotx_90_inv;
    Q_rotx_90_inv = quatInv(Q_rotx_90);

    // XBee Serial Packet Variables
    std::vector<char *> serialPacket(numQuads + 1, NULL);
    std::vector<char *> dataPacket(numQuads + 1, NULL);
    for (int i = 1; i < numQuads + 1; i++) {
        serialPacket[i] = new char[_MOCAP_PACKET_LENGTH];
        serialPacket[i][0] = XBEE_START_BYTE1;
        serialPacket[i][1] = XBEE_START_BYTE2;
        dataPacket[i] = serialPacket[i] + NUM_START_BYTES;
    }
    std::vector<xbee_packet_t> xb_msg(numQuads + 1, xbee_packet_t());

    // Grab Initial Time
    uint64_t time64_u = (uint64_t)utime_now();
    uint32_t time_u = (uint32_t)time64_u;

    // State variable for coordination of hardcoded waypoints on board quads
    int8_t state = 0;

    // Default Desired Position Variables
    float x_d = 0;
    float y_d = 0;
    float z_d = 0;

   

    // Have variable to store CLI to compute weights
    bool computeWeightsFlag = false;

    //////////////////////////////////////////////////////////////////////////////////////
    //                                                                                  //
    //                                                                                  //
    //                          Open the Serial Ports //
    //                                                                                  //
    //                                                                                  //
    //////////////////////////////////////////////////////////////////////////////////////
    std::vector<int> fd(numQuads + 1, 0);


    if (noSerialOutput) {
        printf("Not opening Serial Ports! \n");
    }
    else {
        for (int i = 1; i < numQuads + 1; i++) {
            fd[i] = serial_open(serialPort[i].c_str(), baudRate, 0); // non-blocking reads
            if (fd[i] == -1) {
                printf("Failed to open Serial Port%d: %s\n", i, serialPort[i].c_str());
                return 0;
            } else {
                printf("Successfully opened Serial Port%d: %s\n", i, serialPort[i].c_str());
            }
        }
    }

    // Writing to Logfile
    FILE *fp = NULL;
    fp = initLoggingXBeeGCS(numQuads);


    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                                                                          //
    //                      Send Fake Data for Testing                          //
    //                                                                          //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////
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
            logXBeeGCS(fp, xb_msg, numQuads);
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                                                                          //
    //                       OPTITRACK SOCKET STUFF                             //
    //                                                                          //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    SOCKET dataSocket = 0;
    std::vector<optitrack_message_t> incomingMessages;


    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
    {
        interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0)
    {
        printf(
            "[optitrack_driver] error could not determine network interface for "
            "receiving multicast packets.\n");
        return -1;
    }

    dataSocket = create_optitrack_data_socket(interface, PORT_DATA);

    if (dataSocket == -1)
    {
        printf(
            "[optitrack_driver] error failed to create socket for interface "
            "%s:%d\n",
            interface.c_str(), PORT_DATA);
        return -1;
    }
    else
    {
        printf(
            "[optitrack_driver] successfully created socket for interface "
            "%s:%d\n",
            interface.c_str(), PORT_DATA);
    }

    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                                                                          //
    //                             MAIN THREAD LOOP                             //
    //                                                                          //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////

    // Bool to check if we're stup signal handerls for SIGINT and SIGTERM
    // Catching these signals is important for reading NTP value at the end of the program
    bool initializedSignals = false;

    initTerminalPrinting();

    while (keep_running)
    {
        // Block until we receive a datagram from the network
        recvfrom(dataSocket, packet, sizeof(packet), 0,
                    (sockaddr *)&incomingAddress, &addrLen);
        incomingMessages =
            parse_optitrack_packet_into_messages(packet, sizeof(packet));

        if (initializedSignals == false)
        {
            //Initialize signal handler for SIGNINT and SIGTERM before we start while(1)
            //This is needed to read NTP offset one last time before program ends
            signal(SIGINT, signal_handler);
            signal(SIGTERM, signal_handler);

            initializedSignals = true;
        }

        // Grab Time
        time64_u = utime_now();
        time_u = (uint32_t)time64_u;

        // Update State from Keyboard Input
        if (updateState(state, computeWeightsFlag, x_d, y_d, z_d))
            return 1;

        for (auto &msg : incomingMessages)
        {
            // Transform the data from Optitrack "Y-UP" To "North East Down" if
            // selected
            if (transform)
            {
                frameTransformation(msg, Q_rotx_90, Q_rotx_90_inv);
            }

            if (msg.id <= numQuads)
            {
                // Construct XBee Packet
                xb_msg[msg.id].x = msg.x;
                xb_msg[msg.id].y = msg.y;
                xb_msg[msg.id].z = msg.z;
                xb_msg[msg.id].yaw = QuatToYaw(msg.qw, msg.qx, msg.qy, msg.qz);

                // Construct Serial Message
                memcpy(dataPacket[msg.id], &(xb_msg[msg.id]),_MOCAP_DATA_LENGTH);

                // Compute Fletcher-16 Checksum
                fletcher16(dataPacket[msg.id], _MOCAP_DATA_LENGTH, dataPacket[msg.id] + _MOCAP_DATA_LENGTH);
            }
        }

        // Send Serial Message
        if (!noSerialOutput) {
            send_serial_xbees(fd, serialPacket, _MOCAP_PACKET_LENGTH);
        }

        // Print to Terminal
        printToTerminal(xb_msg);

        // Write to file
        logXBeeGCS(fp, xb_msg, numQuads);

    } //while(keep_running)

    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                          CLEANUP                                         //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////
    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);

    // TODO: Should really close filestream used for logging

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

    // Time
    // fprintf(fp, "time_Since_Epoch_us, ");

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