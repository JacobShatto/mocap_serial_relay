#include <getopt.h>
#include <opti_serial_relay.hpp>
#include "ntp_read.h"
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
  switch(signum)
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

int main(int argc, char** argv) {
  const char* kInterfaceArg     = "interface";
  const char* kSerialBaudArg    = "baudrate";
  const char* kVerbose          = "verbose";
  const char* kLogging          = "logging";
  const char* kTransform        = "transform";
  const char* kSerialPort1      = "serialPort1";
  const char* kSerialPort2      = "serialPort2";
  const char* kSerialPort3      = "serialPort3";
  const char* kSerialPort4      = "serialPort4";
  const char* kSerialPort5      = "serialPort5";
  const char* kNumberOfQuads    = "numberOfQuads";
  const char* kTestingFakeData  = "TestingFakeData";
  const char* kNTP              = "networkTimeProtocol";
  // const char* kXbeeAddrArg = "xbeeAddr";

  getopt_t* gopt = getopt_create();
  getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
  getopt_add_string(
      gopt, 'i', kInterfaceArg, "192.168.1.131",
      "Local network interface for connecting to Optitrack network");
  getopt_add_int(gopt, 'b', kSerialBaudArg, "57600",
                 "Serial baudrate for communication via XBee");
  getopt_add_bool(gopt, 'v', kVerbose, 0,
                  "Print to terminal the Optitrack data");
  getopt_add_bool(gopt, 'l', kLogging, 0, "Save data to logfile");
  getopt_add_bool(gopt, 't', kTransform, 0,
                  "Transform data from Y-Up to NED Frame");
  getopt_add_string(gopt, '1', kSerialPort1, "/dev/ttyO1",
                    "Serial port used to send the XBee packets out (Leader1)");
  getopt_add_string(gopt, '2', kSerialPort2, "/dev/ttyO2",
                    "Serial port used to send the XBee packets out (Leader2)");
  getopt_add_string(gopt, '3', kSerialPort3, "/dev/ttyO3",
                    "Serial port used to send the XBee packets out (Leader3)");
  getopt_add_string(
      gopt, '4', kSerialPort4, "/dev/ttyO4",
      "Serial port used to send the XBee packets out (Follower1)");
  getopt_add_string(
      gopt, '5', kSerialPort5, "/dev/ttyO5",
      "Serial port used to send the XBee packets out (Follower2)");
  getopt_add_int(gopt, 'n', kNumberOfQuads, "1",
                 "Number of quadrotors you are commanding");
  getopt_add_bool(gopt, 'T', kTestingFakeData, 0,
                  "Send fake, hardcoded data instead of optitrack for testing");
  getopt_add_bool(gopt, 'N', kNTP, 0,
                  "Read NTP offset from NTP server (aka HP laptop)");
  // getopt_add_int(gopt,'x',kXbeeAddrArg, "1", "Address of target XBee");

  if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
    printf("Usage: %s [options]", argv[0]);
    getopt_do_usage(gopt);
    return 1;
  }

  std::string interface = getopt_get_string(gopt, kInterfaceArg);
  int baudRate = getopt_get_int(gopt, kSerialBaudArg);
  bool verbose = getopt_get_bool(gopt, kVerbose);
  bool logging = getopt_get_bool(gopt, kLogging);
  bool transform = getopt_get_bool(gopt, kTransform);
  int numQuads = getopt_get_int(gopt, kNumberOfQuads);
  bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
  bool useNTP = getopt_get_bool(gopt, kNTP);
  // int xbeeAddr = getopt_get_int(gopt, kXbeeAddrArg);

  std::vector<std::string> serialPort;
  serialPort.push_back(std::string());
  serialPort.push_back(getopt_get_string(gopt, kSerialPort1));
  serialPort.push_back(getopt_get_string(gopt, kSerialPort2));
  serialPort.push_back(getopt_get_string(gopt, kSerialPort3));
  serialPort.push_back(getopt_get_string(gopt, kSerialPort4));
  serialPort.push_back(getopt_get_string(gopt, kSerialPort5));

  //////////////////////////////////////////////////////////////////////////////
  //                                                                          //
  //                                                                          //
  //                       Initialize Variables                               //
  //                                                                          //
  //                                                                          //
  //////////////////////////////////////////////////////////////////////////////

  //Initialize NTP variables
  ntp_offset = NTP_ERR_NUM;

  // Quat Transformation
  Quat Q_rotx_90;  // Rot about x by 90 degrees
  Q_rotx_90.w = 0.707107;
  Q_rotx_90.x = -0.707107;
  Q_rotx_90.y = 0;
  Q_rotx_90.z = 0;
  Quat Q_rotx_90_inv;
  Q_rotx_90_inv = quatInv(Q_rotx_90);

  // XBee Serial Packet Variables
  int packetLength = _MOCAP_PACKET_LENGTH;
  std::vector<char*> serialPacket(numQuads + 1, NULL);
  std::vector<char*> dataPacket(numQuads + 1, NULL);

  for (int i = 1; i < numQuads + 1; i++) {
    serialPacket[i] = new char[packetLength];
    serialPacket[i][0] = XBEE_START_BYTE1;
    serialPacket[i][1] = XBEE_START_BYTE2;
    dataPacket[i] = serialPacket[i] + NUM_START_BYTES;
  }

  std::vector<xbee_packet_t> xb_msg(numQuads + 1, xbee_packet_t());

  // Grab Initial Time
  uint64_t time64_u = (uint64_t) utime_now();
  uint32_t time_u = (uint32_t)time64_u;

  // State variable for coordination of hardcoded waypoints on board quads
  int8_t state = 0;

  // Default Desired Position Variables
  float x_d = 0;
  float y_d = 0;
  float z_d = 0;

  // Weights (for quads 4 and 5, the followers)
  // Let w*[0] be unused so indexing is more intuitive
  std::vector<std::vector<float>> w(numQuads + 1, std::vector<float>(6, 0));

  if (numQuads == 5) {
    loadWeightsFromFile(w);
  }
  if (numQuads == 4) {
    w[4][1] = 0.333334;
    w[4][2] = 0.333333;
    w[4][3] = 0.333333;
  }

  // Have variable to store CLI to compute weights
  bool computeWeightsFlag = false;

  // Wind Vane Variables
  bool usingWindVane = false;
  char wind = '\0';
  /** TODO: Make this a get-opt argument.
   * Making it default to /dev/ttyUSB0 is an issue for applications
   * That aren't using the wind vane and need USB0
   * for XBees
   */
  
  int fd_w = -1; 
  if (fd_w == -1) {
    usingWindVane = false;
  }
  

  //////////////////////////////////////////////////////////////////////////////////////
  //                                                                                  //
  //                                                                                  //
  //                          Open the Serial Ports //
  //                                                                                  //
  //                                                                                  //
  //////////////////////////////////////////////////////////////////////////////////////
  std::vector<int> fd(numQuads + 1, 0);

  for (int i = 1; i < numQuads + 1; i++) {
    fd[i] = serial_open(serialPort[i].c_str(), baudRate, 0);  // non-blocking reads
    if (fd[i] == -1) 
    {
      printf("Failed to open Serial Port%d: %s\n", i, serialPort[i].c_str());
      return 0;
    } 
    else 
    {
      printf("Successfully opened Serial Port%d: %s\n", i,
             serialPort[i].c_str());
    }
  }

  // Writing to Logfile
  FILE* fp = NULL;
  if (logging) {
    fp = initLoggingXBeeGCS(numQuads, usingWindVane);
    fprintf(fp, "time_Since_Epoch_us, ");
    fprintf(fp, "ntpOffset_ms, ");
    fprintf(fp, "\n");
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

  if (!testingFakeData) {
    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0) {
      interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0) {
      printf(
          "[optitrack_driver] error could not determine network interface for "
          "receiving multicast packets.\n");
      return -1;
    }

    dataSocket = create_optitrack_data_socket(interface, PORT_DATA);

    if (dataSocket == -1) {
      printf(
          "[optitrack_driver] error failed to create socket for interface "
          "%s:%d\n",
          interface.c_str(), PORT_DATA);
      return -1;
    } else {
      printf(
          "[optitrack_driver] successfully created socket for interface "
          "%s:%d\n",
          interface.c_str(), PORT_DATA);
    }

  } else {
    // Create some fake messages!
    for (int i = 1; i < numQuads + 1; i++) {
      optitrack_message_t fakeMsg;

      fakeMsg.id = i;

      switch (i) {
        case 1:
          fakeMsg.x = 0;
          fakeMsg.y = 0;
          fakeMsg.z = 0;
          break;
        case 2:
          fakeMsg.x = 1;
          fakeMsg.y = 0;
          fakeMsg.z = 0;
          break;
        case 3:
          fakeMsg.x = 0;
          fakeMsg.y = 1;
          fakeMsg.z = 0;
          break;
        case 4:
          fakeMsg.x = 0.5;
          fakeMsg.y = 0.5;
          fakeMsg.z = 0;
          break;
        case 5:
          fakeMsg.x = 0.25;
          fakeMsg.y = 0.25;
          fakeMsg.z = 0;
          break;
      }

      fakeMsg.qx = 0.0001;
      fakeMsg.qy = 0.0001;
      fakeMsg.qz = 0.0001;
      fakeMsg.qw = 0.999;

      fakeMsg.trackingValid = true;

      incomingMessages.push_back(fakeMsg);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////
  //                                                                                  //
  //                                                                                  //
  //                    Print Instructions and Headers //
  //                                                                                  //
  //                                                                                  //
  //////////////////////////////////////////////////////////////////////////////////////
  printCLIforStates();

  if (verbose) {
    printCLIHeaders(numQuads);
    if(useNTP){
      printf("   NTP   |");
    }
    printf("\n");
  }

  //////////////////////////////////////////////////////////////////////////////
  //                                                                          //
  //                READ NTP OFFSETS AT START OF PROGRAM                      //
  //                                                                          //
  //////////////////////////////////////////////////////////////////////////////
  //Update NTP offset
  if(useNTP){
    ntp_offset = get_ntp_offset();
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

  // // Set Terminal for non-blocking
  // set_conio_terminal_mode();

  while (keep_running) {
    if (!testingFakeData) {
      // Block until we receive a datagram from the network
      recvfrom(dataSocket, packet, sizeof(packet), 0,
               (sockaddr*)&incomingAddress, &addrLen);
      incomingMessages =
          parse_optitrack_packet_into_messages(packet, sizeof(packet));

      if(initializedSignals == false){
        //Initialize signal handler for SIGNINT and SIGTERM before we start while(1)
        //This is needed to read NTP offset one last time before program ends
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        initializedSignals = true;
      }
    } else {
      // Fake Data
      unsigned int microseconds = 16670;  // "60 Hz" data rate == 16.67 ms
      usleep(microseconds);
    }

    // Grab Time
    time64_u = utime_now();
    time_u = (uint32_t)time64_u;

    // Update State from Keyboard Input
    if (updateState(state, computeWeightsFlag, x_d, y_d, z_d)) return 1;

    for (auto& msg : incomingMessages) {
      // Transform the data from Optitrack "Y-UP" To "North East Down" if
      // selected
      if (transform) {
        frameTransformation(msg, Q_rotx_90, Q_rotx_90_inv);
      }

      if (msg.id <= numQuads) {
        // Construct XBee Packet
        xb_msg[msg.id].time = time_u;
        xb_msg[msg.id].x = msg.x;
        xb_msg[msg.id].y = msg.y;
        xb_msg[msg.id].z = msg.z;
        xb_msg[msg.id].qx = msg.qx;
        xb_msg[msg.id].qy = msg.qy;
        xb_msg[msg.id].qz = msg.qz;
        xb_msg[msg.id].qw = msg.qw;
        xb_msg[msg.id].trackingValid = msg.trackingValid;
        xb_msg[msg.id].state = state;

        /** Prince TODO: Speak with Matt on adding this back in. Matt needs this
        for Cooperative payload stuff, but in general, most people won't be
        using this.
        // Leader Message
        if (msg.id <= 3) {
          xb_msg[msg.id].x_d = x_d;
          xb_msg[msg.id].y_d = y_d;
          xb_msg[msg.id].z_d = z_d;
        } else {  // Follower Message

          // Re-compute Weights in standby
          if (computeWeightsFlag) {
            w[msg.id] = computeWeights(xb_msg, msg.id);
          }

          // Compute Desired Position from Weighted Sum of Neighbors
          xb_msg[msg.id].x_d = 0;
          xb_msg[msg.id].y_d = 0;
          xb_msg[msg.id].z_d = 0;
          for (int i = 1; i < numQuads + 1; i++) {
            xb_msg[msg.id].x_d += w[msg.id][i] * xb_msg[i].x;
            xb_msg[msg.id].y_d += w[msg.id][i] * xb_msg[i].y;
            xb_msg[msg.id].z_d += w[msg.id][i] * xb_msg[i].z;
          }
        }
        **/

        // Construct Serial Message
        memcpy(dataPacket[msg.id], &(xb_msg[msg.id]),
               packetLength - NUM_FRAMING_BYTES);

        // Compute Fletcher-16 Checksum
        fletcher16(dataPacket[msg.id], _MOCAP_DATA_LENGTH, dataPacket[msg.id] + sizeof(xbee_packet_t));

        // Send Serial Message
        if (write(fd[msg.id], serialPacket[msg.id], packetLength) > 0) {
        } else
          printf("Error Serial Port %d: %d \n", msg.id, errno);
      }
    }

    computeWeightsFlag = 0;

    // Wait for all of the Serial Ports to finish writing ("flush" the data)
    for (int i = 1; i < numQuads + 1; i++) {
      fsync(fd[i]);
    }

    // Print Message to Terminal if selected
    if (verbose) {
      printXBeeMsg(xb_msg);
      if(useNTP){
        printf("%+09.2lf|", ntp_offset);
      }
      fflush(stdout);
    }

    // Write to Log File is selected
    if (logging) {
      logXBeeMsgs(fp, xb_msg, numQuads, w, fd_w, usingWindVane, wind);
      fprintf(fp,"%" PRIu64 ",",time64_u);
      fprintf(fp," %lf, ",ntp_offset);
      fprintf(fp, "\n");
    }
  }//while(keep_running)

  //Commands to read and save one last NTP offset value before ending program
  if(useNTP)
  {
    //Read NTP offset one last time before exiting program
    double old_ntp_offset = ntp_offset;
    ntp_offset = get_ntp_offset();

    // One last log to get last NTP offset
    if (logging) {
      logXBeeMsgs(fp, xb_msg, numQuads, w, fd_w, usingWindVane, wind);
      fprintf(fp,"%" PRIu64 ",",time64_u);
      fprintf(fp," %lf, ",ntp_offset);
      fprintf(fp, "\n");
    }

    //One last output to see last NTP offset
    if (verbose) {
      printXBeeMsg(xb_msg);
      printf("%+09.2lf|", ntp_offset);
      printf("\nNTP Offset from %4.4lf to %4.4lf\n", old_ntp_offset, ntp_offset); 
      fflush(stdout);
    }
  }
  


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
