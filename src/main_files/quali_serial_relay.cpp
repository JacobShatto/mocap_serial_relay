// Updated code (Feb 2019):  Use more robust serial message structure with Fletcher-16

#include "quali_serial_relay.hpp"
#include "printing.hpp"
#include "ntp_read.h"
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


void initLogging(FILE **fpinput);
unsigned int bodyID = 0;  // Rigid body ID from Qualisys (hard coded for this version)

int main(int argc, char** argv)
{
  const char* kInterfaceArg     = "interface";
  const char* kSerialBaudArg    = "baudrate";
  const char* kVerbose          = "verbose";
  const char* kLogging          = "logging";    
  const char* kTransform        = "transform";
  const char* kSerialPort       = "serialPort";
  const char* kTestingFakeData  = "TestingFakeData";
  const char* kNTP              = "networkTimeProtocol";
  // const char* kXbeeAddrArg = "xbeeAddr";
  
  getopt_t* gopt = getopt_create();
  getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
  getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.254.1", "Local network interface for connecting to Optitrack network"); 
  getopt_add_int(gopt, 'b', kSerialBaudArg, "57600", "Serial baudrate for communication via XBee");
  getopt_add_bool(gopt,'v',kVerbose, 0, "Print to terminal the Optitrack data");
  getopt_add_bool(gopt,'l',kLogging, 0, "Save data to logfile");
  getopt_add_bool(gopt,'t',kTransform, 0, "Transform data from Y-Up to NED Frame");
  getopt_add_string(gopt, 's',kSerialPort, "/dev/ttyUSB0", "Serial port used to send the XBee packets out");  
  getopt_add_bool(gopt, 'T',kTestingFakeData, 0, "Send fake, hardcoded data instead of optitrack for testing");
  getopt_add_bool(gopt, 'N', kNTP, 0, "Read NTP offset from NTP server (aka HP laptop)");  
  // getopt_add_int(gopt,'x',kXbeeAddrArg, "1", "Address of target XBee");
  // getopt_add_int(gopt,'r',kBBRigidBodyArg, "1", "ID of rigid body to publish.");
  
  if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
    printf("Usage: %s [options]", argv[0]);
    getopt_do_usage(gopt);
    return 1;
  }

  std::string interface = getopt_get_string(gopt, kInterfaceArg);
  int baudRate = getopt_get_int(gopt, kSerialBaudArg);
  bool verbose = getopt_get_bool(gopt, kVerbose);
  bool logging = getopt_get_bool(gopt, kLogging);
  // bool transform = getopt_get_bool(gopt, kTransform);
  std::string serialPort = getopt_get_string(gopt, kSerialPort);
  bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
  bool useNTP = getopt_get_bool(gopt, kNTP);
  // int xbeeAddr = getopt_get_int(gopt, kXbeeAddrArg);
  

  // XBee Serial Packet Variables
  int XBEE_portID;  
  xbee_packet_t xb_msg;
  char* serialPacket;
  char* dataPacket;

  // Initialize Packet Structure
  serialPacket = new char[_MOCAP_PACKET_LENGTH];
  serialPacket[0] = XBEE_START_BYTE1;
  serialPacket[1] = XBEE_START_BYTE2;
  dataPacket = serialPacket + NUM_START_BYTES;


  // Open XBee serial port
  printf("serial = %s\n", serialPort.c_str());
  XBEE_portID = serial_open(serialPort.c_str(),baudRate,1); 	// blocking while sending
  if(XBEE_portID == -1)  {
    printf("Failed to open Serial Port");
    return 1;
  }
  
  // Configure XBee Destination Address
  // if(set_xbee_dest_addr(XBEE_portID, xbeeAddr))
  // {
  //   printf("Could not set Xbee address\n");
  // }
  
  // Quat Transformation
  Quat Q_rotx_90;   // Rot about x by 90 degrees
  Q_rotx_90.w = 0.707107;
  Q_rotx_90.x = -0.707107;
  Q_rotx_90.y = 0;
  Q_rotx_90.z = 0; 
  Quat Q_rotx_90_inv;
  Q_rotx_90_inv = quatInv(Q_rotx_90);
  
  printf("Data size = %d\n",(int) sizeof(xbee_packet_t));
  if(verbose)	{
    // Printf Headers
    printf("\n");
    //printf("           |             Position              |                 RPY               |                  Quaternion                   |\n");      
    printf("   Time   |");
    printf("     x    |");
    printf("     y    |");
    printf("     z    |");
    printf("    qx    |");
    printf("    qy    |");
    printf("    qz    |");
    printf("    qw    |");
    printf(" TrackingValid |");
    printf("  State  |");
    printf(" NTP Offset |");
    printf(" Roll (deg) |");
    printf(" Pitch(deg) |");
    printf(" Yaw  (deg) |");
    
    printf("\n");
  }
  	
  // Writing to Logfile
  FILE *fpblah; 
  if(logging) 
    initLogging(&fpblah);

  // Grab Initial Time
  uint64_t time64_us = (uint64_t) utime_now();
  uint32_t time32_us = (uint32_t) time64_us;

  
  // Accept keyboard inputs for state transitions (standy, takeoff, etc...)
  int8_t state = 0;
  bool computeWeightsFlag = false;
  float x_d = 0;
  float y_d = 0;
  float z_d = 0;
  int numQuads = 0;
  printCLIforStates();
  if (verbose) {
    printCLIHeaders(numQuads);
  }
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
  
  //                      Testing Code (Fake Data)                           
  if (testingFakeData) {
    // Send data at "60 Hz" = 16.67 ms =  16,670 us
    // Send data at "10 Hz" = 100 ms = 100,000 us
    unsigned int microseconds = 16670;
    float testZero = 0;
    
    while (1) {
      usleep(microseconds);              
      time64_us = utime_now();
      time32_us = (uint32_t)time64_us;

      // Update State from Keyboard Input
      if (updateState(state, computeWeightsFlag, x_d, y_d, z_d)) return 1;
      
      // Construct XBee Packet
      xb_msg.time = time32_us;

      //Stay at Origin
      xb_msg.x = x_d;
      xb_msg.y = y_d;
      xb_msg.z = z_d;

      //Unit quaternion
      xb_msg.qx = testZero;
      xb_msg.qy = testZero;
      xb_msg.qz = testZero;
      xb_msg.qw = 1;

      xb_msg.trackingValid = 1;
      xb_msg.state = state;

      // Construct Serial Message
      memcpy(dataPacket, &xb_msg, _MOCAP_DATA_LENGTH);
      fletcher16(dataPacket, _MOCAP_DATA_LENGTH, dataPacket + _MOCAP_DATA_LENGTH);
      
      // send serial message	(returns # of bytes sent on success)
      if(write(XBEE_portID, serialPacket, _MOCAP_PACKET_LENGTH) > 0) 
      {
        if(verbose)	    printXBeeMsg(xb_msg);  
        // "flush" the data 
        fsync(XBEE_portID);
      }			
      else			
        printf("Error: %d \n",errno);		

    }
  }

  // Qualisys code
  CRTProtocol poRTProtocol;   
  printf("Trying to connect to Qualisys Network at %s:%d... \n",(char*)interface.data(),QTM_RT_SERVER_BASE_PORT);

  if (poRTProtocol.Connect((char*)interface.data(), QTM_RT_SERVER_BASE_PORT, 0, 1, 7) == false)
  {
    fprintf(stderr, "Unable to connect to Qualisys Network at %s:%d... \n",
              (char*)interface.data(),QTM_RT_SERVER_BASE_PORT);
    return -1;
  }

  //Get QTM Settings (prints to terminal list of rigid bodies received from QTM)
  getQTMSettings(poRTProtocol);

  //Not sure what this does exactly...
  poRTProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL, CRTProtocol::Component6dEuler);

  //Variables used to store data received from Qualisys Network
  CRTPacket::EPacketType eType;
  unsigned int           nCount;
  CRTPacket*             pRTPacket;
  float                  fX, fY, fZ, fAng1, fAng2, fAng3, fRoll, fPitch, fYaw;
  // bool                   bKeyAbort  = false;

  //Euler to quaternion 
  float deg2rad = 3.14159265 / 180;
  Quat quat;
  Euler_Rotation_Sequence euler_sequence = select_euler_rotation_sequence(poRTProtocol.Get6DOFBodyFirstEuler(0), 
                                                                          poRTProtocol.Get6DOFBodySecondEuler(0), 
                                                                          poRTProtocol.Get6DOFBodyThirdEuler(0));
  if(euler_sequence == invalid_rotation_sequence)
  {
    printf("Improper Euler Angle Rotation Sequence: %s -> %s -> %s\n", 
          poRTProtocol.Get6DOFBodyFirstEuler(0), poRTProtocol.Get6DOFBodySecondEuler(0), poRTProtocol.Get6DOFBodyThirdEuler(0));
    return -1;
  }
  if(euler_sequence < 0)
  {
    printf("Given Euler Rotation sequence is not 'Proper/Classic' (aka Tait-Bryan): %s -> %s -> %s\n", 
          poRTProtocol.Get6DOFBodyFirstEuler(0), poRTProtocol.Get6DOFBodySecondEuler(0), poRTProtocol.Get6DOFBodyThirdEuler(0));
    return -1;
  }
  
  //Read NTP offset once at start of program
  if(useNTP)
  {
    ntp_offset = NTP_ERR_NUM;
    ntp_offset = get_ntp_offset();
  }

  //Initialize signal handler for SIGNINT and SIGTERM before we start while(1)
  //This is needed to read NTP offset one last time before program ends
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);


  //Main while-loop: continues until SIGINT is received
  while (keep_running) {
    // Grab Time
    time64_us = utime_now();
    time32_us = (uint32_t)time64_us;

    // Update UAV State from Keyboard Input
    if (updateState(state, computeWeightsFlag, x_d, y_d, z_d)) return 1;

    //Attempt to receive rigid body data from Qualisys network (using socket)
    if (poRTProtocol.ReceiveRTPacket(eType, true, 500000)){
      switch (eType) {

        case CRTPacket::PacketError : 
          // sHeader.nType 0 indicates an error
          fprintf(stderr, "Error when streaming frames: %s\n", poRTProtocol.GetRTPacket()->GetErrorString());
          break;

        case CRTPacket::PacketData:         
          // Data received
          pRTPacket = poRTProtocol.GetRTPacket();
          nCount  = pRTPacket->Get6DOFEulerBodyCount();

          if(nCount == 0)
          {
            break;
          }
          
          //  HARDCODED bodyID to 1
          pRTPacket->Get6DOFEulerBody(bodyID, fX, fY, fZ, fAng1, fAng2, fAng3);
          if(std::isnan(fX) ||  std::isnan(fY) ||  std::isnan(fZ) ||  
              std::isnan(fAng1) ||  std::isnan(fAng2) ||  std::isnan(fAng3))
          {
            continue;
          }

          arrange_euler_angles(euler_sequence, fRoll, fPitch, fYaw, fAng1, fAng2, fAng3);
            
          // Convert Euler to Quaternion
          quat = EulerToQuat(fRoll*deg2rad, fPitch*deg2rad, fYaw*deg2rad);

          // Construct XBee Packet
          xb_msg.time = time32_us;
          xb_msg.x = fX / 1000.0; //Convert from mm to m
          xb_msg.y = fY / 1000.0; //Convert from mm to m
          xb_msg.z = fZ / 1000.0; //Convert from mm to m
          xb_msg.qx = quat.x;
          xb_msg.qy = quat.y;
          xb_msg.qz = quat.z;
          xb_msg.qw = quat.w;
          xb_msg.trackingValid = 1; // Hardcoded to 1 for Qualisys for now; (uint32_t) msg.trackingValid;
          xb_msg.state = state;

          // Construct Serial Message
          memcpy(dataPacket, &xb_msg, _MOCAP_DATA_LENGTH);
          fletcher16(dataPacket, _MOCAP_DATA_LENGTH, dataPacket + _MOCAP_DATA_LENGTH);
          
          // send serial message	(returns # of bytes sent on success)
          if(write(XBEE_portID, serialPacket, _MOCAP_PACKET_LENGTH) > 0) 
          {
              if(verbose){
                printXBeeMsg(xb_msg);  
                printf(" %+4.4lf |", ntp_offset);
                printf(" %+03.6lf |", fRoll);
                printf(" %+03.6lf |", fPitch);
                printf(" %+03.6lf |", fYaw);
                fflush(stdout);
              }	    
              // "flush" the data 
              fsync(XBEE_portID);
          }			
          else			
          {
              printf("ERROR in quali_serial_relay. Could not write errno = %d \n",errno);
          }

          // Write to file
          if(logging){	
            log_xbee_msg_quali(fpblah, xb_msg, time32_us, time64_us, ntp_offset, fRoll, fPitch, fYaw);
          }
          break;

        default:
          break;
      }//switch(eType)
    } //if(received packet)
  }//while(keep_running)

  poRTProtocol.StreamFramesStop();
  poRTProtocol.Disconnect(); // Disconnect from the server

  //Read NTP offset one last time before exiting program
  if(useNTP)
  {
    double old_ntp_offset = ntp_offset;
    ntp_offset = get_ntp_offset();  

    // One last log to get last NTP offset
    if(logging)
    {
      log_xbee_msg_quali(fpblah, xb_msg, time32_us, time64_us, ntp_offset,
                        fRoll, fPitch, fYaw);
    }
    //Print change in NTP offset if we're in verbose mode    
    if(verbose)
    {
      printf("\nNTP Offset from %4.4lf to %4.4lf\n", old_ntp_offset, ntp_offset);
      fflush(stdout);
    }
  }

  // Cleanup options now that we've parsed everything we need
  getopt_destroy(gopt);
  fclose(fpblah);
  return 0;
}


// Initialize data log file
void initLogging(FILE **fpinput) {
    *fpinput = fopen("qualisys_logfile.csv", "w");
    FILE *fp = *fpinput;
    printf("file open.\n");
    // Data
    fprintf(fp, "u_Time, "); 
    fprintf(fp, "x, ");
    fprintf(fp, "y, ");
    fprintf(fp, "z, ");
    fprintf(fp, "qx, ");
    fprintf(fp, "qy, ");
    fprintf(fp, "qz, ");
    fprintf(fp, "qw, ");
    fprintf(fp, "tracking_valid,");
    fprintf(fp, "time_Since_Epoch_us, ");
    fprintf(fp, "ntpOffset_ms, ");
    fprintf(fp, "roll_deg, ");
    fprintf(fp, "pitch_deg, ");
    fprintf(fp, "yaw_deg, ");
    
    // End of Line
    fprintf(fp,"\n");
    fflush(fp);
}

Euler_Rotation_Sequence select_euler_rotation_sequence(const char * first_axis, const char * second_axis, const char * third_axis)
{
    if(      !strcmp("Yaw", first_axis)   &&  !strcmp("Roll", second_axis)   && !strcmp("Yaw", third_axis))
    {
        return yaw_roll_yaw;
    }
    else if( !strcmp("Roll", first_axis)  &&  !strcmp("Pitch", second_axis)  && !strcmp("Roll", third_axis))
    {
        return roll_pitch_roll;
    }
    else if( !strcmp("Pitch", first_axis) &&  !strcmp("Yaw", second_axis)    && !strcmp("Pitch", third_axis))
    {
        return pitch_yaw_pitch;
    }
    else if( !strcmp("Yaw", first_axis)   &&  !strcmp("Pitch", second_axis)  && !strcmp("Yaw", third_axis))
    {
        return yaw_pitch_yaw;
    }
    else if( !strcmp("Roll", first_axis)  &&  !strcmp("Yaw", second_axis)    && !strcmp("Roll", third_axis))
    {
        return roll_yaw_roll;
    }
    else if( !strcmp("Pitch", first_axis) &&  !strcmp("Roll", second_axis)   && !strcmp("Pitch", third_axis))
    {
        return pitch_roll_pitch;
    }
    else if( !strcmp("Roll", first_axis)  &&  !strcmp("Pitch", second_axis)  && !strcmp("Yaw", third_axis))
    {
        return roll_pitch_yaw;
    }
    else if( !strcmp("Pitch", first_axis) &&  !strcmp("Yaw", second_axis)    && !strcmp("Roll", third_axis))
    {
        return pitch_yaw_roll;
    }
    else if( !strcmp("Yaw", first_axis)   &&  !strcmp("Roll", second_axis)   && !strcmp("Pitch", third_axis))
    {
        return yaw_roll_pitch;
    }
    else if( !strcmp("Roll", first_axis)  &&  !strcmp("Yaw", second_axis)    && !strcmp("Pitch", third_axis))
    {
        return roll_yaw_pitch;
    }
    else if( !strcmp("Yaw", first_axis)   &&  !strcmp("Pitch", second_axis)  && !strcmp("Roll", third_axis))
    {
        return yaw_pitch_roll;
    }
    else if( !strcmp("Pitch", first_axis)    &&  !strcmp("Roll", second_axis)    && !strcmp("Yaw", third_axis) )
    {
        return pitch_roll_yaw;
    }

    return invalid_rotation_sequence;
} 

bool arrange_euler_angles(Euler_Rotation_Sequence seq, 
                         float &fRoll, float &fPitch, float &fYaw, 
                         float fAng1, float fAng2, float fAng3)
{
    switch(seq)
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
