// Updated code (Feb 2019):  Use more robust serial message structure with Fletcher-16

#include "qualisys_driver.h"
void initLogging(FILE **fpinput);
unsigned int bodyID = 0;  // Rigid body ID from Qualisys (hard coded for this version)

int main(int argc, char** argv)
{
  const char* kInterfaceArg = "interface";
  const char* kSerialBaudArg = "baudrate";
  const char* kVerbose = "verbose";
  const char* kLogging = "logging";    
  const char* kTransform = "transform";
  const char* kSerialPort = "serialPort";
  const char* kTestingFakeData  = "TestingFakeData";
  const char* kXbeeAddrArg = "xbeeAddr";
  
  getopt_t* gopt = getopt_create();
  getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
  getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.254.1", "Local network interface for connecting to Optitrack network"); 
  getopt_add_int(gopt, 'b', kSerialBaudArg, "57600", "Serial baudrate for communication via XBee");
  getopt_add_bool(gopt,'v',kVerbose, 0, "Print to terminal the Optitrack data");
  getopt_add_bool(gopt,'l',kLogging, 0, "Save data to logfile");
  getopt_add_bool(gopt,'t',kTransform, 0, "Transform data from Y-Up to NED Frame");
  getopt_add_string(gopt, 's',kSerialPort, "/dev/ttyUSB0", "Serial port used to send the XBee packets out");  
  getopt_add_bool(gopt, 'T',kTestingFakeData, 0, "Send fake, hardcoded data instead of optitrack for testing");  
  getopt_add_int(gopt,'x',kXbeeAddrArg, "1", "Address of target XBee");
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
  bool transform = getopt_get_bool(gopt, kTransform);
  std::string serialPort = getopt_get_string(gopt, kSerialPort);
  bool testingFakeData = getopt_get_bool(gopt, kTestingFakeData);
  int xbeeAddr = getopt_get_int(gopt, kXbeeAddrArg);
  
  ////////////////////
  // XBee Serial Port Variables
  int XBEE_portID;  
  xbee_packet_t xb_msg;
  char dataPacket[_OPTI_PACKET_LENGTH];
  dataPacket[0] = 0x81;  dataPacket[1] = 0xA1;  // Two start bytes
  unsigned char ck0=0, ck1=0;
  
  // Open a serial port
  //XBEE_portID = serial_open("/dev/ttyUSB0",baudRate,1); 	// blocking while sending
  printf("serial = %s\n", serialPort.c_str());
  XBEE_portID = serial_open(serialPort.c_str(),baudRate,1); 	// blocking while sending
  if(XBEE_portID == -1)  {
    printf("Failed to open Serial Port");
    return 1;
  }
  
  // Configure XBee Destination Address
  printf("Programming XBee...\n");
  char config_mode[] = "+++";
  char dest_addr_read[] = "ATDL\r\n";
  char dest_addr_cmd[80];
  sprintf(dest_addr_cmd, "ATDL %d\r\n", xbeeAddr);
  char cmd_null[] = "ATCN\r\n";
  char xbee_resp[] = "OK";
  char resp[10];
  write(XBEE_portID,config_mode,sizeof(config_mode)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Received Incorrect Response\n");
  write(XBEE_portID,dest_addr_cmd,sizeof(dest_addr_cmd)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Received Incorrect Response\n");
  write(XBEE_portID,dest_addr_read,sizeof(dest_addr_read)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  char dest_addr[10];
  sprintf(dest_addr,"%d",xbeeAddr);
  if(!strcmp(resp,dest_addr))
    printf("Received Incorrect Response\n");  
  write(XBEE_portID,cmd_null,sizeof(cmd_null)-1);
  usleep(1E5);
  read(XBEE_portID,resp,sizeof(resp));
  if(!strcmp(resp,xbee_resp))
    printf("Recieved Incorrect Response\n");
  printf("RESP:%s\n",resp);
  
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
    printf("  Wind    |");
    
    printf("\n");
  }
  	
  // Writing to Logfile
  FILE *fpblah; 
  if(logging) 
    initLogging(&fpblah);

  // Grab Initial Time
  int64_t init_time64_u = utime_now();
  int64_t time64_u = utime_now();
  uint32_t time_u = (uint32_t) (time64_u - init_time64_u);
  
  // Wind Vane Variables
  bool usingWindVane = false;
  char wind;
  int fd_w; 
  fd_w = serial_open("/dev/ttyO4",9600,0); 	// non-blocking reads
  if(fd_w == -1) {
     usingWindVane = false;
  }
  
  
  //                      Testing Code (Fake Data)                           
  if (testingFakeData) {
    // Send data at "60 Hz" = 16.67 ms =  16,670 us
    // Send data at "10 Hz" = 100 ms = 100,000 us
    unsigned int microseconds = 16670;
    float testZero = 0;
    
    while (1) {
      usleep(microseconds);              
      time64_u = utime_now();
      time_u = (uint32_t) (time64_u - init_time64_u);
      
      // Construct XBee Packet
      xb_msg.time = time_u;
      xb_msg.x = testZero;
      xb_msg.y = testZero;
      xb_msg.z = testZero;
      xb_msg.qx = testZero;
      xb_msg.qy = testZero;
      xb_msg.qz = testZero;
      xb_msg.qw = testZero;
      xb_msg.trackingValid = 1;

      // Construct Serial Message
      // char* ptr = dataPacket;
      memcpy(dataPacket+2, &xb_msg, _OPTI_DATA_LENGTH);
      ck0=0; ck1=0;  // Fletcher-16 checksum
      for (int i=0; i < (int)  _OPTI_DATA_LENGTH; i++) {
	ck0 += dataPacket[i+2];
	ck1 += ck0;
      }
      dataPacket[_OPTI_DATA_LENGTH + 2] = ck0;
      dataPacket[_OPTI_DATA_LENGTH + 3] = ck1;
      
      // send serial message	(returns # of bytes sent on success)
      if(write(XBEE_portID,dataPacket,_OPTI_PACKET_LENGTH) > 0) {
	if(verbose)	    printXBeeMsg(xb_msg);  
	// "flush" the data 
	fsync(XBEE_portID);
      }			else			
	printf("Error: %d \n",errno);			
    }
  }

  // Qualisys code

  CRTProtocol poRTProtocol;   
  printf("Trying to connect to Qualisys Network at %s:%d... \n",(char*)interface.data(),QTM_RT_SERVER_BASE_PORT);

  if (poRTProtocol.Connect((char*)interface.data(), QTM_RT_SERVER_BASE_PORT, 0, 1, 7))    {
    getQTMSettings(poRTProtocol);
    poRTProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL, CRTProtocol::Component6dEuler);
    CRTPacket::EPacketType eType;
    bool                   bKeyAbort  = false;
    unsigned int           nCount;
    CRTPacket*             pRTPacket;
    float                  fX, fY, fZ, fAng1, fAng2, fAng3;
    
    // Set Terminal for non-blocking        
    // set_conio_terminal_mode();
    
    while (1) {
      // Grab Time
      time64_u = utime_now();
      time_u = (uint32_t) (time64_u - init_time64_u);

      if (poRTProtocol.ReceiveRTPacket(eType, true))      {
	switch (eType) {
	case CRTPacket::PacketError : // sHeader.nType 0 indicates an error
	  fprintf(stderr, "Error when streaming frames: %s\n", poRTProtocol.GetRTPacket()->GetErrorString());
	  break;
	case CRTPacket::PacketData:         // Data received
	  pRTPacket = poRTProtocol.GetRTPacket();
	  nCount  = pRTPacket->Get6DOFEulerBodyCount();
	  if (nCount > 0)  {
	    //  HARDCODED bodyID to 1
	    pRTPacket->Get6DOFEulerBody(bodyID, fX, fY, fZ, fAng1, fAng2, fAng3);
	    if(std::isnan(fX) ||  std::isnan(fY) ||  std::isnan(fZ) ||  
	       std::isnan(fAng1) ||  std::isnan(fAng2) ||  std::isnan(fAng3))
	      continue;

	    /*if (debugging) {
	      printf("X=    %10.4f  Y=     %10.4f  Z=   %10.4f\n", fX, fY, fZ);
	      printf("                  Roll= %10.4f  Pitch= %10.4f  Yaw= %10.4f\n\n", fAng1, fAng2, fAng3);
	      } */
	    // Convert Euler to Quaternion
	    float deg2rad = 3.14159265 / 180;
	    Quat quat = EulerToQuat(fAng1*deg2rad, fAng2*deg2rad, fAng3*deg2rad);
	    
	    // Construct XBee Packet
	    xb_msg.time = time_u;
	    xb_msg.x = fX;
	    xb_msg.y = fY;
	    xb_msg.z = fZ;
	    xb_msg.qx = quat.w;
	    xb_msg.qy = quat.x;
	    xb_msg.qz = quat.y;
	    xb_msg.qw = quat.z;
	    xb_msg.trackingValid = 1; // Hardcoded to 1 for Qualisys for now; (uint32_t) msg.trackingValid;
	    
	    // Construct Serial Message
	    // char* ptr = dataPacket;
	    memcpy(dataPacket+2, &xb_msg, _OPTI_DATA_LENGTH);
	    ck0=0; ck1=0;  // Fletcher-16 checksum
	    for (int i=0; i < (int) _OPTI_DATA_LENGTH; i++) {
	      ck0 += dataPacket[i+2];
	      ck1 += ck0;
	    }
	    dataPacket[_OPTI_DATA_LENGTH + 2] = ck0;
	    dataPacket[_OPTI_DATA_LENGTH + 3] = ck1;
	    
	    // send serial message	(returns # of bytes sent on success)
	    if(write(XBEE_portID,dataPacket,_OPTI_PACKET_LENGTH) > 0) {
	      if(verbose)	    printXBeeMsg(xb_msg);  
	      // "flush" the data 
	      fsync(XBEE_portID);
	    }			else			
	      printf("Error: %d \n",errno);			
	    
      char currWind;
      if(read(fd_w,&wind,1)>0)
          currWind = wind;
	    
      // Write to file
	    if(logging)		{
	      fprintf(fpblah,"%u, ",time_u);			
	      fprintf(fpblah,"%7.6f, ",xb_msg.x);
	      fprintf(fpblah,"%7.6f, ",xb_msg.y);
	      fprintf(fpblah,"%7.6f, ",xb_msg.z);
	      fprintf(fpblah,"%7.6f, ",xb_msg.qx);
	      fprintf(fpblah,"%7.6f, ",xb_msg.qy);
	      fprintf(fpblah,"%7.6f, ",xb_msg.qz);
	      fprintf(fpblah,"%7.6f, ",xb_msg.qw);
	      fprintf(fpblah,"%u,  ",xb_msg.trackingValid);
	      fprintf(fd_w,"%c, ",currWind);
        fprintf(fpblah,"\n");
	    }
	  }
	}
      }
    }
  
    poRTProtocol.StreamFramesStop();
    poRTProtocol.Disconnect(); // Disconnect from the server
  }

  // Cleanup options now that we've parsed everything we need
  getopt_destroy(gopt);
  fclose(fpblah);
  return 0;
}


// Initialize data log file
void initLogging(FILE **fpinput) {
    *fpinput = fopen("optitrack_logfile.csv", "w");
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
    fprintf(fp, "WindByte,");
    
    // End of Line
    fprintf(fp,"\n");
    fflush(fp);
}

