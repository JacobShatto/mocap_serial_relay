#include <serial.h>
#include <printing.hpp>

void initLogging(FILE *fp) {
  fp = fopen("optitrack_logfile.csv", "w");

  // Data
  fprintf(fp, "u_Time, ");
  fprintf(fp, "x, ");
  fprintf(fp, "y, ");
  fprintf(fp, "z, ");
  fprintf(fp, "qx, ");
  fprintf(fp, "qy, ");
  fprintf(fp, "qz, ");
  fprintf(fp, "qw, ");
  fprintf(fp, "state, ");
  // fprintf(fp, "x_d, ");
  // fprintf(fp, "y_d, ");
  // fprintf(fp, "z_d, ");

  // End of Line
  fprintf(fp, "\n");
}

void initLoggingSimpleOptirelay(FILE *fp, int numQuads) {
  fp = fopen("optitrack_logfile.csv", "w");

  // Data
  for (int i = 1; i < numQuads + 1; i++) {
    fprintf(fp, "Quad %d, ", i);
    fprintf(fp, "u_Time, ");
    fprintf(fp, "x, ");
    fprintf(fp, "y, ");
    fprintf(fp, "z, ");
    fprintf(fp, "qx, ");
    fprintf(fp, "qy, ");
    fprintf(fp, "qz, ");
    fprintf(fp, "qw, ");
    fprintf(fp, "trackingValid, ");
  }

  // End of Line
  fprintf(fp, "\n");
}

FILE *initLoggingXBeeGCS(int numQuads, bool usingWindVane) {
  FILE *fp;

  fp = fopen("optitrack_logfile.csv", "w");

  // Data
  for (int i = 1; i < numQuads + 1; i++) {
    fprintf(fp, "Quad %d, ", i);
    fprintf(fp, "u_Time, ");
    fprintf(fp, "x, ");
    fprintf(fp, "y, ");
    fprintf(fp, "z, ");
    fprintf(fp, "qx, ");
    fprintf(fp, "qy, ");
    fprintf(fp, "qz, ");
    fprintf(fp, "qw, ");
    fprintf(fp, "trackingValid, ");
    fprintf(fp, "state, ");
    // fprintf(fp, "x_d, ");
    // fprintf(fp, "y_d, ");
    // fprintf(fp, "z_d, ");
    fprintf(fp, "w%d[1], ", i);
    fprintf(fp, "w%d[2], ", i);
    fprintf(fp, "w%d[3], ", i);
  }

  if (usingWindVane) {
    fprintf(fp, "windByte, ");
  }

  // End of Line
  // fprintf(fp, "\n");

  return fp;
}

// Print CLI for States
void printCLIforStates() {
  printf("Switch States Using Keyboard Input as Follows: \n");
  printf("    0: 's' STANDBY \n");
  printf("    1: 't' TAKEOFF \n");
  printf("    2: 'g' GUIDANCE \n");
  printf("    3: 'l' LAND \n");
  printf("    4: 'p' PAUSE \n");

  printf("\nControl Position Setpoint for Follower Mode as Follows: \n");
  printf(" @: Z++ \n");
  printf(" X: Z-- \n");
  printf(" W: X++ \n");
  printf(" A: Y-- \n");
  printf(" S: X-- \n");
  printf(" D: Y++ \n");
  printf(" O: X=Y=Z=0 \n");
}

void printCLIHeaders_simple(int numQuads) {
  printf("                  | Valid Rigid Body Tracking ");

  printf("\n");
  printf("   Time   |");
  printf(" State |");

  for (int i = 1; i < numQuads + 1; i++) {
    printf(" Q%d |", i);
  }
  printf("\n");
}

// Printf Headers
void printCLIHeaders(int numQuads) {
  printf("                  | Valid Rigid Body Tracking ");

  printf("\n");
  printf("    Time   |");
  printf(" State |");
  printf(" Tracking Valid|");
  printf("   x   |");
  printf("   y   |");
  printf("   z   |");
  printf("  qw   |");
  printf("  qx   |");
  printf("  qy   |");
  printf("  qz   |");

  // // Desired Position
  // if (numQuads > 1) {
  //   printf(" x_d-Q%d  |", numQuads - 1);
  //   printf(" y_d-Q%d  |", numQuads - 1);
  //   printf(" z_d-Q%d  |", numQuads - 1);
  // }

  // printf(" x_d-Q%d  |", numQuads);
  // printf(" y_d-Q%d  |", numQuads);
  // printf(" z_d-Q%d  |", numQuads);

  // for (int i = 1; i < numQuads + 1; i++) {
  //   printf(" Q%d |", i);
  // }
  // printf("\n");
}

void logXBeeMsgs(FILE *fp, std::vector<xbee_packet_t> xb_msg, int numQuads,
                 std::vector<std::vector<float>> w, int fd_w,
                 bool usingWindVane, char &wind) {
  for (int i = 1; i < numQuads + 1; i++) {
    fprintf(fp, " , ");
    fprintf(fp, "%d, ", xb_msg[i].time);
    fprintf(fp, "%7.6f, ", xb_msg[i].x);
    fprintf(fp, "%7.6f, ", xb_msg[i].y);
    fprintf(fp, "%7.6f, ", xb_msg[i].z);
    fprintf(fp, "%7.6f, ", xb_msg[i].qx);
    fprintf(fp, "%7.6f, ", xb_msg[i].qy);
    fprintf(fp, "%7.6f, ", xb_msg[i].qz);
    fprintf(fp, "%7.6f, ", xb_msg[i].qw);
    fprintf(fp, "%d, ", xb_msg[i].trackingValid);
    fprintf(fp, "%d, ", xb_msg[i].state);
    // fprintf(fp, "%7.6f, ", xb_msg[i].x_d);
    // fprintf(fp, "%7.6f, ", xb_msg[i].y_d);
    // fprintf(fp, "%7.6f, ", xb_msg[i].z_d);
    fprintf(fp, "%7.6f, ", w[i][1]);
    fprintf(fp, "%7.6f, ", w[i][2]);
    fprintf(fp, "%7.6f, ", w[i][3]);
  }

  char currWind = 0xFF;

  if (usingWindVane) {
    if (read(fd_w, &wind, 1) > 0) {
      currWind = wind;
    }
    fprintf(fp, "%d, ", currWind);
  }

  // End of Line
  // fprintf(fp, "\n");
}

void log_xbee_msg_quali(FILE *fp, xbee_packet_t &xb_msg, 
                        uint32_t time32_us, uint64_t time64_us, double ntp_offset, 
                        float roll, float pitch, float yaw)
{
  fprintf(fp,"%lu, ", time32_us);
  fprintf(fp,"%7.6f, ",xb_msg.x);
  fprintf(fp,"%7.6f, ",xb_msg.y);
  fprintf(fp,"%7.6f, ",xb_msg.z);
  fprintf(fp,"%7.6f, ",xb_msg.qx);
  fprintf(fp,"%7.6f, ",xb_msg.qy);
  fprintf(fp,"%7.6f, ",xb_msg.qz);
  fprintf(fp,"%7.6f, ",xb_msg.qw);
  fprintf(fp,"%u, ",xb_msg.trackingValid);
  fprintf(fp,"%llu, ", time64_us);
  fprintf(fp,"%lf, ",ntp_offset);
  fprintf(fp,"%f, ",roll);
  fprintf(fp,"%f, ",pitch);
  fprintf(fp,"%f, ",yaw);
  fprintf(fp,"\n");
}
