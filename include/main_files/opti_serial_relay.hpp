#ifndef __OPTI_SERIAL_RELAY__
#define __OPTI_SERIAL_RELAY__

#include <getopt.h>
#include <optitrack_channels.h>
#include <serial.h>
#include <timestamp.h>
#include <optitrack.hpp>

#include <vector>

#include <errno.h>  //Errors for read/write
#include <stdio.h>
#include <unistd.h>  // read / write / sleep

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <cmath>
#include <fstream>  // std::ifstream
#include <iostream>

// Below for PRId64
#include <inttypes.h>
#include <cinttypes>

#include <stdint.h>

/// Helper Functions
#include <Quaternion.h>
#include <crc16.h>
#include <nonBlockingCLI.h>
#include <simpleMatrixStuff.h>
#include <printing.hpp>
#include <xbee_packet.hpp>

#define weightsFileName "weights.txt"

std::vector<float> computeWeightsHelper(xbee_packet_t &x1, xbee_packet_t &x2,
                                        xbee_packet_t &x3, xbee_packet_t &xn) {
  // Set Up Matrix Variable
  std::vector<std::vector<float>> mat;
  for (int i = 0; i < 3; i++) {
    mat.push_back(std::vector<float>());
    for (int j = 0; j < 3; j++) {
      mat[i].push_back(0);
    }
  }

  // 2D Homogenous Coordinates
  mat[0][0] = x1.x;
  mat[0][1] = x2.x;
  mat[0][2] = x3.x;
  mat[1][0] = x1.y;
  mat[1][1] = x2.y;
  mat[1][2] = x3.y;
  mat[2][0] = 1;
  mat[2][1] = 1;
  mat[2][2] = 1;

  /*
  // Print out Matrix mat
  printf("mat = \n");
  printf("[%7.6f, %7.6f, %7.6f]\n",mat[0][0],mat[0][1],mat[0][2]);
  printf("[%7.6f, %7.6f, %7.6f]\n",mat[1][0],mat[1][1],mat[1][2]);
  printf("[%7.6f, %7.6f, %7.6f]\n",mat[2][0],mat[2][1],mat[2][2]);
  */
  std::vector<std::vector<float>> i = matrixInverse_3x3(mat);

  /*
  // Print out Matrix i
  printf("\ni = \n");
  printf("[%7.6f, %7.6f, %7.6f]\n",i[0][0],i[0][1],i[0][2]);
  printf("[%7.6f, %7.6f, %7.6f]\n",i[1][0],i[1][1],i[1][2]);
  printf("[%7.6f, %7.6f, %7.6f]\n",i[2][0],i[2][1],i[2][2]);

  // Print out position vector
  printf("\nx = \n");
  printf("[%7.6f]",xn.x);
  printf("[%7.6f]",xn.y);
  printf("[1]");
  */

  std::vector<float> w(4, 0.33);

  w[1] = i[0][0] * xn.x + i[0][1] * xn.y + i[0][2];
  w[2] = i[1][0] * xn.x + i[1][1] * xn.y + i[1][2];
  w[3] = i[2][0] * xn.x + i[2][1] * xn.y + i[2][2];

  /*
  // Print out weight vector
  printf("\nw = \n");
  printf("[%7.6f]",w[1]);
  printf("[%7.6f]",w[2]);
  printf("[%7.6f]",w[3]);
  */
  return w;
}

std::vector<float> computeWeights(std::vector<xbee_packet_t> xb_msg,
                                  int quadNum) {
  std::vector<float> wLocal;
  std::vector<float> w(xb_msg.size(), 0);

  // 1 follower case
  if (xb_msg.size() == 5) {
    wLocal = computeWeightsHelper(xb_msg[1], xb_msg[2], xb_msg[3], xb_msg[4]);
    w[1] = wLocal[1];
    w[2] = wLocal[2];
    w[3] = wLocal[3];
    return w;
  }

  // 2 followers (hardcoded topology for now)
  if (quadNum == 4) {
    wLocal = computeWeightsHelper(xb_msg[1], xb_msg[3], xb_msg[5], xb_msg[4]);
    w[1] = wLocal[1];
    w[3] = wLocal[2];
    w[5] = wLocal[3];
    return w;
  }
  if (quadNum == 5) {
    wLocal = computeWeightsHelper(xb_msg[2], xb_msg[3], xb_msg[4], xb_msg[5]);
    w[2] = wLocal[1];
    w[3] = wLocal[2];
    w[4] = wLocal[3];
    return w;
  }

  return w;
}

void loadWeightsFromFile(std::vector<std::vector<float>> &w) {
  std::ifstream inFile(weightsFileName);
  std::string line, val;
  std::string::size_type sz;  // alias of size_t
  int currNum = 0;

  // Read in the data
  if (inFile.is_open()) {
    while (getline(inFile, line)) {
      currNum++;

      w[currNum][1] = std::stof(line, &sz);
      line = line.substr(sz);
      w[currNum][2] = std::stof(line, &sz);
      line = line.substr(sz);
      w[currNum][3] = std::stof(line, &sz);
      line = line.substr(sz);
      w[currNum][4] = std::stof(line, &sz);
      line = line.substr(sz);
      w[currNum][5] = std::stof(line, &sz);
    }
    inFile.close();
  } else {
    std::cout << "Unable to open file" << std::endl;
  }

  std::cout << "Weights Read From File:" << std::endl;
  for (int i = 1; i < 6; i++) {
    for (int j = 1; j < 6; j++) {
      std::cout << w[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  return;
}

#endif  //__OPTI_SERIAL_RELAY__