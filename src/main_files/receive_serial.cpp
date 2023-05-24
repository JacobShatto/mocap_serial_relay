#include <serial.h>
#include <unistd.h>  // read / write
#include <iostream>

using std::cout;
using std::endl;

int main(int argc, char** argv) {
  unsigned char c = 'D';

  cout << "This program compiles and prints!" << endl;

  if (argc != 3) {
    cout << "Usage: " << endl;
    cout << " ./receiveSerial <Serial Port> <Baud Rate> " << endl << endl;

    cout << " <Serial Port> = /dev/ttyUSB0, etc..." << endl;
    cout << " <Baud Rate> = 9600,115200, etc..." << endl;

    return 1;
  }

  int fd = serial_open(argv[1], std::stoi(argv[2]), 0);  // blocking == 1 now,

  if (fd == -1) {
    cout << "Failed to open Serial Port" << endl;
    return 1;
  }

  while (1) {
    // Try to Read 1 Byte
    if (read(fd, &c, 1) > 0) {
      cout << c << endl;
    }
  }

  return 0;
}
