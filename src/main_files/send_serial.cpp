#include <serial.h>
#include <string.h>  //string (strlen)
#include <unistd.h>  // read / write
#include <iostream>

#include <errno.h>  //Errors for read/write
#include <sys/time.h>

using std::cout;
using std::endl;

int64_t utime_now(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

int main(int argc, char** argv) {
  cout << "This program compiles and prints!" << endl;

  if (argc != 4) {
    cout << "Usage: " << endl;
    cout << " ./sendSerial <Serial Port> <Baud Rate> <Message> " << endl
         << endl;

    cout << " <Serial Port> = /dev/ttyUSB0, etc..." << endl;
    cout << " <Baud Rate> = 9600,115200, etc..." << endl;

    return 1;
  }

  int fd = serial_open(argv[1], std::stoi(argv[2]), 1);  // blocking == 1 now,

  if (fd == -1) {
    cout << "Failed to open Serial Port" << endl;
    return 1;
  }

  while (1) {
    // returns # of bytes written on success
    if (write(fd, argv[3], strlen(argv[3])) > 0) {
      // "flush" the data
      fsync(fd);
      // cout << "Successfully wrote! " << endl;
    }

    // returns -1 on failure
    else {
      cout << "Error" << endl;
      cout << errno << endl;
      return 1;
    }
  }

  return 0;
}
