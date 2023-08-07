#ifndef SERIAL_H_
#define SERIAL_H_

// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


#define PORT "/dev/ttyUSB0"
#define BAUDRATE 921600
#define TIMEOUT 1


class Serial
{
private:
  bool is_initialized = false;
  bool is_open = false;
  struct termios tty;
  uint8_t timeout;
  char read_buf [256]; // allocate memory for read buffer
  char write_buf [255];

  void set_termios();


public:
  Serial();

  Serial(const char* port_, uint8_t timeout_);
 
  ~Serial();

  void set_baudrate(unsigned int baudrate);
  int isOpen();
  void write_(std::string msg);
  void write_(unsigned int msg[]);

  void read_();

  void close_();

  int serial;
};




#endif // SERIAL_H_