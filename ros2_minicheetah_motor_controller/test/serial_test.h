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
  void write_string(std::string* msg);
  void write_bytearray(uint8_t* msg, uint8_t size);

  void read_();
  void close_();

  int serial;
  bool debug=true;
};




#endif // SERIAL_H_