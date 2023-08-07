// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "serial_test.h"

#define PORT "/dev/ttyUSB0"
#define BAUDRATE 921600
#define TIMEOUT 1


Serial::Serial(){}

Serial::Serial(const char* port_, uint8_t timeout_)
{
  serial = open(port_, O_RDWR);
  if (serial < 0){
    printf("Error while opening device: %s\n", port_);
    exit(0);
  }
  timeout = timeout_;
  is_initialized = true;
  set_termios();
}

Serial::~Serial(){}

 
void Serial::set_termios()
{
    // Read in existing settings, and handle any error
    if(tcgetattr(serial, &tty) != 0){
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(0);
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
}

void Serial::set_baudrate(unsigned int baudrate)
{
  // Set in/out baud rate
    unsigned int baud;
    if (baudrate==0){baud=B0;}
    else if (baudrate==50){baud = B50;}
    else if (baudrate==75){baud = B75;}
    else if (baudrate==110){baud = B110;}
    else if (baudrate==134){baud = B134;}
    else if (baudrate==150){baud = B150;}
    else if (baudrate==200){baud = B200;}
    else if (baudrate==300){baud = B300;}
    else if (baudrate==600){baud = B600;}
    else if (baudrate==1200){baud = B1200;}
    else if (baudrate==1800){baud = B1800;}
    else if (baudrate==2400){baud = B2400;}
    else if (baudrate==4800){baud = B4800;}
    else if (baudrate==9600){baud = B9600;}
    else if (baudrate==19200){baud = B19200;}
    else if (baudrate==38400){baud = B38400;}
    else if (baudrate==57600){baud = B57600;}
    else if (baudrate==115200){baud = B115200;}
    else if (baudrate==230400){baud = B230400;}
    else if (baudrate==460800){baud = B460800;}
    else if (baudrate==500000){baud = B500000;}
    else if (baudrate==576000){baud = B576000;}
    else if (baudrate==921600){baud = B921600;}
    else if (baudrate==1000000){baud = B1000000;}
    else if (baudrate==1152000){baud = B1152000;}
    else if (baudrate==1500000){baud = B1500000;}
    else if (baudrate==2000000){baud = B2000000;}
    else if (baudrate==2500000){baud = B2500000;}
    else if (baudrate==3000000){baud = B3000000;}
    else if (baudrate==3500000){baud = B3500000;}
    else if (baudrate==4000000){baud = B4000000;}
    else{
      printf("%i baudrate is not valid\n", baudrate);
      exit(0);
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    // save termios settings and check for error
    if (tcsetattr(serial, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(0);
    }
    is_open = true;
}

int Serial::isOpen()
{
  return is_open;
}

void Serial::write_(std::string msg){
  // strcpy(write_buf, msg);
  write(serial, &msg, sizeof(&msg));
  // TODO: read error

  printf("Serial sent %li bytes of data: \n", sizeof(&msg));
  printf("%hhn", msg);
}

void Serial::write_(unsigned int msg[]){
  write(serial, &msg, sizeof(&msg));
  // TODO: read error

  printf("Serial sent %li bytes of data: \n", sizeof(&msg));
}

// Read bytes. The behaviour of read() 
// e.g.:
// - does it block?,
// - how long does it block for?
//  depends on the termios configuration settings, specifically VMIN and VTIME
void Serial::read_()
{
  memset(&read_buf, '\0', sizeof(read_buf));
}

void Serial::close_()
{
  close(serial);
  // TODO: read error
  is_open = false;
}


int main(int argc, char** argv)
{
  Serial serial{"/dev/ttyUSB0", 1};
  serial.set_baudrate(921600);

  uint8_t tx[3];
  tx[0] = 10;
  tx[1] = 20;
  tx[2] = 30;
  printf("tx size: %li\n", sizeof(tx));

  serial.write_(tx);

  printf("is serial open: %i\n", serial.isOpen());

  return 0;
}