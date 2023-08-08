

#include "serial_test.h"

MotorSerialCom::MotorSerialCom()
{   
    serial_port_ = DEFAULT_PORT;
    baudrate_ = DEFAULT_BAUDRATE;
    serial = open(serial_port_, O_RDWR);
    if (serial < 0)
    {
        printf("[MotorSerialCom][Error] Error while opening device: %s\n", DEFAULT_PORT);
        exit(0);
    }

    is_serial_initialized = true;
    set_termios();
    set_baudrate(DEFAULT_BAUDRATE);
    printf("[MotorSerialCom][Info] Serial port %s is successfully initialized. \n", DEFAULT_PORT);
}

MotorSerialCom::MotorSerialCom(const char* port, unsigned int baudrate, uint8_t timeout)
{
    serial_port_ = port;
    baudrate_ = baudrate;
    serial = open(serial_port_, O_RDWR);
    if(serial<0)
    {
        printf("[MotorSerialCom][Error] Error while opening device: %s\n", serial_port_);
        exit(0);
    }
    is_serial_initialized = true;
    set_termios();
    set_baudrate(baudrate_);
    printf("[MotorSerialCom][Info] Serial port %s is successfully initialized. \n", serial_port_);
}

MotorSerialCom::~MotorSerialCom()
{
  close_();
}


void MotorSerialCom::set_termios()
{
    // Read in existing settings, and handle any error
    if(tcgetattr(serial, &tty) != 0){
      printf("[MotorSerialCom][Error] Error %i from tcgetattr: %s\n", errno, strerror(errno));
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

void MotorSerialCom::set_baudrate(unsigned int baudrate)
{
    // Set in/out baud rate
    if (baudrate==0){baudrate_=B0;}
    else if (baudrate==50){baudrate_ = B50;}
    else if (baudrate==75){baudrate_ = B75;}
    else if (baudrate==110){baudrate_ = B110;}
    else if (baudrate==134){baudrate_ = B134;}
    else if (baudrate==150){baudrate_ = B150;}
    else if (baudrate==200){baudrate_ = B200;}
    else if (baudrate==300){baudrate_ = B300;}
    else if (baudrate==600){baudrate_ = B600;}
    else if (baudrate==1200){baudrate_ = B1200;}
    else if (baudrate==1800){baudrate_ = B1800;}
    else if (baudrate==2400){baudrate_ = B2400;}
    else if (baudrate==4800){baudrate_ = B4800;}
    else if (baudrate==9600){baudrate_ = B9600;}
    else if (baudrate==19200){baudrate_ = B19200;}
    else if (baudrate==38400){baudrate_ = B38400;}
    else if (baudrate==57600){baudrate_ = B57600;}
    else if (baudrate==115200){baudrate_ = B115200;}
    else if (baudrate==230400){baudrate_ = B230400;}
    else if (baudrate==460800){baudrate_ = B460800;}
    else if (baudrate==500000){baudrate_ = B500000;}
    else if (baudrate==576000){baudrate_ = B576000;}
    else if (baudrate==921600){baudrate_ = B921600;}
    else if (baudrate==1000000){baudrate_ = B1000000;}
    else if (baudrate==1152000){baudrate_ = B1152000;}
    else if (baudrate==1500000){baudrate_ = B1500000;}
    else if (baudrate==2000000){baudrate_ = B2000000;}
    else if (baudrate==2500000){baudrate_ = B2500000;}
    else if (baudrate==3000000){baudrate_ = B3000000;}
    else if (baudrate==3500000){baudrate_ = B3500000;}
    else if (baudrate==4000000){baudrate_ = B4000000;}
    else{
      printf("[MotorSerialCom][Error] %i baudrate is not valid\n", baudrate);
      exit(0);
    }
    cfsetispeed(&tty, baudrate_);
    cfsetospeed(&tty, baudrate_);
    // save termios settings and check for error
    if (tcsetattr(serial, TCSANOW, &tty) != 0) {
      printf("[MotorSerialCom][Error] Error while configuring termios.\n");
      printf("[MotorSerialCom][Error] Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(0);
    }
    is_serial_open = true;
    printf("[MotorSerialCom][Info] Serial baudrate is set to: %i.\n", baudrate);
}

bool MotorSerialCom::isOpen()
{
    return is_serial_open;
}

// Serial write a string message
// return: Num of written bytes, or -1 if Error
long int MotorSerialCom::write_string(std::string* msg)
{
    if (isOpen()){
        long int sent_bytes = write(serial, msg->c_str(), msg->size());
        if(sent_bytes<0){
            printf("[MotorSerialCom][Error] Failed to send serial message: %s\n", msg->c_str());
        }
        else if(debug){
            printf("[MotorSerialCom][Debug] Serial sent %li bytes of message: %s\n", sent_bytes, msg->c_str());
        }
        return sent_bytes;
    }
    else{
        printf("[MotorSerialCom][Error] Serial port is not opened.\n");
        return -1;
    }
    
}

// Serial write an array of uint8_t data
// return: Num of written bytes, or -1 if Error
long int MotorSerialCom::write_bytearray(uint8_t* bytearray, uint8_t size)
{
    if(isOpen()){
        long int sent_bytes = write(serial, bytearray, size);
        if(sent_bytes<0){
            printf("[MotorSerialCom][Error] Failed to send serial data: |");
            for(int i=0; i<size; i++){
                printf(" %i |", *(bytearray+i));
            }
            printf("\n");
        }
        else if(debug){
            printf("[MotorSerialCom][Debug] Serial sent %i bytes of data: |", size);
            for(int i=0; i<size; i++){
                printf(" %i |", *(bytearray+i));
            }
            printf("\n");
        }
        return sent_bytes;
    }
    else{
        printf("[MotorSerialCom][Error] Serial port is not opened.\n");
        return -1;
    }
}
    
    

// Read bytes. The behaviour of read() 
// e.g.:
// - does it block?,
// - how long does it block for?
//  depends on the termios configuration settings, specifically VMIN and VTIME
// void Serial::read_()
// {
//   memset(&read_buf, '\0', sizeof(read_buf));
// }



int MotorSerialCom::close_()
{
    int closed = close(serial);
    if(closed<0){
      printf("[MotorSerialCom][Error] Error while closing the serial port\n");
    }
    else{
      is_serial_open = false;
      printf("[MotorSerialCom][Info] Serial port %s is closed.\n", serial_port_);
    }
    return closed;
}


int main(int argc, char** argv)
{
  MotorSerialCom serial{"/dev/ttyUSB0", 115200, 1};

  

  uint8_t tx[9];
  tx[0] = 1;
  for(int i=1; i<8; i++)
  {
    tx[i] = 0xFF;
  }
  tx[8] = 0xFD;
  printf("tx size: %li\n", sizeof(tx));

  serial.write_bytearray(tx, sizeof(tx));

  // printf("%i", serial.debug);

  return 0;
}

