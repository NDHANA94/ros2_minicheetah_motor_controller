
#include "motor_serial_com.h"

MotorSerialCom::MotorSerialCom()
{
    sprintf(print_msg_buf, "Initializing Serial Port....\n");
    print_info(print_msg_buf);
    serial_port_ = DEFAULT_PORT;
    serial_baudrate_ = DEFAULT_BAUDRATE;
    serial_timeout_ = DEFAULT_TIMEOUT;

    serial = open(serial_port_.c_str(), O_RDWR);
    if (serial < 0)
    {
        sprintf(print_msg_buf, "Failed to oper serial port: %s\n", serial_port_.c_str());
        print_error(print_msg_buf);
        exit(0);
    }
    is_serial_initialized = true;
    set_termios();
    set_baudrate(DEFAULT_BAUDRATE);

    sprintf(print_msg_buf, "Serial port %s is successfully initialized.\n", serial_port_.c_str());
    print_info(print_msg_buf);
}

MotorSerialCom::MotorSerialCom(const char* port, uint32_t baudrate, uint8_t timeout)
{   
    sprintf(print_msg_buf, "Initializing Serial Port....\n");
    print_info(print_msg_buf);
    serial_port_ = port;
    serial_baudrate_ = baudrate;
    serial_timeout_ = timeout;

    serial = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(serial<0)
    {
        sprintf(print_msg_buf, "Failed to oper serial port: %s\n", serial_port_.c_str());
        print_error(print_msg_buf);
        exit(0);
    }
    is_serial_initialized = true;
    set_termios();
    set_baudrate(baudrate);

    sprintf(print_msg_buf, "Serial port %s is successfully initialized.\n", serial_port_.c_str());
    print_info(print_msg_buf);
}

MotorSerialCom::~MotorSerialCom(){
    close_();
}

void MotorSerialCom::set_termios()
{
    // Read in existing settings, and handle any error
    if(tcgetattr(serial, &tty) != 0){
      sprintf(print_msg_buf, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
      print_error(print_msg_buf);
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
      sprintf(print_msg_buf,"%i baudrate is not valid\n", baudrate);
      print_error(print_msg_buf);
      exit(0);
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    // save termios settings and check for error
    if (tcsetattr(serial, TCSANOW, &tty) != 0) {
      sprintf(print_msg_buf,"Error while configuring termios.\n");
      print_error(print_msg_buf);
      sprintf(print_msg_buf,"Error %i from tcsetattr: %s\n", errno, strerror(errno));
      print_error(print_msg_buf);
      exit(0);
    }
    is_serial_open = true;
    sprintf(print_msg_buf,"Serial baudrate is set to %i.\n", serial_baudrate_);
    print_info(print_msg_buf);
}

bool MotorSerialCom::isOpen()
{
    return is_serial_open;
}

// Serial write a string message
// return: Num of written bytes, or -1 if Error
long int MotorSerialCom::write_string(std::string msg)
{
    long int sent_bytes = write(serial, msg.c_str(), msg.size());
    if(sent_bytes<0){
        sprintf(print_msg_buf,"Failed to send serial message: %s\n", msg.c_str());
        print_error(print_msg_buf);
    }
    else if(debug){
        sprintf(print_msg_buf,"Serial sent %li bytes of message: %s\n", sent_bytes, msg.c_str());
        print_debug(print_msg_buf);
        
    }
    return sent_bytes;
}

// Serial write an array of uint8_t data
// return: Num of written bytes, or -1 if Error
long int MotorSerialCom::write_bytearray(uint8_t* bytearray, uint8_t size)
{
    long int sent_bytes = write(serial, bytearray, size);
    if(sent_bytes<0){
        std::string msg = "Failed to send serial data: | ";
        for(int i=0; i<size; i++){
            // sprintf(print_msg_buf," %i |", *(bytearray+i));
            msg += std::to_string(*(bytearray+i));
            msg += " | ";
        }
        msg+="\n";
        print_error(msg);
    }
    else if(debug){
        sprintf(print_msg_buf,"Serial sent %i bytes of data: | ", size);
        std::string msg = print_msg_buf;
        for(int i=0; i<size; i++){
            sprintf(print_msg_buf, " %i |", *(bytearray+i));
            msg += std::to_string(*(bytearray+i));
            msg += " | ";
        }
        msg+="\n";
        print_debug(msg);
    }
    return sent_bytes;
}

void MotorSerialCom::read_()
{
    int size = 6;

    memset(&read_buf_char, 0, sizeof(read_buf_uint));
    int num_bytes = read(serial, &read_buf_char, size);

    if (num_bytes < 0) {
      printf("Error reading: %s\n", strerror(errno));
    }

    printf("rx: %s\n", read_buf_char);
    // for(int i = 0; i<sizeof(read_buf_char); i++)
    // {
    //     printf(" %i |", *(read_buf_char + i));
    // }
    // printf(" \n");

}

int MotorSerialCom::close_()
{
    int is_closed = close(serial);
    if(is_closed<0)
    {
        sprintf(print_msg_buf,"Failed to close the serial port: %s\n", serial_port_.c_str());
        print_error(print_msg_buf);
    }
    else{
        sprintf(print_msg_buf,"Serial port %s is successfully closed!\n", serial_port_.c_str());
        print_info(print_msg_buf);
    }
    return is_closed;

}

void MotorSerialCom::print_info(std::string msg)
{
    printf("\033[0;33m[MotorSerialCom][Info] %s\033[0m", msg.c_str());
}

void MotorSerialCom::print_error(std::string msg)
{
    printf("\033[1;31m[MotorSerialCom][Error] %s\033[0m", msg.c_str());
}

void MotorSerialCom::print_debug(std::string msg)
{
    printf("\033[1;32m[MotorSerialCom][Debug] %s\033[0m", msg.c_str());
}