#ifndef MOTOR_SERIAL_COM_H_
#define MOTOR_SERIAL_COM_H_

// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUDRATE 115200
#define DEFAULT_TIMEOUT 1


class MotorSerialCom
{
private:
    bool is_serial_initialized = false;
    bool is_serial_open = false;
   
    char read_buf_char [255];
    uint8_t read_buf_uint [9];

    std::string serial_port_;
    uint32_t serial_baudrate_;
    float serial_timeout_;

    char print_msg_buf[255];

    struct termios tty;
    
    void set_termios();

    void print_info(std::string msg);
    void print_error(std::string msg);
    void print_debug(std::string msg);

public:
    MotorSerialCom();
    MotorSerialCom(const char* port, uint32_t baudrate, uint8_t timeout);
    ~MotorSerialCom();

    void set_baudrate(unsigned int baudrate);
    bool isOpen();
    long int write_string(std::string msg);
    long int write_bytearray(uint8_t* msg, uint8_t size);
    // ?? read_bytearray();

    void read_();
    int close_();

    int serial;
    bool debug=true;
};


#endif //MOTOR_SERIAL_COM_H_