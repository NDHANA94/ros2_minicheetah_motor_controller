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
    uint8_t timeout_;
    char read_buf_char [255];
    uint8_t read_buf_uint [255];

    struct termios tty;
    
    void set_termios();

public:
    MotorSerialCom();
    MotorSerialCom(const char* port, uint8_t timeout);
    ~MotorSerialCom();

    void set_baudrate(unsigned int baudrate);
    bool isOpen();
    long int write_string(std::string* msg);
    long int write_bytearray(uint8_t* msg, uint8_t size);
    // ?? read_bytearray();

    void read_();
    void close_();

    int serial;
    bool debug=true;
};


#endif //MOTOR_SERIAL_COM_H_