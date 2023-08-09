#include <stdio.h>
#include <string>
#include <iostream>

#include "motor_serial_com.h"


#define BAUDRATE 115200
#define PORT "/dev/ttyUSB0"



int main(int argc, char** argv)
{
     MotorSerialCom mserial{"/dev/ttyUSB0", 115200, 1};
    //  mserial.debug = false;
     mserial.write_string("hello world!");
     

     uint8_t tx[9];
     tx[0] = 100;
     tx[1] = 10;
     tx[2] = 1;
    //  mserial.write_bytearray(tx, sizeof(tx));
     mserial.read_();

    return 1;
}