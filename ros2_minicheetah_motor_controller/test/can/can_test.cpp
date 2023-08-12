// #include <system.h>
#include <iostream>
#include <cstdio>


void init_slcan(std::string device, uint32_t bitrate=1000000, uint32_t baudrate){
    /* 
        Make sure to edit sudoers file using `sudo visudo` command to be able to run sudo commands withoout password:
            - add below line after %sudo ALL=(ALL:ALL) ALL` line:
                `username ALL = (ALL) NOPASSWD: ALL`
            replace user name with your username of the linux system
    */
   int bitrate_;
   switch (bitrate)
   {
    case 10000:
        bitrate_ = 0;
        break;
    case 20000:
        bitrate_ = 1;
        break;
    case 50000:
        bitrate_ = 2;
        break;
    case 100000:
        bitrate_ = 3;
        break;
    case 125000:
        bitrate_ = 4;
        break;
    case 250000:
        bitrate_ = 5;
        break;
    case 500000:
        bitrate_ = 6;
        break;
    case 800000:
        bitrate_ = 7;
        break;
    case 1000000:
        bitrate_ = 8;
        break;
    default:
        bitrate_ = 8;
        break;
   }

   char* cmd;
   // Create SocketCAN device from serial interface
   sprintf(cmd, "sudo slcand -o -c -s%i -S%i can0", bitrate_, baudrate);
   system(cmd);
}

int main()
{
    system("echo hello");


}