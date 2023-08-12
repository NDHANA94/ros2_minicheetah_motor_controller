#ifndef MOTOR_H_
#define MOTOR_H_

/*
MIT License

Copyright (c) 2023 Nipun Dhananjaya

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"



#define MAX_NUM_MOTORS 12

// ---------------------
struct{
    double set;
    double min;
    double max;
}typedef param_t;

struct{
    param_t p_des;
    param_t v_des;
    param_t kp;
    param_t kd;
    param_t i_ff;
    double max_motor_current;
}typedef control_params_t;
// ----------------------

struct{
    uint8_t raw;
    double  unpacked;
}typedef state_t_;

struct{
    state_t_ position;
    state_t_ velocity;
    state_t_ current;
}typedef state_t;
// ----------------------

struct{
    int s = -1;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    char *ifname;
    int ret = EXIT_FAILURE;
}typedef can_t;

struct{
    double p_max;
    double p_min;
    double v_max;
    double v_min;
    double i_max;
    double i_min;
}typedef limit_t;

// =====================================================
#pragma once
class Motor
{
private:
    int status; // ....| is_enable | is_available | error |

    void pack_cmd(control_params_t* ctrl_params);
    int send_can(can_t* can);
    int read_can();
    void unpack_read();
    
public:
    Motor();
    ~Motor();

    limit_t limit;
    uint8_t id;
    state_t state;
    control_params_t ctrl_param;

    can_t* can_ptr;
    
    int enable();
    int disable();
    int set_zero();
    int set_position(double position);
    int set_position(double position, double velocity);
    int set_position(double position, double velocity, double kp, double kd, double i_ff);
    int set_velocity(double velocity);
    int set_kp(double kp);
    int set_kd(double kd);
    int set_iff(double iff);
    int set_position_range(double min, double max);
    int set_velocity_range(double min, double max);
    int set_kp_range(double min, double max);
    int set_kd_range(double min, double max);
    int set_iff_range(double min, double max);
    int set_current_limit(double max_current); // motor will be disabled once the motor currunt exceed this limit.
    
};


#endif // MOTOR_H_