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
#include <vector>
#include <bitset>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

#include "ros2_minicheetah_motor_controller/color_print.h"

#define MAX_NUM_MOTORS 12

#define PDES_PARAMS_SET     0b10000
#define VDES_PARAMS_SET     0b01000
#define KP_PARAMS_SET       0b00100
#define KD_PARAMS_SET       0b00010
#define IFF_PARAMS_SET      0b00001

#define MOTOR_ID_SET        0b00000001
#define MOTOR_AVAILABLE     0b00000010
#define MOTOR_ENABLED       0b00000100
#define MOTOR_PARAMS_SET    0b00001000
#define MOTOR_IN_RANGE      0b00010000
#define MOTOR_CAN_SET       0b00100000
#define MOTOR_READY_TO_MOVE 0x3F        // or 0b00111111

#define SET     |=
#define UNSET   &=~


#define MOTOR_ENABLE_CMD        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
#define MOTOR_DISABLE_CMD       {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}
#define MOTOR_SETZERO_CMD       {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}

// ---------------------
// range: min, max
struct{
    double min;
    double max;
}typedef range_t;

/**
 * Motor params: 
 * \param p.min: Minimum value of motor position 
 * \param p.max: Maximum value of motor position
 * \param v.min: Minimum value motor velocity
 * \param v.max: Maximum value motor velocity 
 * \param kp.min: Minimum value motor Kp
 * \param kp.max: Maximum value motor Kp 
 * \param kd.min: Minimum value motor Kd
 * \param kd.max: Maximum value motor Kd 
 * \param i_ff.min: Minimum value motor feed forward current
 * \param i_ff.max: Maximum value motor feed foward current
 * \param set_status: status of params configuration
 */
struct{
    range_t p_des;
    range_t v_des;
    range_t kp;
    range_t kd;
    range_t i_ff;
    int set_status = 0b00000; // | set p_des | set v_des| set kp | set kd | set i_ff |
}typedef motor_params_t;
// ----------------------

/**
 * Limit motor position, velocity and current
 * \param position.min: to keep motor position not lower than this value
 * \param position.max: to keep motor position not higher than this value
 * \param velocity.min: to keep motor velocity not lower than this value
 * \param velocity.max: to keep motor velocity not hiher than this value
 * \param current.min: to keep motor current not lower than this value
 * \param current.max: to keep motor current not highr than this value
*/
struct{
    range_t position;
    range_t velocity;
    range_t current;
}typedef motor_limit_t;
// ----------------------


/**
 * Motor commands:
 * \param p_des: Desired position
 * \param v_des: Desired position
 * \param kp: Stiffness
 * \param kd: Damping
 * \param i_ff: Feed forward current
 */
struct{
    double p_des;
    double v_des;
    double kp;
    double kd;
    double i_ff;
}typedef motor_cmd_t;
// ---------------------

struct{
    uint8_t raw;
    double  unpacked;
}typedef state_t_;

/**
 * State of the motor
 * \param position: current position of the motor
 * \param velocity: current velocity of the motor
 * \param current: current motor intake electric current
  */
struct{
    state_t_ position;
    state_t_ velocity;
    state_t_ current;
}typedef motor_states_t;
// ----------------------

/**
 * CAN Socket parameters
 * \param s: Socket
 * \param frame: CAN frame
 * \param ifr: 
 * \param *ifname: 
 * \param ret:  
  */
struct{
    int s = -1;
    struct sockaddr_can addr;
    struct can_frame cmd_frame; // use for sending motor command
    struct can_frame fb_frame; // use for receiving motor feedback
    struct ifreq ifr;
    char *ifname;
    int ret = EXIT_FAILURE;
    struct timeval tv;
    uint8_t write_retries;
}typedef can_t;


// =====================================================
#pragma once
class Motor
{
private:
    uint8_t id;
    
    motor_limit_t limit;
    motor_states_t states;
    motor_cmd_t cmd;
    unsigned char status = 0; // 0b00xxxxxx : | 7: - | 6: - | 5: CAN set | 4: in range | 3: params set | 2: enabled | 1: available | 0: id set |
    can_t* can_ptr;
    // rclcpp::Logger(*loggerPtr_)();
    char print_msg[50];
    
    void pack_cmd();
    int _send_can_frame(struct can_frame frame);
    int send_can_frame(struct can_frame frame);
    int get_motor_response();
    void unpack_read();

    bool check_is_motor_available();
    void set_motor_status(unsigned char state_); // OK
    void reset_motor_status(unsigned char state_); // OK

    void print_info(char* info);
    void print_error(char* err);
    
    
    
public:
    Motor();
    Motor(can_t* can);
    ~Motor();
    motor_params_t motor_params;
    void set_id(int id_);  // OK
    void set_can(can_t* can); // OK
    bool debug_mode=true;

    void set_position_range(double max); // OK
    void set_velocity_range(double max); // OK
    void set_kp_range(double max); // OK
    void set_kd_range(double max); // OK
    void set_iff_range(double max); // OK

    void set_limit_position(std::vector<double> limit_p); // OK
    void set_limit_velocity(double limit_v); // OK
    void set_limit_current(double limit_i); // OK
 
    int enable(); // TODO
    int disable(); // TODO
    int set_zero(); // TODO

    void set_kp(double kp); // OK
    void set_kd(double kd); // OK
    void set_iff(double iff); // OK

    int set_position(double position); // TODO
    int set_position(double position, double velocity); // TODO
    int set_position(double position, double velocity, double kp, double kd, double i_ff); // TODO
    int set_velocity(double velocity); // TODO

    motor_states_t get_states();
    
    bool is_status_set(unsigned char state_); // OK
    unsigned char get_status(); // OK

    std::string get_can_frame_as_string(struct can_frame frame_);

};


#endif // MOTOR_H_