#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

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

#include <cstdio>
#include <iostream>
#include "motor.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"

#pragma once
class MotorController
{
private:
    Motor* motor;
    serial::Serial serial_;
    

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);


    uint8_t _id, _num_of_motors, _num_of_available_motors;
    uint8_t _com_interface; // 0-Serial, 1-CAN
    uint8_t* tx_packet; // [id, p_des_H[8], p_des_L[8], v_des_H[8], v_des_L[4]+kp_H[4], kp_L[8], kd_H[8], kd_L[4]+iff_H[4], iff_L[8]]
    uint8_t* rx_packet; // [id, p_H[8], p_L[8], v_H[8], v_L[4]+i_H[4], i_L[8]]

    // private method
    void unpack_rx_packet(Motor *motor_); // return [p, v, iff]
    void pack_tx_packet(Motor *motor_); 
    // uint8_t* read_serial(uint8_t bytes);
    // bool write_serial();

    float fmaxf(float x, float y);
    float fminf(float x, float y);
    int float2uint(float x, float x_min, float x_max, int bits);
    float uint2float(int x, float x_min, float x_max, int bits);

    void init_serial(std::string port, uint32_t baudrate, uint8_t timeout);
    void init_can(std::string port, uint32_t bitrate, uint8_t timeout);
    void send_motor_cmd(uint8_t tx[9]);


public:
    // constructor
    MotorController(uint8_t com_interface, uint8_t num_of_motors);
    // decontructor
    ~MotorController();

    uint8_t Error;

    // add a new motor
    Motor* add_motor(uint8_t id); 
    // set motor parameters
    void set_motor_params(Motor* motor_, float max_p, float max_v, float max_kp, float max_kd, float max_iff);
    // set custom control limits 
    void set_control_limits(Motor* motor_, float min_p, float max_p, float max_v, float max_i);

    // set cureent motor position as zero position
    void set_motor_zero(Motor* motor_);
    // enable motor
    void enable_motor(Motor* motor_);
    // enable all the added motors
    void enable_all_motors(); 
    // disable motor
    void disable_motor(Motor* motor_); 
    // disable all added motors
    void disable_all_motors(); 

    // get motor states: is_enabled, position, velocity, current
    MotorStates get_motor_states(Motor* motor_); 

    // set motor position
    void set_motor_position(Motor* motor_, ControlCmds cmd); // set position of one motor

    // uint8_t get_num_of_motors();
    // bool is_motor_enable();
};

#endif //__MOTOR_MODULE_H__
