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

struct {
    float max_p;
    float max_v;
    float max_kp;
    float max_kd;
    float max_iff;
} typedef MotorParams;

struct{
    float min_p;
    float max_p;
    float max_v;
    float max_i;
}typedef MotorControlLimits;

struct{
    float p_des;
    float v_des;
    float kp;
    float kd;
    float i_ff;
}typedef ControlCmds;

struct {
    bool is_enabled;
    float position;
    float velocity;
    float curent;
}typedef MotorStates;

struct Motor
{
    uint8_t id;
    uint8_t rx_packet[6]; // [ id , p_H[8] , p_L[8] , v_H[8[] , v_L[4]+i_H[4] , i_L[8] ]
    uint8_t tx_packet[9]; // [ id, p_H[8], p_L[8], v_H[8], v_L[4]+kp_H[4], kp_L[8], kd_H[8], kd_L[4]+iff_H[4], iff_L[8] ]
    uint8_t Error;
    bool config_status[2]; // config_status[0] == true; -> if motor_params were added
                           // config_status[1] == true; -> if motor control limits were added 

    MotorParams params;
    MotorControlLimits control_limits; 
    ControlCmds control_cmd; 
    MotorStates states;

    Motor();
    Motor(uint8_t id_, MotorParams motor_params_, MotorControlLimits control_limits_);
    ~Motor();
};


#endif // MOTOR_H_