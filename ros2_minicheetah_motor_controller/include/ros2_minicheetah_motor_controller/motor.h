#ifndef MOTOR_H_
#define MOTOR_H_

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