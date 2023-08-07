#ifndef TEST_H__
#define TEST_H__

#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>

// #include "serial/serial.h"


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
    uint8_t rx_packet[5]; // [p_H[8] , p_L[8] , v_H[8[] , v_L[4]+i_H[4] , i_L[8] ]
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

#pragma once
class MotorModule
{
public:
    // constructor
    MotorModule(uint8_t com_interface, uint8_t num_of_motors);
    // decontructor
    ~MotorModule();

    // public methods
    Motor* add_motor(uint8_t id); // add a motor
    void set_motor_params(Motor* motor_, float max_p, float max_v, float max_kp, float max_kd, float max_iff);
    void set_control_limits(Motor* motor_, float min_p, float max_p, float max_v, float max_i);

    void set_motor_zero(Motor* motor_);

    void enable_motor(Motor* motor_);
    void enable_all_motors(); // enable all motors

    void disable_motor(Motor* motor_); // disable one motor
    void disable_all_motors(); // disable all motors

    MotorStates get_motor_states(Motor* motor_); // get status of one motor

    void set_motor_position(Motor* motor_, ControlCmds cmd); // set position of one motor

    Motor* get_motors();

    // void set_motor_positions(tx_packet* tx_data)
    // uint8_t get_num_of_motors();
    // bool is_motor_enable();
    
    // error
    uint8_t Error;




 
private:
    // member variables
    uint8_t _id, _num_of_motors, _num_of_available_motors;
    uint8_t _com_interface; // 0-Serial, 1-CAN
    uint8_t* tx_packet; // [id, p_des_H[8], p_des_L[8], v_des_H[8], v_des_L[4]+kp_H[4], kp_L[8], kd_H[8], kd_L[4]+iff_H[4], iff_L[8]]
    uint8_t* rx_packet; // [id, p_H[8], p_L[8], v_H[8], v_L[4]+i_H[4], i_L[8]]

    // create an array of motor obejetcs
    Motor* motor;

    // serial::Serial serial_;
    

    // private methods
    void unpack_rx_packet(Motor* motor_); // return [p, v, iff]
    void pack_tx_packet(Motor *motor_); 
    // uint8_t* read_serial(uint8_t bytes);
    // bool write_serial();

    float fmaxf(float x, float y);
    float fminf(float x, float y);
    int float2uint(float x, float x_min, float x_max, int bits);
    float uint2float(int x, float x_min, float x_max, int bits);
    
};


#endif //TEST_H__