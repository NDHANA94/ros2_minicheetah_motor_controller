#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include <cstdio>
#include <iostream>
#include "motor.h"

#pragma once
class MotorController
{
private:
    uint8_t _id, _num_of_motors, _num_of_available_motors;
    uint8_t _com_interface; // 0-Serial, 1-CAN
    uint8_t* tx_packet; // [id, p_des_H[8], p_des_L[8], v_des_H[8], v_des_L[4]+kp_H[4], kp_L[8], kd_H[8], kd_L[4]+iff_H[4], iff_L[8]]
    uint8_t* rx_packet; // [id, p_H[8], p_L[8], v_H[8], v_L[4]+i_H[4], i_L[8]]

    // ptr for motor obejetcs
    Motor* motor;
    
    // private method
    void unpack_rx_packet(uint8_t rx_data[6]); // return [p, v, iff]
    void pack_tx_packet(Motor *motor_); 
    // uint8_t* read_serial(uint8_t bytes);
    // bool write_serial();

    float fmaxf(float x, float y);
    float fminf(float x, float y);
    int float2uint(float x, float x_min, float x_max, int bits);
    float uint2float(int x, float x_min, float x_max, int bits);


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
