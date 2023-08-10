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

#include "motor.h"

// motor default constructor
Motor::Motor()
{
    id = 0;
    Error = 0;
    // rx_packet = new uint8_t [6];
    // tx_packet = new uint8_t [9];
    // motor_params = new uint16_t [5];
    // control_limits = new int8_t [4];
    // control_params = new uint8_t [5];
    // states = new float [4];
}

// motor Parameterized constructor
Motor::Motor(uint8_t id_, MotorParams motor_params_, MotorControlLimits control_limits_)
{
    id = id_;
    Error = 0;
    params = motor_params_;
    control_limits = control_limits_;
    // rx_packet = new uint8_t [6];
    // tx_packet = new uint8_t [9];
    // control_limits = new int8_t [4];
    // motor_params = new uint16_t [5];
    // control_params = new uint8_t [5];
    // states = new float [4];
}

Motor::~Motor()
{   
    printf("closing motor id-%i \n", id);
    // delete [] rx_packet;
    // delete [] tx_packet;
    // delete [] motor_params;
    // delete [] control_limits;
    // delete [] control_params;
    // delete [] states;
    // rx_packet = nullptr;
    // tx_packet = nullptr;
    // motor_params = nullptr;
    // control_limits = nullptr;
    // control_params = nullptr; 
    // states = nullptr;
}