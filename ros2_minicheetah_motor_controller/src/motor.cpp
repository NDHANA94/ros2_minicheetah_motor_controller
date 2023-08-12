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

#include "ros2_minicheetah_motor_controller/motor.h"

Motor::Motor()
{
    
    id = 0;
    state.position.raw = 0;
    state.velocity.raw = 0;
    state.current.raw = 0;
    state.position.unpacked = 0;
    state.velocity.unpacked = 0;
    state.current.unpacked = 0;
    ctrl_param.p_des.max = 12.5;
    ctrl_param.p_des.min = -ctrl_param.p_des.max;
    ctrl_param.v_des.max = 65.0;
    ctrl_param.v_des.min = -ctrl_param.v_des.max;
    ctrl_param.kp.max = 500;
    ctrl_param.kp.min = 0;
    ctrl_param.kd.max = 5.0;
    ctrl_param.kd.min = 0;
    ctrl_param.i_ff.max = 20;
    ctrl_param.i_ff.min = -ctrl_param.i_ff.min;
    status = 0b0000;
    
}

// // motor Parameterized constructor
// Motor::Motor(uint8_t id_)
// {
    
// }

Motor::~Motor()
{   
    printf("Motor class constructor awoked");
}

int Motor::enable()
{
    printf("Motor class deconstructor awoked");
    return 0;
}

int Motor::disable()
{
    return 0;
}

int Motor::set_zero()
{
    return 0;
}

int Motor::set_position(double position)
{
    return 0;
}

int Motor::set_position(double position, double velocity)
{
    return 0;
}

int Motor::set_position(double position, double velocity, double kp, double kd, double i_ff)
{
    return 0;
}

int Motor::set_velocity(double velocity)
{
    return 0;
}

int Motor::set_kp(double kp)
{
    return 0;
}

int Motor::set_kd(double kd)
{
    return 0;
}

int Motor::set_iff(double iff)
{
    return 0;
}

int Motor::set_position_range(double min, double max)
{
    return 0;
}

int Motor::set_velocity_range(double min, double max)
{
    return 0;
}

int Motor::set_kp_range(double min, double max)
{
    return 0;
}

int Motor::set_kd_range(double min, double max)
{
    return 0;
}

int Motor::set_iff_range(double min, double max)
{
    return 0;
}

int Motor::set_current_limit(double max_current)
{
    return 0;
}

void Motor::pack_cmd(control_params_t* ctrl_params)
{
    
}

int Motor::send_can(can_t* can)
{
    return 0;
}

int Motor::read_can()
{
    return 0;
}

void Motor::unpack_read()
{

}
