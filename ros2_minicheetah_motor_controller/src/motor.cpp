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
    status = 0;
}

Motor::Motor(can_t* can)
{
    id = 0;
    status = 0;
    can_ptr = can;
}

Motor::~Motor()
{   
    delete [] can_ptr;
    printf("Motor class constructor awoked");
}

void Motor::set_can(can_t* can)
{
    can_ptr = can;
}

void Motor::set_id(int id_)
{
    id = id_;
    status |= MOTOR_ID_SET; // set id_set bit of status flag
}


void Motor::set_position_range(double max)
{
    motor_params.p_des.max = max;
    motor_params.p_des.min = -motor_params.p_des.max;
    // set 'set p_des' bit of the set_status flag
    motor_params.set_status |= PDES_PARAMS_SET;
    // if all motor params are set, then set the 'params set' (4th) bit of the status.
    if (this->motor_params.set_status == 0b11111){
        set_motor_status(MOTOR_PARAMS_SET);
        sprintf(print_msg, "%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
        printf(print_msg);
    } 
}

void Motor::set_velocity_range(double max)
{
    motor_params.v_des.max = max;
    motor_params.v_des.min = -motor_params.v_des.max;
    // set 'set v_des' bit of the set_status flag
    motor_params.set_status |= VDES_PARAMS_SET;
    // if all motor params are set, then set the 'params set' (4th) bit of the status.
    if (motor_params.set_status == 0b11111){
        set_motor_status(MOTOR_PARAMS_SET);
        sprintf(print_msg, "%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
        printf(print_msg);
    }
}

void Motor::set_kp_range(double max)
{
    motor_params.kp.max = max;
    motor_params.kp.min = 0.0;
    // set 'set kp' bit of the set_status flag
    motor_params.set_status |= KP_PARAMS_SET;
    // if all motor params are set, then set the 'params set' (4th) bit of the status.
    if (motor_params.set_status == 0b11111){
        set_motor_status(MOTOR_PARAMS_SET);
        sprintf(print_msg, "%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
        printf(print_msg);
    }
}

void Motor::set_kd_range(double max)
{
    motor_params.kd.max = max;
    motor_params.kd.min = 0.0;
    // set 'set kd' bit of the set_status flag
    motor_params.set_status |= KD_PARAMS_SET;
    // if all motor params are set, then set the 'params set' (4th) bit of the status.
    if (motor_params.set_status == 0b11111){
        set_motor_status(MOTOR_PARAMS_SET);
        sprintf(print_msg, "%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
        printf(print_msg);
    }
}

void Motor::set_iff_range(double max)
{
    motor_params.i_ff.max = max;
    motor_params.i_ff.min = -motor_params.i_ff.max;
    // set 'set i_ff' bit of the set_status flag
    motor_params.set_status |= IFF_PARAMS_SET;
    // if all motor params are set, then set the 'params set' (4th) bit of the status.
    if (motor_params.set_status == 0b11111){
        set_motor_status(MOTOR_PARAMS_SET);
        // sprintf(print_msg, "%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
        printf("%s[INFO][motor id-%i]motor params are set%s\n", BLUE, this->id, NC);
    }
}

void Motor::set_limit_position(std::vector<double> limit_p)
{
    if(limit_p[0] > limit_p[1]){
        double temp = limit_p[0];
        limit_p[0] = limit_p[1];
        limit_p[1] = temp;
    }
    limit.position.min = limit_p[0];
    limit.position.max = limit_p[1];
}

void Motor::set_limit_velocity(double limit_v)
{
    limit.velocity.min = limit_v;
    limit.velocity.max = -limit.velocity.min;
}

void Motor::set_limit_current(double limit_i)
{
  limit.current.max = limit_i;
  limit.current.min = limit.current.max;
}

int Motor::enable()
{   
    // set id
    can_ptr->tx_frame.can_id = this->id;
    // set payload size
    can_ptr->tx_frame.can_dlc = 8;
    // add data
    unsigned char cmd[8] = MOTOR_ENABLE_CMD;
    for(int i = 0; i<8; i++){
        can_ptr->tx_frame.data[i] = cmd[i];
    }
    // send CAN frame and check error
    ssize_t nbytes = write(can_ptr->s, &can_ptr->tx_frame, sizeof(struct can_frame));
    if (nbytes < 0){
        print_error("Failed to send enable command");
    }
    else{
        print_info("Sent enable command");
    }
    
    // wait for the resoponse from the motor and read the response
    nbytes = read(can_ptr->s, &can_ptr->rx_frame, sizeof(struct can_frame));
    if(nbytes < 0){
        print_error("No response from the motor");
        for(int i=0; i<can_ptr->write_retries; i++){

    }
    }
    
    
    sprintf(print_msg,"can recv %i bytes\n", nbytes);
    print_info(print_msg);

    // add a condition which check if the motor is enabled and the set MOTOR_ENABLED status
    set_motor_status(MOTOR_ENABLED);
    return 0;
}

int Motor::disable()
{
    // add a condition which check if the motor is disabled and the reset MOTOR_ENABLED status
    reset_motor_status(MOTOR_ENABLED);
    return 0;
}

int Motor::set_zero()
{
    // add a condition which check if the motor is set to zero 
    return 0;
}

void Motor::set_kp(double kp){
    cmd.kp = kp;
}

void Motor::set_kd(double kd){
    cmd.kd = kd;
}

void Motor::set_iff(double iff){
    cmd.i_ff = iff;
}

int Motor::set_position(double position){
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

motor_states_t Motor::get_states(){
    return states;
}



void Motor::pack_cmd()
{
    
}

int Motor::send_can()
{
    u_int8_t nbytes = write(can_ptr->s, &can_ptr->tx_frame, sizeof(struct can_frame));
    if(nbytes < 0){
        // RCLCPP_ERROR(node->)
    }
    return 0;
}

int Motor::read_can()
{
    return 0;
}

void Motor::unpack_read()
{

}

bool Motor::check_is_motor_available(){
    return 0;
}

void Motor::set_motor_status(unsigned char state_){
    status |= state_;
}

void Motor::reset_motor_status(unsigned char state_){
    status &=~ state_;
}

unsigned char Motor::get_status(){
    return status;
}

bool Motor::is_status_set(unsigned char state_){
    return status & state_;
}

void Motor::print_info(char* info){
    printf("[INFO]%s[motor][id: %i]%s%s\n", BLUE, id, info, NC);
}

void Motor::print_error(char* err){
    sprintf(print_msg, "[ERROR]%s[motor][id: %i]%s%s\n", RED, id, err, NC);
    perror(print_msg);
}
