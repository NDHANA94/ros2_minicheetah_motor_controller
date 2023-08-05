#include "ros2_minicheetah_motor_controller/motor.h"

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