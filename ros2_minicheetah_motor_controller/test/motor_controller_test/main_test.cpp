#include <cstdio>
#include <iostream>

#include <stdio.h>
#include <yaml.h>
#include <assert.h>

#include "motor_controller.h"
// #include "rclcpp/rclcpp.hpp"


using namespace std;

void print_motor_params(Motor* m){
    printf("---------------------\n");
    printf("motor %i params: \n", m->id);
    printf("\t max_p: %f\n", m->params.max_p);
    printf("\t max_v: %f\n", m->params.max_v);
    printf("\t max_kp: %f\n", m->params.max_kp);
    printf("\t max_kd: %f\n", m->params.max_kd);
    printf("\t max_iff: %f\n", m->params.max_iff);
    printf("---------------------");
}

void print_control_limits(Motor* m){
    printf("---------------------\n");
    printf("motor %i control limits: \n", m->id);
    printf("\t min_p: %f\n", m->control_limits.min_p);
    printf("\t max_p: %f\n", m->control_limits.max_p);
    printf("\t max_v: %f\n", m->control_limits.max_v);
    printf("\t max_i: %f\n", m->control_limits.max_i);
    printf("---------------------\n");
}

void print_states(Motor* m){
    printf("---------------------\n");
    printf("motor %i states: \n", m->id);
    printf("\t is_enabled: %i\n", m->states.is_enabled);
    printf("\t position: %f\n", m->states.position);
    printf("\t velocity: %f\n", m->states.velocity);
    printf("\t current: %f\n", m->states.curent);
    printf("---------------------\n");
}

void print_rx_data(Motor* m)
{
    printf("motor %i rx_packet: | %i | %i | %i | %i | %i |\n", m->id, m->rx_packet[0], m->rx_packet[1],\
     m->rx_packet[2], m->rx_packet[3], m->rx_packet[4]);
}

int main(int argc, char **argv)
{
    MotorController m_drv{0, 4};

    Motor* m1 = m_drv.add_motor(1);
    Motor* m2 = m_drv.add_motor(2);

    m_drv.set_motor_params(m1, 12.5, 65.0, 500.0, 5.0, 20);
    m_drv.set_control_limits(m1, -1.0, 1.0, 1.5, 15);

    // m_drv.set_motor_params(m2, 10.5, 55.0, 300.0, 10.0, 30);
    // m_drv.set_control_limits(m2, -1.5, 1.5, 1.0, 10);

    m_drv.enable_motor(m1);
    // m_drv.disable_motor(m1);
    m_drv.set_motor_zero(m1);

    m1->rx_packet[0] = 127;
    m1->rx_packet[1] = 255;
    m1->rx_packet[2] = 127;
    m1->rx_packet[3] = 0;
    m1->rx_packet[4] = 0;

    MotorStates state_ = m_drv.get_motor_states(m1);
    
    print_motor_params(m1);
    print_control_limits(m1);
    print_rx_data(m1);
    print_states(m1);

    


    return 0;
}


/* 
 g++ main_test.cpp motor_controller_test.cpp

 ./a.out
 
 */