#ifndef MOTOR_NODE_H__
#define MOTOR_NODE_H__

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include "serial/serial.h"

#include "ros2_minicheetah_motor_controller/motor_controller.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define COM_INTERFACE_1 "can"
#define COM_INTERFACE_2 "serial"

using namespace std::chrono_literals;



struct 
{
  serial::Serial self;
  std::string port;
  double baudrate;
  int timeout;
  uint8_t tx_buf[9];
  uint8_t rx_buf[255];
}typedef serial_port_t;

struct{
  std::string port;
  double bitrate;
  int timeout;
  uint8_t tx_buf[9];
  uint8_t rx_buf[255];
}typedef can_port_t;


class MiniCheetahMotorController : public rclcpp::Node
{
private:
    void declare_parameters();
    void read_parameters();
    void init_serial();
    void subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const;
    void timer_callback();

    std::string subscriber_name_ = "motor_cmds", com_interface_;
    double read_cmd_response_delay_, read_set_zero_response_delay_, max_p_, max_v_, max_kp_, max_kd_, max_iff_;
    int motor_state_update_freq_, num_of_motors_ ;

    serial_port_t serial_;
    can_port_t _can;

    // enum ComInterface {CAN, SERIAL};

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MiniCheetahMotorController();
    ~MiniCheetahMotorController();


};

#endif //MOTOR_NODE_H__