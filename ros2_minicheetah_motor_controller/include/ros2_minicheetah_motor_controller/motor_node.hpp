#ifndef MOTOR_NODE_H__
#define MOTOR_NODE_H__

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "serial/serial.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_minicheetah_motor_controller/motor.h"
#include "ros2_minicheetah_motor_controller/color_print.h"

#define PKG_NAME "ros2_minicheetah_motor_controller"


using namespace std::chrono_literals;



struct{
  bool is_can_initialized=false;
  bool is_socket_initialzed=false;
  bool is_open=false;
  bool error=false;
}typedef can_status_t;

struct{
  std::string interface_type;
  std::string port;
  std::string interface_name;
  int bitrate;
  int baudrate;
  int txqueuelen;
  int timeout_ms;
  uint8_t tx_buf[9];
  uint8_t rx_buf[255];
  can_status_t status;
}typedef can_port_t;

#pragma once
class MiniCheetahMotorController : public rclcpp::Node
{
private:
    void init_motors();
    void declare_parameters();
    void read_parameters();
    void init_slcan();
    void init_can();
    int can_read();

    void subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const;
    void timer_callback();

    std::string subscriber_name_ = "motor_cmds", com_interface_;
    double read_cmd_response_delay_, read_set_zero_response_delay_, max_p_, max_v_, max_kp_, max_kd_, max_iff_;
    int motor_state_update_freq_, num_of_motors_;
    std::string param_child[9] = {"id", "max_p", "max_v", "max_kp", "max_kd", "max_iff", "limit_p", "limit_v", "limit_i"};

    can_port_t _can;
    std::string pkg_share_dir;
    std::string slcan_bringup_file_dir;
    can_t can;


    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MiniCheetahMotorController();
    ~MiniCheetahMotorController();

    Motor** motor; // pointer to create an array of Motor objects


};

#endif //MOTOR_NODE_H__


// struct 
// {
//   serial::Serial self;
//   std::string port;
//   double baudrate;
//   int timeout;
//   uint8_t tx_buf[9];
//   uint8_t rx_buf[255];
// }typedef serial_port_t;


