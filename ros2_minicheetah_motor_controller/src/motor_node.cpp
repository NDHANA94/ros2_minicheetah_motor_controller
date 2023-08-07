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

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include "serial/serial.h"

#include "ros2_minicheetah_motor_controller/motor_controller.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class MiniCheetahMotorController : public rclcpp::Node
{
public:
  MiniCheetahMotorController()
  : Node("minicheetar_motor_node")
  { declare_parameters();
    read_parameters();
    init_serail();
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>\
    (subscriber_name_, 10, std::bind(&MiniCheetahMotorController::subscription_callback, this, std::placeholders::_1));
    
  }

  ~MiniCheetahMotorController(){
    serial_.close();
    RCLCPP_INFO(this->get_logger(), "Closing motor_node");
  }



private:

  void declare_parameters(){
    this->declare_parameter("subscriber_name");
    this->declare_parameter("com_interface");
    this->declare_parameter("serial_port");
    this->declare_parameter("serial_baud");
    this->declare_parameter("serial_timeout");
    this->declare_parameter("read_cmd_response_delay");
    this->declare_parameter("read_set_zero_response_delay");
    this->declare_parameter("state_update_frequency");
    this->declare_parameter("num_of_motors");
    this->declare_parameter("max_p");
    this->declare_parameter("max_v");
    this->declare_parameter("max_kp");
    this->declare_parameter("max_kd");
    this->declare_parameter("max_iff");
  }

  void read_parameters(){
    subscriber_name_ = this->get_parameter("subscriber_name").as_string();
    com_interface_ = this->get_parameter("com_interface").as_string();
    serial_port_ = this->get_parameter("serial_port").as_string();
    serial_baud_ = this->get_parameter("serial_baud").as_int();
    serial_timeout_ = this->get_parameter("serial_timeout").as_int();
    read_cmd_response_delay_ = this->get_parameter("read_cmd_response_delay").as_double();
    read_set_zero_response_delay_ = this->get_parameter("read_set_zero_response_delay").as_double();
    motor_state_update_freq_ = this->get_parameter("state_update_frequency").as_int();
    num_of_motors_ = this->get_parameter("num_of_motors").as_int();
    max_p_ = this->get_parameter("max_p").as_double();
    max_v_ = this->get_parameter("max_v").as_double();
    max_kp_ = this->get_parameter("max_kp").as_double();
    max_kd_ = this->get_parameter("max_kd").as_double();
    max_iff_ = this->get_parameter("max_iff").as_double();
  }

  void init_serail(){
    try
    {
      serial_.setPort(serial_port_.c_str());
      serial_.setBaudrate(serial_baud_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none);
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
      serial_.setTimeout(time_out);
      serial_.open();
    }
    catch(serial::IOException& e)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to open serial port " << serial_port_.c_str());
      exit(0);
    }
    if (serial_.isOpen())
    {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Initialized Serial port " << serial_port_.c_str());
    }
    else
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to initialize serial port " << serial_port_.c_str());
    }
  }
  
  void subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
  {
    std::string data = "";
    for(int i = 0; i < int(sizeof(msg->data)); i++){
      printf("%f", msg->data[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Recieved data:\ns");
    
  }

 

  std::string subscriber_name_ = "motor_cmds", com_interface_, serial_port_;
  double read_cmd_response_delay_, read_set_zero_response_delay_, max_p_, max_v_, max_kp_, max_kd_, max_iff_;
  int motor_state_update_freq_, num_of_motors_ , serial_baud_, serial_timeout_;
  
  serial::Serial serial_;

  
  

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniCheetahMotorController>());
  rclcpp::shutdown();

  return 0;
}
