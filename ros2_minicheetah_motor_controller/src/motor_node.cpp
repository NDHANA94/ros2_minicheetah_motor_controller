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

// #include <cstdio>
// #include <functional>
// #include <memory>
// #include <string>
// #include <chrono>
// #include "serial/serial.h"

// #include "ros2_minicheetah_motor_controller/motor_controller.h"

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_minicheetah_motor_controller/motor_node.hpp"

// using namespace std::chrono_literals;



MiniCheetahMotorController::MiniCheetahMotorController()
: Node("motor_controller_node")
{
  declare_parameters();
  read_parameters();

  if(com_interface_ == COM_INTERFACE_1){}
  else if (com_interface_ == COM_INTERFACE_2) init_serial();
  else {}
  subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>
                  (subscriber_name_, 
                  10, 
                  std::bind(&MiniCheetahMotorController::subscription_callback, 
                  this, std::placeholders::_1));
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1/motor_state_update_freq_*1000)), std::bind(&MiniCheetahMotorController::timer_callback, this));

}

MiniCheetahMotorController::~MiniCheetahMotorController()
{
  serial_.self.close();
  RCLCPP_INFO(this->get_logger(), "Closing motor_node");
}


void MiniCheetahMotorController::declare_parameters()
{
  this->declare_parameter("subscriber_name");
  this->declare_parameter("com_interface");
  this->declare_parameter("serial_port.port");
  this->declare_parameter("serial_port.baudrate");
  this->declare_parameter("serial_port.timeout");
  this->declare_parameter("can_port.port");
  this->declare_parameter("can_port.interface");
  this->declare_parameter("can_port.bitrate");
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

void MiniCheetahMotorController::read_parameters(){
  subscriber_name_ = this->get_parameter("subscriber_name").as_string();
  com_interface_ = this->get_parameter("com_interface").as_string();
  serial_.port = this->get_parameter("serial_port.port").as_string();
  serial_.baudrate = this->get_parameter("serial_port.baudrate").as_int();
  serial_.timeout = this->get_parameter("serial_port.timeout").as_int();
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

void MiniCheetahMotorController::init_serial()
{
  try
    {
      serial_.self.setPort(serial_.port.c_str());
      serial_.self.setBaudrate(serial_.baudrate);
      serial_.self.setFlowcontrol(serial::flowcontrol_none);
      serial_.self.setParity(serial::parity_none);
      serial_.self.setStopbits(serial::stopbits_one);
      serial_.self.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_.timeout);
      serial_.self.setTimeout(time_out);
      serial_.self.open();
    }
    catch(serial::IOException& e)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to open serial port " << serial_.port.c_str());
      exit(0);
    }
    if (serial_.self.isOpen())
    {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Initialized Serial port " << serial_.port.c_str());
    }
    else
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to initialize serial port " << serial_.port.c_str());
      exit(0);
    }
}

  
void MiniCheetahMotorController::subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
{
  std::string data = "";
  for(int i = 0; i < int(sizeof(msg->data)); i++){
    printf("%f", msg->data[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Recieved data:\ns");
}


void MiniCheetahMotorController::timer_callback()
{
  uint8_t tx_data[9];
  tx_data[0] = 100;
  size_t written_bytes = serial_.self.write(tx_data, sizeof(tx_data));
  RCLCPP_INFO(this->get_logger(), "\033[0;32mserial wrote %i bytes: |%i|%i|%i|%i|%i|%i|%i|%i|%i|\033[0m", written_bytes,
                                                          tx_data[0], tx_data[1], tx_data[2], tx_data[3],
                                                          tx_data[4], tx_data[5], tx_data[6], tx_data[7], tx_data[8]);

  uint8_t rx_data[9];
  memset(rx_data, 0, sizeof(rx_data));
  size_t bytes_read = serial_.self.read(rx_data, 9);
  RCLCPP_INFO(this->get_logger(), "\033[0;36mserial read %i bytes:  |%i|%i|%i|%i|%i|%i|%i|%i|%i|\033[0m", 
                                                          bytes_read,
                                                          rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                                                          rx_data[4], rx_data[5], rx_data[6], rx_data[7], rx_data[8]);
}



//  ============================================================================
//  =========================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniCheetahMotorController>());
  rclcpp::shutdown();

  return 0;
}
