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

#include <sys/ioctl.h>
#include "ros2_minicheetah_motor_controller/motor_node.hpp"
#include "ros2_minicheetah_motor_controller/motor.h"


MiniCheetahMotorController::MiniCheetahMotorController()
: Node("motor_controller_node")
{
  declare_parameters(); 
  init_motors();
  read_parameters(); 
  init_can(); // initialize can interface, if there will be an error terminate the program.


  subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>
                  (subscriber_name_, 
                  10, 
                  std::bind(&MiniCheetahMotorController::subscription_callback, 
                  this, std::placeholders::_1));
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1/motor_state_update_freq_*1000)), std::bind(&MiniCheetahMotorController::timer_callback, this));

}

MiniCheetahMotorController::~MiniCheetahMotorController()
{
  char* slcand_close_cmd;
  slcand_close_cmd = new char[255];
  sprintf(slcand_close_cmd, "%s -r", slcan_bringup_file_dir.c_str());
  system(slcand_close_cmd);
  delete [] slcand_close_cmd;
  slcand_close_cmd = nullptr;
  // delete [] motor;
  // motor = nullptr;
  RCLCPP_INFO(this->get_logger(), "Closing motor_node");
}

 // load num_of_motors befor this and the read parameters
void MiniCheetahMotorController::init_motors()
{
  motor = new Motor[num_of_motors_]{};
}

void MiniCheetahMotorController::declare_parameters()
{
  this->declare_parameter("subscriber_name");

  this->declare_parameter("can_interface.interface_type");
  this->declare_parameter("can_interface.port");
  this->declare_parameter("can_interface.interface_name");
  this->declare_parameter("can_interface.bitrate_code");
  this->declare_parameter("can_interface.baudrate");
  this->declare_parameter("can_interface.txqueuelen");

  this->declare_parameter("read_cmd_response_delay");
  this->declare_parameter("read_set_zero_response_delay");
  this->declare_parameter("state_update_frequency");

  this->declare_parameter("num_of_motors");
  num_of_motors_ = this->get_parameter("num_of_motors").as_int();

  
  uint8_t num_of_childs = sizeof(param_child)/sizeof(param_child[0]);
  char param[25];
  if(num_of_motors_ > 12){
    num_of_motors_ = 12;
    RCLCPP_WARN(this->get_logger(), "\033[0;33mNumber of maximum motors are 12. Only first 12 motors will be initialized\033[0m");
  }
  for(int i=1; i<=num_of_motors_; i++){
    for(int j=0; j<num_of_childs; j++){
      sprintf(param, "m%i_params.%s", i, param_child[j].c_str());
      this->declare_parameter(param);
    }
  }
}

void MiniCheetahMotorController::read_parameters(){
  subscriber_name_ = this->get_parameter("subscriber_name").as_string();

  _can.interface_type  = this->get_parameter("can_interface.interface_type").as_string();
  _can.port            = this->get_parameter("can_interface.port").as_string();
  _can.interface_name  = this->get_parameter("can_interface.interface_name").as_string();
  _can.bitrate         = this->get_parameter("can_interface.bitrate_code").as_int();
  _can.baudrate        = this->get_parameter("can_interface.baudrate").as_int();
  _can.txqueuelen      = this->get_parameter("can_interface.txqueuelen").as_int();

  read_cmd_response_delay_ = this->get_parameter("read_cmd_response_delay").as_double();
  read_set_zero_response_delay_ = this->get_parameter("read_set_zero_response_delay").as_double();
  motor_state_update_freq_ = this->get_parameter("state_update_frequency").as_int();

  num_of_motors_ = this->get_parameter("num_of_motors").as_int();

  uint8_t num_of_childs = sizeof(param_child)/sizeof(param_child[0]);
  char param[25];
  for(int i=0; i<num_of_motors_; i++){
    for(int j=0; j<num_of_childs; j++){
      switch (j)
      {
      case 0: // id
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].id = this->get_parameter(param).as_int();
        break;
      case 1: // max_p
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].ctrl_param.p_des.max = this->get_parameter(param).as_double();
        motor[i].ctrl_param.p_des.min = -this->get_parameter(param).as_double();
        break; 
      case 2: // max_v
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].ctrl_param.v_des.max = this->get_parameter(param).as_double();
        motor[i].ctrl_param.v_des.min = -this->get_parameter(param).as_double();
        break;
      case 3: // max_kp
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].ctrl_param.kp.max = this->get_parameter(param).as_double();
        motor[i].ctrl_param.kp.min = 0;
        break;
      case 4: // max_kd
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].ctrl_param.kd.max = this->get_parameter(param).as_double();
        motor[i].ctrl_param.kd.min = 0;
        break;
      case 5: // max_iff
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].ctrl_param.i_ff.max = this->get_parameter(param).as_double();
        motor[i].ctrl_param.i_ff.min = -this->get_parameter(param).as_double();
        break;
      case 6: // limit_p -> to limit operatable max/min position
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].limit.p_min = this->get_parameter(param).as_double_array()[0];
        motor[i].limit.p_max = this->get_parameter(param).as_double_array()[1];
        break;
      case 7: // limit_v -> to limit operatable max/min velocity
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].limit.v_max = this->get_parameter(param).as_double();
        motor[i].limit.v_min = -motor[i].limit.v_max;
        break;
      case 8: // limit_v -> to limit operatable max/min velocity
        sprintf(param, "m%i_param.%s", i, param_child[j].c_str());
        motor[i].limit.i_max = this->get_parameter(param).as_double();
        motor[i].limit.i_min = -motor[i].limit.i_max;
        break;
      default:
        break;
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Loaded parameters: \n\tcan iface type: %s, \n\tcan_port: %s\n\tcan_iface_name: %s",
                                  _can.interface_type.c_str(), _can.port.c_str(), _can.interface_name.c_str());
}

void MiniCheetahMotorController::init_can()
{
  // select can device
  if(_can.interface_type == "SLCAND"){init_slcan();}
  else if (_can.interface_type == "CAN"){}

  // Initialize CAN Socket and check errors
  if (_can.status.is_can_initialized && (can.s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Socket");
    RCLCPP_ERROR(this->get_logger(), "\033[0;31mFailed to open socket\033[0m");
    exit(0);
  }
  else{
    RCLCPP_INFO(this->get_logger(), "\033[0;32mSocket is opened sucessfully\033[0m");
    strcpy(can.ifr.ifr_name, _can.interface_name.c_str());
    ioctl(can.s, SIOCGIFINDEX, &can.ifr);
    
    memset(&can.addr, 0, sizeof(can.addr));
    can.addr.can_family = AF_CAN;
    can.addr.can_ifindex = can.ifr.ifr_ifindex;

    sleep(2);
    if (bind(can.s, (struct sockaddr *)&can.addr, sizeof(can.addr)) < 0) {
		perror("Bind");
    RCLCPP_ERROR(this->get_logger(), "\033[0;31mFailed to bind the socket\033[0m");
		exit(0);
	}
    else{
      RCLCPP_INFO(this->get_logger(), "\033[0;32mSuccessfully binded the Socket\033[0m");
      _can.status.is_socket_initialzed=true;
    }
  }
}

void MiniCheetahMotorController::init_slcan()
{
  // run the bash script to enable can interface
  RCLCPP_INFO(this->get_logger(), "\033[0;33mInitialzing %s interface as %s ....\033[0m", _can.interface_type.c_str(), _can.interface_name.c_str());
  pkg_share_dir = ament_index_cpp::get_package_share_directory(PKG_NAME);
  slcan_bringup_file_dir = pkg_share_dir + "/bash_scripts/slcan.bash";
  char* can_bringup_default_cmd;
  can_bringup_default_cmd = new char[255];
  sprintf(can_bringup_default_cmd, "%s", slcan_bringup_file_dir.c_str());
  char* total_cmd;
  total_cmd = new char[500];
  char* cmd_options;
  cmd_options = new char[255];
  sprintf(cmd_options, "-s%i -S%i -t%i --can-name %s --dev-path %s", _can.bitrate, _can.baudrate, _can.txqueuelen, _can.interface_name.c_str(), _can.port.c_str());
  sprintf(total_cmd, "%s %s", can_bringup_default_cmd, cmd_options);
  // RCLCPP_INFO(this->get_logger(), "%s", total_cmd);
  int is_can_bringup = system(total_cmd);

  // check errors
  if(is_can_bringup == 0)
  {
    RCLCPP_INFO(this->get_logger(), "\033[0;32m%s interface is successfully initialized as %s\033[0m", _can.interface_type.c_str(), _can.interface_name.c_str());
    _can.status.is_can_initialized=true;
    delete [] can_bringup_default_cmd;
  }
  else if (is_can_bringup == 256)
  {
    RCLCPP_ERROR(this->get_logger(), "\033[0;31mFailed to initialize %s.  %s can not be found!\033[0m ", _can.interface_type.c_str(), _can.port.c_str());
    delete [] can_bringup_default_cmd;
    exit(1);
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "\033[0;31mFailed to execute slcan.bash script.\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[0;33mPlease give permission to slcan.bash file by running;\033[0m \033[0:36msudo chmod a+x %s\033[0m", can_bringup_default_cmd );
    // delete allocated memory
    delete [] can_bringup_default_cmd;
    can_bringup_default_cmd = nullptr;
    delete [] cmd_options;
    cmd_options = nullptr;
    delete [] total_cmd;
    total_cmd = nullptr;
    exit(1);
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
  // uint8_t tx_data[9];
  // tx_data[0] = 100;
  // size_t written_bytes = serial_.self.write(tx_data, sizeof(tx_data));
  // RCLCPP_INFO(this->get_logger(), "\033[0;32mserial wrote %i bytes: |%i|%i|%i|%i|%i|%i|%i|%i|%i|\033[0m", written_bytes,
  //                                                         tx_data[0], tx_data[1], tx_data[2], tx_data[3],
  //                                                         tx_data[4], tx_data[5], tx_data[6], tx_data[7], tx_data[8]);

  // uint8_t rx_data[9];
  // memset(rx_data, 0, sizeof(rx_data));
  // size_t bytes_read = serial_.self.read(rx_data, 9);
  // RCLCPP_INFO(this->get_logger(), "\033[0;36mserial read %i bytes:  |%i|%i|%i|%i|%i|%i|%i|%i|%i|\033[0m", 
  //                                                         bytes_read,
  //                                                         rx_data[0], rx_data[1], rx_data[2], rx_data[3],
  //                                                         rx_data[4], rx_data[5], rx_data[6], rx_data[7], rx_data[8]);
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





