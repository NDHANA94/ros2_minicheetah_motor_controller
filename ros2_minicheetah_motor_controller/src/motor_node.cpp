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
  RCLCPP_INFO(this->get_logger(), "%sInitializing motor_node.%s", GREEN, NC);

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

  for(int i=0; i<num_of_motors_; i++){
    motor[i]->enable();
    RCLCPP_INFO(this->get_logger(), "can.frame.id: %u   || can.frame.data: |%u|%u|%u|%u|%u|%u|%u|%u|",can.tx_frame.can_id, can.tx_frame.data[0], can.tx_frame.data[1], can.tx_frame.data[2], can.tx_frame.data[3], can.tx_frame.data[4], can.tx_frame.data[5], can.tx_frame.data[6], can.tx_frame.data[7]);
  }

}

MiniCheetahMotorController::~MiniCheetahMotorController()
{
  // deallocate motor memory
  for(int i=0; i<num_of_motors_; i++){
    delete[] motor[i];
  }
  delete[]motor;
  motor = NULL;
  // close slcan
  RCLCPP_INFO(this->get_logger(), "%sclosing motor_node!%s", PURPLE, NC);
  char slcand_close_cmd[255];
  sprintf(slcand_close_cmd, "%s -r", slcan_bringup_file_dir.c_str());
  RCLCPP_INFO(this->get_logger(), "%s system cmd: %s %s", PURPLE, slcand_close_cmd, NC);
  int fb = system(slcand_close_cmd);
  RCLCPP_INFO(this->get_logger(), "%s system cmd fb: %i %s", PURPLE, fb, NC);
  RCLCPP_INFO(this->get_logger(), "%sCAN is closed!%s", PURPLE, NC);
  
  RCLCPP_INFO(this->get_logger(), "%smotor_node is closed!%s", PURPLE, NC);
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
  can_read();
  
}

// ------------------------------------------------

 // Make sure to declare parameters before using init_motors() method
void MiniCheetahMotorController::init_motors()
{
  motor = new Motor*[num_of_motors_];
  for(int i = 0; i<num_of_motors_; i++){
    // motor[i].set_can(&can);
    motor[i] = new Motor(&can);
  }
  RCLCPP_INFO(this->get_logger(), "%sallocated memory for %i motor objects.%s", GREEN, num_of_motors_, NC);
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
  this->declare_parameter("can_interface.rcvtimeo");
  this->declare_parameter("read_cmd_response_delay");
  this->declare_parameter("read_set_zero_response_delay");
  this->declare_parameter("state_update_frequency");
  this->declare_parameter("num_of_motors");
  // declare motor params for each motors
  num_of_motors_ = this->get_parameter("num_of_motors").as_int();
  uint8_t num_of_childs = sizeof(param_child)/sizeof(param_child[0]);
  char param[25];
  if(num_of_motors_ > 12){
    num_of_motors_ = 12;
    RCLCPP_WARN(this->get_logger(), "%sNumber of maximum motors are 12. Only first 12 motors will be initialized%s", YELLOW, NC);
  }
  for(int i=1; i<=num_of_motors_; i++){
    for(int j=0; j<num_of_childs; j++){
      sprintf(param, "m%i_params.%s", i, param_child[j].c_str());
      this->declare_parameter(param);
    }
  }
  RCLCPP_INFO(this->get_logger(), "%smotor_node ros-params are declared.%s", GREEN, NC);
}

void MiniCheetahMotorController::read_parameters(){
  subscriber_name_ = this->get_parameter("subscriber_name").as_string();
  _can.interface_type  = this->get_parameter("can_interface.interface_type").as_string();
  _can.port            = this->get_parameter("can_interface.port").as_string();
  _can.interface_name  = this->get_parameter("can_interface.interface_name").as_string();
  _can.bitrate         = this->get_parameter("can_interface.bitrate_code").as_int();
  _can.baudrate        = this->get_parameter("can_interface.baudrate").as_int();
  _can.txqueuelen      = this->get_parameter("can_interface.txqueuelen").as_int();
  _can.timeout_ms      = this->get_parameter("can_interface.rcvtimeo").as_int();
  read_cmd_response_delay_ = this->get_parameter("read_cmd_response_delay").as_double();
  read_set_zero_response_delay_ = this->get_parameter("read_set_zero_response_delay").as_double();
  motor_state_update_freq_ = this->get_parameter("state_update_frequency").as_int();
  num_of_motors_ = this->get_parameter("num_of_motors").as_int();

  uint8_t num_of_childs = sizeof(param_child)/sizeof(param_child[0]);
  char param[50];
  for(int i=0; i<num_of_motors_; i++){
    for(int j=0; j<num_of_childs; j++){
      switch (j)
      {
      case 0: // id
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_id(int(this->get_parameter(param).as_int()));
        break;
      case 1: // max_p
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_position_range(this->get_parameter(param).as_double());
        break; 
      case 2: // max_v
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_velocity_range(this->get_parameter(param).as_double());
        break;
      case 3: // max_kp
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_kp_range(this->get_parameter(param).as_double());
        break;
      case 4: // max_kd
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_kd_range(this->get_parameter(param).as_double());
        break;
      case 5: // max_iff
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_iff_range(this->get_parameter(param).as_double());
        break;
      case 6: // limit_p -> to limit operatable max/min position
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_limit_position(this->get_parameter(param).as_double_array());
        break;
      case 7: // limit_v -> to limit operatable max/min velocity
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_limit_velocity(this->get_parameter(param).as_double());
        break;
      case 8: // limit_v -> to limit operatable max/min velocity
        sprintf(param, "m%i_params.%s", i+1, param_child[j].c_str());
        motor[i]->set_limit_current(this->get_parameter(param).as_double());
        break;
      default:
        break;
      RCLCPP_INFO(this->get_logger(), "%sset %s%s", BLUE, param, NC);
      }
    }
    if(!motor[i]->is_status_set(MOTOR_PARAMS_SET)){
      RCLCPP_ERROR(this->get_logger(), "%s Failed to set motor%i params.%s", RED, i+1,  NC);
      exit(1);
    }
  }
  RCLCPP_INFO(this->get_logger(), "%smotor_node ros-params are successfully loaded.%s", GREEN, NC);
  
}

void MiniCheetahMotorController::init_can()
{
  // select can device
  if(_can.interface_type == "SLCAND"){init_slcan();}
  else if (_can.interface_type == "CAN"){}

  // Initialize CAN Socket and check errors
  can.s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 1
  if (_can.status.is_can_initialized && (can.s < 0))
  {
    perror("Socket");
    RCLCPP_ERROR(this->get_logger(), "%sFailed to open socket%s", RED, NC);
    exit(1);
  }
  else if(!_can.status.is_can_initialized){
    RCLCPP_ERROR(this->get_logger(), "%sCould not find a CAN interface.%s", RED, NC);
    exit(1);
  }
  
  RCLCPP_INFO(this->get_logger(), "%sSocket is opened sucessfully%s", GREEN, NC);
  strcpy(can.ifr.ifr_name, _can.interface_name.c_str()); // 2
  ioctl(can.s, SIOCGIFINDEX, &can.ifr); // 3
  
  memset(&can.addr, 0, sizeof(can.addr));
  can.addr.can_family = AF_CAN; // 4
  can.addr.can_ifindex = can.ifr.ifr_ifindex; // 5

  sleep(2);
  if (bind(can.s, (struct sockaddr *)&can.addr, sizeof(can.addr)) < 0) { // 6
    perror("Bind");
    RCLCPP_ERROR(this->get_logger(), "%sFailed to bind the socket to a CAN interface.%s", RED, NC);
    exit(1);
  }

  RCLCPP_INFO(this->get_logger(), "%sSocket is successfully bound to %s interface%s", GREEN, _can.interface_name.c_str(), NC);
  _can.status.is_socket_initialzed=true;
  
  // assign can_ptr of each initialized motors to the address of 'can' struct.
  // for(int i = 0; i<num_of_motors_; i++){
  //   motor[i].set_can(&can);
  // }
  RCLCPP_INFO(this->get_logger(), "%sCAN interface is connected with all the %i motor objects.%s", GREEN, num_of_motors_, NC);

  // set CAN DLC (payload length)
  can.tx_frame.can_dlc = 8;
  RCLCPP_INFO(this->get_logger(), "%sCAN payload size is set to %i Bytes%s", GREEN, can.tx_frame.can_dlc, NC);

  // set receive timeout
  can.tv.tv_sec = _can.timeout_ms/1000;
  can.tv.tv_usec = 0;
  setsockopt(can.s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&can.tv, sizeof(can.tv));
  RCLCPP_INFO(this->get_logger(), "%sCAN receive timeout (RCVTIMEO) is set to %ims%s", GREEN, can.tv.tv_sec, NC);

}

void MiniCheetahMotorController::init_slcan()
{
  // run the bash script to enable can interface
  RCLCPP_INFO(this->get_logger(), "%sInitialzing %s interface as %s%s", GREEN, _can.interface_type.c_str(), _can.interface_name.c_str(), NC);
  pkg_share_dir = ament_index_cpp::get_package_share_directory(PKG_NAME);
  slcan_bringup_file_dir = pkg_share_dir + "/bash_scripts/slcan.bash";
  char can_bringup_default_cmd[255];
  sprintf(can_bringup_default_cmd, "%s", slcan_bringup_file_dir.c_str());
  char total_cmd[655];
  char cmd_options[400];
  sprintf(cmd_options, "-s%i -S%i -t%i --can-name %s --dev-path %s", _can.bitrate, _can.baudrate, _can.txqueuelen, _can.interface_name.c_str(), _can.port.c_str());
  sprintf(total_cmd, "%s %s", can_bringup_default_cmd, cmd_options);
  // RCLCPP_INFO(this->get_logger(), "%s", total_cmd);
  int is_can_bringup = system(total_cmd);

  // check errors
  if(is_can_bringup == 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s%s interface is successfully initialized as %s%s", GREEN, _can.interface_type.c_str(), _can.interface_name.c_str(), NC);
    _can.status.is_can_initialized=true;
  }
  else if (is_can_bringup == 256)
  {
    RCLCPP_ERROR(this->get_logger(), "%sFailed to initialize %s.  %s can not be found!%s", RED, _can.interface_type.c_str(), _can.port.c_str(), NC);
    exit(1);
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "%sFailed to execute slcan.bash script.%s", RED, NC);
    RCLCPP_INFO(this->get_logger(), "%sPlease give permission to slcan.bash file by running;\033[0m \033[0:36msudo chmod a+x %s%s", YELLOW, can_bringup_default_cmd, NC);
    exit(1);
  }
}

int MiniCheetahMotorController::can_read()
{
  int nbytes = read(can.s, &can.rx_frame, sizeof(struct can_frame));
  if (nbytes < 0){
    perror("can raw socket read");
    RCLCPP_ERROR(this->get_logger(), "%sCAN raw socket read error.%s", RED, NC);
    return 1;
  }

  // paranoid check ...
  if(nbytes < int(sizeof(struct can_frame))){
    fprintf(stderr, "read: incomplete CAN frame\n");
    RCLCPP_WARN(this->get_logger(), "%sCAN READ: incomplete CAN frame.%s", RED, NC);
    return 1;
  }
  RCLCPP_INFO(this->get_logger(), "%scan received: id=%u , data: |%u|%u|%u|%u|%u|%u|%u|%u| %s", YELLOW, can.rx_frame.can_id,
                                                            can.rx_frame.data[0], can.rx_frame.data[1], can.rx_frame.data[2],
                                                            can.rx_frame.data[3], can.rx_frame.data[4], can.rx_frame.data[5],
                                                            can.rx_frame.data[6], can.rx_frame.data[7], NC);
  return 0;
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





