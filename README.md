# ros2_minicheetah_motor_controller

A ros2 node for Mini-Cheetah BLDC actuator.

## !!! NOT READY TO USE !!!

## !!! WORK IN PROGERSS !!! 

![](https://ae01.alicdn.com/kf/Habc6643ff4d34925bd65ab1aa82d594as/MIT-Mini-Cheetah-four-legged-robot-dog-servo-motor-joint-motor-modul-reducer-driver-robot-arm.jpg)



### create ros2 workspace 
```
    mkdir -p ros2_ws/src
    cd ros2_ws/src
```

### install pkgs
```
    git clone git@github.com:NDHANA94/ros2_minicheetah_motor_controller.git
    git clone https://github.com/RoverRobotics-forks/serial-ros2.git #clone ros2_serial
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
```