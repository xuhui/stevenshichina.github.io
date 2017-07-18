---
title: 通过串口控制移动底座
date: 2017-07-18 15:08:51
categories: ROS
tags: ROS serial
comments: true
---
移动底座与ROS上层的通信一般采用串口或者CAN接口，本次设计选用串口与ROS上层通信。Navigation Stack 中发布给 base_controller 的话题为 cmd_vel ，因此需要设计一个节点用于接收 cmd_vel 话题，获取该话题中的消息将其转换成移动底座可识别的速度及角速度指令，通过串口发送给移动底座，从而控制移动底座按既定要求运动。该节点还需要接收底座的通过串口上传过来的里程编码消息并转换成里程计消息发布到 ROS 上层，为 ROS 导航提供必须的里程计消息。
<!--more-->
# 订阅cmd_vel话题
设计一个节点，我们暂时命名为 my_serial_node ,在该节点中借助 [ros-serial](https://github.com/wjwwood/serial)实现串口的收发功能，并订阅话题 cmd_vel, 为了方便测试我们这里暂时订阅 /turtle1/cmd_vel 话题，详细看代码：
   ```
  #include <ros/ros.h>
  #include <serial/serial.h>
  #include <std_msgs/String.h>
  #include <std_msgs/Empty.h>
  #include <geometry_msgs/Twist.h>
 
  serial::Serial ser;
 
  //订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
      ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
      ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
      std::cout << "Twist Received" << std::endl;
  }
  int main (int argc, char** argv){
      ros::init(argc, argv, "my_serial_node");
      ros::NodeHandle nh;
 
      
      //订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
      ros::Subscriber write_sub = nh.subscribe("/turtle1/cmd_vel",1000,cmd_vel_callback);
      ros::Publisher read_pub = nh.advertise<std_msgs::String>("sensor", 1000);
 
      try
      {
          ser.setPort("/dev/ttyUSB0");
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
      }
      catch (serial::IOException& e)
      {
          ROS_ERROR_STREAM("Unable to open port ");
          return -1;
      }
 
      if(ser.isOpen()){
          ROS_INFO_STREAM("Serial Port initialized");
      }else{
         return -1;
      }

   ```
# 测试
   ```
 $ roscore &
 $ rosrun turtlesim turtle_teleop_key
   ```
在另外一个 terminal 中运行上面的节点:
   ```
$ rosrun my_serial_node my_serial_node
   ```
当用手按下上下左右方向键时会看到实时的打印速度信息：
![](control-mobile-base-by-serial/cmd_vel.jpg)

未完