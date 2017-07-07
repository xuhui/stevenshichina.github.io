---
title: ROS初级十五 发布里程计消息
date: 2017-07-07 08:40:30
categories: ROS
tags: ROS odometry
comments: true
---
ROS导航功能包使用 [tf](http://wiki.ros.org/tf) 来决定机器人在环境中的位置并根据传感器的数据生成静态地图。但是 [tf](http://wiki.ros.org/tf) 却不能提供机器人速度的任何信息。因此导航功能包需要里程计信息源能够发布包含速度信息的变换及里程计消息。本篇将学习 nav_msgs/Odometry 消息，并且通过代码学习如何通过 ROS 以及 [tf](http://wiki.ros.org/tf) 变换发布里程计消息。
<!--more-->
# nav_msgs/Odometry 消息
nav_msgs/Odometry 消息包含机器人在自由空间中的位置估计以及速度值。可以使用消息查看命令来查看 nav_msgs/Odometry 消息的详细信息：
   ```
 $ rosmsg show nav_msgs/Odometry
   ```
详细的介绍可参考 [nav_msgs/Odometry.msg](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) 
   ```
 # This represents an estimate of a position and velocity in free space.  
 # The pose in this message should be specified in the coordinate frame given by header.frame_id.
 # The twist in this message should be specified in the coordinate frame given by the child_frame_id
 Header header
 string child_frame_id
 geometry_msgs/PoseWithCovariance pose
 geometry_msgs/TwistWithCovariance twist
   ```
pose 参数包含机器人在里程计框架下的位置估计并附带一个可选的位姿估算协方差。 twist 参数包含机器人在子坐标系下的速度信息，并附带一个可选的速度估算协方差，这个子坐标系通常是机器人移动基座的坐标参考系。
# 使用 tf 发布里程计变换
[tf](http://wiki.ros.org/tf) 软件库负责维护机器人坐标系之间的变换树，因此任何里程计源都应该发布它管理的坐标系信息。这一部分的详细使用和说明请参考我的前一篇文章[ROS初级十四 tf 配置](http://stevenshi.me/2017/06/08/ros-primary-tutorial-14/#more)。
# 代码实现
下面我们通过编写一些简单的代码实现里程计消息以及利用 tf 实现变换，模拟一个机器人进行圆周运动。新建包 odometry_publisher_tutorial,依赖项为 tf 以及 nav_msgs: 
   ```
 $ catkin_create_pkg odometry_publisher_tutorial tf nav_msgs roscpp
   ```
新建cpp文件，并命名为 odometry_publisher.cpp,拷贝如下代码：
   ```
   #include <ros/ros.h>
   //包含 tf 以及 nav_msgs 相关的头文件
   #include <tf/transform_broadcaster.h>
   #include <nav_msgs/Odometry.h>
  
   int main(int argc, char** argv){
     ros::init(argc, argv, "odometry_publisher");
 
    ros::NodeHandle n;
 
    //创建一个publisher 命名为 odom_pub 消息类型为 nav_msgs::Odometry
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
 
    //创建对象 tf 广播变换
    tf::TransformBroadcaster odom_broadcaster;
    //假定机器人的起始位置位于odom坐标参考系的原点即0点
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
 
    //设定一个初始速度，在 odom 坐标参考系下
    double vx = 0.1;// x 方向 0.1m/s
    double vy = -0.1;//  y 方向 -0.1m/s
    double vth = 0.1;// 角速度为0.1rad/s
 
    ros::Time current_time, last_time;
    //获取系统当前时间
    current_time = ros::Time::now();
    last_time = ros::Time::now();
 
    //以 1HZ频率发布
    ros::Rate r(1.0);
    while(n.ok()){
      current_time = ros::Time::now();
 
      //计算里程计信息
      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
      double delta_th = vth * dt;
 
      //更新里程计信息
      x += delta_x;
      y += delta_y;
      th += delta_th;
 
      //以下为了兼容三维系统下的消息结构，将里程计的偏航角转换成四元数
      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
         //first, we'll publish the transform over tf
      //TransformStamped 类型为tf 发布时需要的类型
      geometry_msgs::TransformStamped odom_trans;
      //时间戳
      odom_trans.header.stamp = current_time;
      //父参考坐标系 id
      odom_trans.header.frame_id = "odom";
      //子参考系 id
      odom_trans.child_frame_id = "base_link";
      //我们希望发布从odom到base_link的变换，因此这两个坐标系的关系不要搞错
 
      //填充变换数据
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
 
      //发送变换
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
 
      //填充时间戳，发布nav_msgs/Odometry 里程计消息
      //以便于导航包可以获取速度信息
      //还需设置时间戳以及父子参考坐标系
      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
 
      //最后填充机器人的位置以及速度信息，
      //并且发布该信息，因为是机器人本体，
      //所以参考坐标系为 base_link
      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
 
      //set the velocity
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;
 
      //publish the message
      odom_pub.publish(odom);

     last_time = current_time;
     r.sleep();
   }
 }

   ```

# 编译
设置 CMakeLists.txt 添加依赖项：
   ```
add_executable(odometry_publisher src/odometry_publisher.cpp)
if(nav_msgs_EXPORTED_TARGETS)
	add_dependencies(odometry_publisher ${nav_msgs_EXPORTED_TARGETS})
endif()
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
install(TARGETS odometry_publisher
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
   ```
编译：
   ```
 $ cd ~/catkin_ws/
 $ catkin_make
   ```
参考：[Publishing Odometry Information Over ROS](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)