---
title: ROS初级十六 发布传感器数据流
date: 2017-07-07 13:41:15
categories: ROS
tags: ROS sensor
comments: true
---
传感器数据发布的正确与否直接影响到 ROS 导航功能的安全和性能。如果没有正确获取传感器的数据，机器人将会迷路或撞上障碍物。有很多传感器为机器人导航提供可用的信息，比如激光雷达、摄像头、声呐、红外以及碰撞传感器等。目前ROS的导航功能包仅支持激光雷达以及点云数据也就是 sensor_msgs/LaserScan 和 sensor_msgs/PointCloud 两种消息类型。本篇学习如何发布导航需要的这两种传感器数据消息。
<!--more-->
# ROS 消息头信息
不管是 sensor_msgs/LaserScan 消息还是 sensor_msgs/PointCloud 消息，都和ROS中的其他消息一样包含  [tf](http://wiki.ros.org/tf) 框架以及时间依赖信息。为了标准化这些消息，ROS中提供了 Header 消息类型用于标识诸如此类消息的起始部分。以下消息展示了 Header 类型的三个域，可以参考 [std_msgs/Header.msg](http://docs.ros.org/api/std_msgs/html/msg/Header.html)。 seq 标识消息的顺序号，自动累加；stamp 用于存储和消息相关的时间信息即时间戳，它必须与数据产生的时间相符； frame_id 用于标识传感器数据采集的参考坐标系，比如激光雷达的数据必须与它的坐标参考系相符。
   ```
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
   ```
# 发布激光雷达消息
## 激光雷达消息结构
ROS在 [sensor_msgs](http://wiki.ros.org/sensor_msgs) 中提供一个特殊的消息类型专门用于存储机器人的激光雷达数据叫做 LaserScan。LaserScan 的消息类型为虚拟的激光雷达数据采集提供了方便。在讨论如何发布激光雷达消息之前，我们先看一下激光雷达消息的格式，参考 [sensor_msgs/LaserScan.msg](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html):
   ```
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
   ```
为了更进一步理解激光雷达消息，接下来我们写一些代码介绍如何发布激光雷达消息。
## 代码实现发布LaserScan消息
新建包 laser_scan_publisher_tutorial，依赖项为 roscpp sensor_msgs
   ```
 $ cd ~/catkin_ws/src
 $ catkin_create_pkg laser_scan_publisher_tutorial sensor_msgs roscpp
   ```
在 src 目录下新建文件 laser_scan_publisher.cpp 添加以下内容：
   ```
 #include <ros/ros.h>
 #include <sensor_msgs/LaserScan.h>
  
 int main(int argc, char** argv){
    ros::init(argc, argv, "laser_scan_publisher");
  
    ros::NodeHandle n;
     //定义发布器 scan_pub 消息类型为sensor_msgs::LaserScan
     ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
 
    //虚拟一些激光雷达的数据
    //在真实激光雷达中要替换这些数据使用真实数据
    unsigned int num_readings = 100;//每圈扫描点个数
    double laser_frequency = 40;//扫描频率
    double ranges[num_readings];//范围
    double intensities[num_readings];//存储 激光雷达的强度数据
 
    int count = 0;
    ros::Rate r(1.0);
    while(n.ok()){
      //generate some fake data for our laser scan
      for(unsigned int i = 0; i < num_readings; ++i){
        ranges[i] = count;
        intensities[i] = 100 + count;//模拟一些数据
      }
      ros::Time scan_time = ros::Time::now();
 
      //populate the LaserScan message
      //填充 LaserScan 消息结构
      sensor_msgs::LaserScan scan;
      scan.header.stamp = scan_time;//时间戳
      scan.header.frame_id = "laser_frame";//参考坐标系
      scan.angle_min = -1.57;//扫描起始角度
      scan.angle_max = 1.57;//扫描结束角度
      scan.angle_increment = 3.14 / num_readings;//角度递增
      scan.time_increment = (1 / laser_frequency) / (num_readings);//时间递增
      scan.range_min = 0.0;//最小的扫描范围 单位为m
      scan.range_max = 100.0;//最大的扫描范围
 
      scan.ranges.resize(num_readings);//暂未理解
      scan.intensities.resize(num_readings);
      for(unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
      }
      //发布激光雷达扫描数据
      scan_pub.publish(scan);
      ++count;
      r.sleep();
    }
  }

   ```
修改CMakeLists.txt添加以下项：
   ```
 add_executable(laser_scan_publisher src/laser_scan_publisher.cpp)
 if(sensor_msgs_EXPORTED_TARGETS)
    add_dependencies(laser_scan_publisher ${sensor_msgs_EXPORTED_TARGETS})
 endif()
 target_link_libraries(laser_scan_publisher
    ${catkin_LIBRARIES}
  )
 install(TARGETS laser_scan_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

   ```
编译：
   ```
 $ cd ~/catkin_ws
 $ catkin_make
   ```
# 发布点云消息数据
## 点云消息数据结构
ROS 提供 sensor_msgs/PointCloud 消息类型用于存储和共享点云数据。它支持三维空间的点云数据，并且支持通道设置，比如可以设置一个强度通道用于存储这些点的数值强度，以下为点云消息的数据结构，参考[sensor_msgs/PointCloud.msg](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html):
   ```
# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
   ```
## 代码实现点云消息数据的发布
新建包 point_cloud_publisher_tutorial 依赖项为 sensor_msgs roscpp：
   ```
 $ catkin_create_pkg point_cloud_publisher_tutorial sensor_msgs roscpp
   ```
在 src 目录下新建文件 point_cloud_publisher.cpp 添加以下内容：
   ```
 #include <ros/ros.h>
 #include <sensor_msgs/PointCloud.h>
  
 int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_publisher");
  
    ros::NodeHandle n;
    //创建ROS发布器 名称为 cloud_pub 消息类型为sensor_msgs::PointCloud
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);
 
    unsigned int num_points = 100;//点的个数
 
    int count = 0;
    ros::Rate r(1.0);//发布频率
    while(n.ok()){
      //定义点云消息数据cloud
      sensor_msgs::PointCloud cloud;
      cloud.header.stamp = ros::Time::now();//时间戳
      cloud.header.frame_id = "sensor_frame";//参考坐标系
 
      cloud.points.resize(num_points);//设置点云数据的点数
 
      //we'll also add an intensity channel to the cloud
      cloud.channels.resize(1);//添加一个强度通道
      cloud.channels[0].name = "intensities";
      cloud.channels[0].values.resize(num_points);//设置通道的宽度与点数匹配
 
      //模拟填充一些点云数据并且填充强度通道数据
      //generate some fake data for our point cloud
      for(unsigned int i = 0; i < num_points; ++i){
        cloud.points[i].x = 1 + count;
        cloud.points[i].y = 2 + count;
        cloud.points[i].z = 3 + count;
        cloud.channels[0].values[i] = 100 + count;
      }
 
      //发布点云数据
      cloud_pub.publish(cloud);
      ++count;
      r.sleep();
    }
  }

   ```
修改 CMakeLists.txt 添加以下项：
 ```
add_executable(point_cloud_publisher src/point_cloud_publisher.cpp)
if(sensor_msgs_EXPORTED_TARGETS)
     add_dependencies(point_cloud_publisher ${sensor_msgs_EXPORTED_TARGETS})
endif()
target_link_libraries(point_cloud_publisher
    ${catkin_LIBRARIES}
  )
install(TARGETS point_cloud_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
 ```
回到 ~/catkin_ws 即可编译。
参考 [Publishing Sensor Streams Over ROS](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors)

