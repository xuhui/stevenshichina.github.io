---
title: ROS中级一 配置并使用ROS导航功能包集
date: 2017-07-10 09:41:05
categories: ROS
tags: ROS navigation
comments: true
---
本篇学习如何配置 ROS 的 Navigation 功能包以及如何让导航功能在机器人上运行起来。先来看一下 ROS 导航架构图：

![](ros-intermediate-tutorial-1/overview_navigation.png)

<!--more-->
# 硬件要求
ROS 的二维导航功能包 navigation 根据输入的里程计消息以及激光雷达等传感器信息通过内部导航算法计算出安全的机器人速度控制指令，移动机器人到指定位置，从而完成既定的导航任务。尽管导航功能包设计的尽量通用，但仍然对机器人的硬件有一定的要求：
a,导航功能包目前只对差分或全向轮式机器人起作用，并且假定机器人可以使用速度指令进行控制，速度指令格式遵循x方向、y方向以及角度。
b,导航功能包要求机器人必须安装有激光雷达或深度相机等二维平面测距设备。
c,导航功能包起初以正方形机器人为模型开发，因此对正方形的机器人支持较好，对于其他形状的机器人也支持，但可能表现不佳。
# 导航架构分析
从上图中可以看出， navigation 功能包集的输入有 传感器 tf 变换 (sensor transforms)、里程计 (odometry source) 以及激光雷达或者点云数据 (sensor sources)，输出是 cmd_vel 速度消息 (base controller)。框内部分为 navigation 的核心 move_base 功能，它负责规划整个导航所需的流程，它是导航所必须的，灰色框内部分是可选的，蓝色框内是需要我们提供的组件。一般在 ROS 中进行导航功能使用到的三个包分别为 move_base、gmapping、amcl; move_base 根据获取的传感器消息进行路径规划，并移动机器人到达指定位置；gmapping 根据获取的激光雷达数据或者深度相机建立地图；amcl可以根据已有地图进行机器人的定位。
# 机器人配置
## TF变换
导航功能包集需要机器人不断的使用 [tf](http://wiki.ros.org/tf) 发布机器人的坐标系之间的变换关系，关于这一块可以参考 [TF配置](http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup/TF) 也可以参考本站文章 {% post_link ros-primary-tutorial-14 ROS初级十四 tf配置%}。
## 传感器信息(sensor source)
机器人要想避开现实环境中的障碍物离不开传感器信息的实时获取，这里的传感器我们假定为激光雷达或点云数据，传感器需要不断的发布 sensor_msgs/LaserScan 或者 sensor_msgs/PointCloud 消息。关于如何发布这些消息可以参考站内文章 {% post_link ros-primary-tutorial-16 ROS初级十六发布传感器数据流 %} 或者参考 [在ROS上发布传感器数据流](http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup/Sensors)。
## 里程计消息(odometry source)
ROS 导航功能包集在正确的导航之前需要获取底层传过来的里程计消息，里程计消息使用 [tf](http://wiki.ros.org/tf) 和 nav_msgs/Odometry 发布，关于如何发布可参考 [Publishing Odometry Information over ROS](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom) 也可参考站内文章 {% post_link ros-primary-tutorial-15 ROS初级十五 发布里程计消息 %}。
## 基座控制器(base controller)
ROS 导航功能包集 navigation 可以通过 cmd_vel 话题发布 geometry_msgs/Twist 类型的消息，这个消息基于机器人的基座坐标系 base_link ,它负责将运动命令传递给移动基座。也就是必须有一个节点负责定制 cmd_vel 话题，并将该话题上的速度(vx, vy, vtheta)指令转化为电机指令(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)发送到移动底座，使机器人按既定要求移动。
## 地图(map_server)
地图并不是 ROS 导航功能包集所必须的。
# 导航功能包集配置
假设上述需要的所有环境配置都已完成，机器人使用 [tf](http://wiki.ros.org/tf)发布坐标变换，并从传感器接收 sensor_msgs/LaserScan 或者 sensor_msgs/PointCloud 消息以便于导航，同时需要使用 [tf](http://wiki.ros.org/tf) 和 nav_msgs/Odometry 消息发布导航消息，消息以命令形式发布给移动底座，从而控制机器人到既定位置。
## 创建一个软件包
首先创建一个软件包用于保存我们需要的所有配置文件以及启动文件。创建功能包时需要添加一些依赖项，包括机器人配置中所使用到的功能包，其中 move_base 是必须添加的依赖项：
   ```
 $ catkin_create_pkg my_robot_name_2dnav move_base my_tf_configuration_dep my_odom_configuration_dep my_sensor_configuration_dep
   ```
使用时将后面的依赖项修改为自己机器人配置所需的依赖，这是我自己的机器人所有使用的依赖项：
   ```
 $ catkin_create_pkg my_robot_2dnav move_base roscpp tf geometry_msgs sensor_msgs nav_msgs
   ```
## 创建机器人启动配置文件
创建一个ROS launch 配置文件以及发布机器人所需的 tf 变换树。新建文件 my_robot_configuration.launch 并放到我们的软件包的launch目录下:
   ```
 $ roscd my_robot_2dnav
 $ mkdir launch
 $ vim my_robot_configuration.launch
   ```
添加以下内容到文件 my_robot_configuration.launch 中：
   ```
 <launch>
       <node pkg="sensor_node_pkg" type="sensor_node_type" name="sensor_node_name" output="screen">
           <param name="sensor_param" value="param_value" />
       </node>
       <node pkg="odom_node_pkg" type="odom_node_type" name="odom_node" output="screen">
           <param name="odom_param" value="param_value" />
       </node>
  
       <node pkg="transform_configuration_pkg" type="transform_configuration_type" name="transform_configuration_name" output="screen">
          <param name="transform_configuration_param" value="param_value" />
      </node>
  </launch>

   ```
这里只是一个launch模板，接下来我们根据自己的机器人去完善修改它。上面代码会首先启动机器人运行导航功能包所需的所有传感器，实际使用时需要将我们自己的传感器对应的ROS驱动包替换 sensor_node_pkg，将自己的传感器类型替换 sensor_node_type，通常与节点名一致；用自己的传感器节点名替换 sensor_node_name, sensor_param 包含所有必须的参数，如果有多个传感器，在这里一起启动它们。接下来，我们启动移动底座的里程计，同样将我们自己的里程计替换pkg,type,name,并根据实际情况指定相关参数。最后启动相应的 tf 变换，同样将我们自己的 tf 变换替换对应的 pkg,type,name 根据实际情况修改相关参数。
## 代价地图配置(local_costmap & global_costmap)
导航功能包集需要两个代价地图来保存现实环境中的障碍物信息，这两个代价地图分别为 local_costmap 和 global_costmap。前者用于局部路径规划与避障，后者用于全局的路径规划。有些参数是两个代价地图都必须的，也有一些参数为各自所有。对于代价地图，有三个配置项分别为 common 配置项、 global 配置项以及 local 配置项。如果想看完整的配置，请参阅 [costmap_2d](http://wiki.ros.org/costmap_2d)。
### 共同配置common
导航功能包集使用代价地图存储障碍物信息，我们需要指出要监听的传感器话题，以便于更新数据。创建一个名为 costmap_common_params.yaml 的文件，添加以下内容：

   ```
 obstacle_range: 2.5
 raytrace_range: 3.0
 footprint: [[x0, y0], [x1, y1], ... [xn, yn]]
 #robot_radius: ir_of_robot
 inflation_radius: 0.55

 observation_sources: laser_scan_sensor point_cloud_sensor

 laser_scan_sensor: {sensor_frame: frame_name, data_type: LaserScan, topic: topic_name, marking: true, clearing: true}

 point_cloud_sensor: {sensor_frame: frame_name, data_type: PointCloud, topic: topic_name, marking: true, clearing: true}

   ```
obstacle_range 参数决定了引入障碍物到代价地图的传感器读数的最大范围，此处我们设置为 2.5m，这意味着机器人只会更新以其底盘中心为半径2.5m以内的障碍物信息。raytrace_range 参数确定空白区域内光线追踪的范围，此处我们设置为3.0m，意味着机器人将试图根据传感器读数清除其前面3.0m远的空间。
footprint 为机器人的形状设置，如果是圆形可以直接指定机器人半径 robot_radius。当指定 footprint 时，机器人的中心默认是在(0.0,0.0)，支持顺时针和逆时针方向。inflation_radius 为代价地图膨胀半径，膨胀半径应该设置为障碍物产生代价的最大距离，此处设为0.55意味着机器人所有路径与障碍物保持0.55米或更远的距离。
observation_sources 参数定义了一系列传递空间信息给代价地图的各种传感器。frame_name 参数应设置为传感器坐标帧的名称， data_type 参数应设置为 LaserScan 或 PointCloud，这取决于主题使用的消息， topic_name 应该设置为发布传感器数据的主题的名称。  marking 和 clearing 参数确定传感器是否用于向代价地图添加障碍物信息，或从代价地图清除障碍信息，或两者都有。 
### 全局配置 global_costmap
新建一个文件命名为 global_costmap_params.yaml 用于存储全局代价地图配置选项，添加以下内容：

   ```
global_costmap:
  global_frame: /map
  robot_base_frame: base_link
  update_frequency: 5.0
  static_map: true

   ```
global_frame 参数定义了全局代价地图运行所在的坐标系。在这种情况下,我们会选择/map坐标系。 robot_base_frame 参数定义了代价地图参考的的机器人移动底盘的坐标系。update_frequency 参数决定了代价地图更新的频率。 static_map 参数决定代价地图是否根据 map_server 提供的地图初始化。如果不打算使用现有的地图，将其设为false。 
### 本地配置 local_costmap
新建一个文件命名为 localal_costmap_params.yaml，添加以下内容：
   ```
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 6.0
  height: 6.0
  resolution: 0.05
   ```
global_frame，robot_base_frame，update_frequency，static_map参数与全局配置意义相同。publish_frequency 参数决定了代价地图发布可视化信息的频率。 rolling_window 参数设置为true，意味着随着机器人在现实环境中移动，代价地图会保持以机器人为中心。 width 、height、resolution 参数分别设置局部代价地图的宽度(米)、高度(米)和分辨率(米/单元)。 这里的分辨率和静态地图的分辨率可能不同，但我们通常把他们设成一样的。
## BaseLocalPlanner 配置
Base_Local_Planner 负责根据全局路径规划计算速度命令并发送到机器人移动底座，根据自己的机器人规格配置以下参数以便于正常启动和运行机器人。新建一个文件命名为 base_local_planner_params.yaml，添加以下内容：
   ```
TrajectoryPlannerROS:
  max_vel_x: 0.45
  min_vel_x: 0.1
  max_vel_theta: 1.0
  min_in_place_vel_theta: 0.4

  acc_lim_theta: 3.2
  acc_lim_x: 2.5
  acc_lim_y: 2.5

  holonomic_robot: true
   ```
前面为机器人的速度限制，后面为机器人的加速度限制。
## 创建一个 launch 启动文件
当做好前面所有的配置文件后，我们需要一个launch 文件用于一起启动它们，新建一个文件命名为 move_base.launch 添加以下内容：
   ```
<launch>
  <master auto="start"/>

  <!-- Run the map server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find my_map_package)/my_map.pgm my_map_resolution"/>

  <!--- Run AMCL -->
  <include file="$(find amcl)/examples/amcl_omni.launch" />

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find my_robot_name_2dnav)/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find my_robot_name_2dnav)/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find my_robot_name_2dnav)/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find my_robot_name_2dnav)/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find my_robot_name_2dnav)/base_local_planner_params.yaml" command="load" />
  </node>
</launch>
   ```
需要修改的地方是更改地图服务器使指向自己的地图，如果是差分驱动的机器人，将 amcl_omni.launch 改为 amcl_diff.launch 。
## AMCL配置
AMCL 有许多配置选项影响定位的性能，详细可参考 [amcl](http://wiki.ros.org/amcl)
# 运行
启动两个终端分别运行：
   ```
roslaunch my_robot_configuration.launch
roslaunch move_base.launch
   ```
通过图形化显示给导航功能包发送一个目标信息，可参考[rviz and navigation tutorial](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack)，使用代码给导航功能包集发送导航目标，参考 [sending simple navigation goals](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals)

参考：
英文 [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup) 
