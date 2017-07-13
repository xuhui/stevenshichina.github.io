---
title: ROS初级十七 URDF解读
date: 2017-07-8 09:19:45
categories: ROS
tags: URDF
comments: true
---
URDF的全称为 Unified Robot Description Format，通用机器人描述格式，它是一种特殊的 xml 格式文件，它能够抽象的描述现实中的机器人硬件，包括外形和组件。URDF 包括很多不同的功能包以及组件，它们之间的关系如图所描述：

![](ros-primary-tutorial-17/urdf_diagram.png)
<!--more-->

# 构建可视化机器人模型
开始之前先下载 [joint_state_publisher](http://wiki.ros.org/joint_state_publisher) 以及 [urdf_tutorial](http://wiki.ros.org/urdf_tutorial) 两个包。
   ```
 $ cd ~/catkin_ws/src/
 $ git clone  https://github.com/ros/urdf_tutorial.git
 $ git clone https://github.com/ros/robot_model.git
   ```
 [joint_state_publisher](http://wiki.ros.org/joint_state_publisher) 属于 robot_model 代码仓的一个 package ，需要注意的是 robot_model 有很多分支，我们下载后需要选择 indigo-devel 分支：
   ```
 $ cd ~/catkin_ws/src/robot_model/   
 $ git checkout indigo-devel
   ```
## 单个模型
先看一个简单的模型描述，在 urdf_tutorial/urdf 中的 01-myfirst.urdf 文件：
   ```
   <?xml version="1.0"?>
   <robot name="myfirst">
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.6" radius="0.2"/>
         </geometry>
       </visual>
     </link>
   </robot>

   ```
首先描述了机器人的名字 myfirst,该机器人仅有一个 link 名字为  base_link, 它包含一个 cylinder 长度为 0.6m，半径为 0.2m。
使用 Rviz 查看这个模型：
   ```
$ roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf
   ```
出现错误：
   ```
Traceback (most recent call last):
  File "/opt/ros/indigo/share/xacro/xacro.py", line 62, in <module>
    xacro.main()
  File "/opt/ros/indigo/lib/python2.7/dist-packages/xacro/__init__.py", line 673, in main
    f = open(args[0])
IOError: [Errno 2] No such file or directory: 'urdf/01-myfirst.urdf'
Invalid <param> tag: Cannot load command parameter [robot_description]: command [/opt/ros/indigo/share/xacro/xacro.py urdf/01-myfirst.urdf] returned with code [1]. 

Param xml is <param command="$(find xacro)/xacro.py $(arg model)" name="robot_description"/>
The traceback for the exception was written to the log file
   ```
错误提示貌似找不到 01-myfirst.urdf 这个文件，但事实上它是存在的，换一种方式打开：
   ```
 $ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'
   ```
如果进入到目录 ~/catkin_ws/src/urdf_tutorial/urdf_tutorial 下再用上述命令打开就不会出现错误：
   ```
 $ cd ~/catkin_ws/src/urdf_tutorial/urdf_tutorial
 $ roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf
   ```
图中所示，我们可以看到只有一个圆柱体存在：

![](ros-primary-tutorial-17/myfirsturdf.png)

需要注意的两点：
a,固定坐标系以网格中心作为参考点，它正是 base_link 的坐标系；
b,圆柱体的中心正好位于网格中心即 base_link原点，因此，圆柱体的另一半在网格下面。

## 多个模型 
一个模型就是一个 link 元素，添加多个模型就需要多个link，link 之间需要使用 joint 连接，joint 有固定于可动之分。下面看一个固定 joint 连接，文件位于 urdf_tutorial/urdf 中的 02-multipleshapes.urdf：
   ```
  <?xml version="1.0"?>
   <robot name="multipleshapes">
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.6" radius="0.2"/>
         </geometry>
       </visual>
     </link>
 
    <link name="right_leg">
      <visual>
        <geometry>
          <box size="0.6 0.1 0.2"/>
        </geometry>
      </visual>
    </link>
 
    <joint name="base_to_right_leg" type="fixed">
      <parent link="base_link"/>
      <child link="right_leg"/>
    </joint>
 
  </robot>
   ```
在圆柱体的基础上又定义了一个box，尺寸为 0.6m x 0.1m x 0.2m ,一个 joint 连接两个模型，joint 必须指定它所连接的两个模型的父link与子link,子 link 的位置依赖于父 link 的位置。
可以运行命令查看：
   ```
 $ roslaunch urdf_tutorial display.launch model:=urdf/02-multipleshapes.urdf 
   ```
图中可以看到两个模型的中心重合，因为他们的坐标原点一样，如果我们不希望他们重合，可以定义多个坐标系原点。
## 原点
上图腿的位置位于圆柱体的中间部分，下面我们调整它的位置，参见文件urdf_tutorial/urdf/03-origins.urdf
   ```
<?xml version="1.0"?>
   <robot name="origins">
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.6" radius="0.2"/>
         </geometry>
       </visual>
     </link>
 
    <link name="right_leg">
      <visual>
        <geometry>
          <box size="0.6 0.1 0.2"/>
        </geometry>
        <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      </visual>
    </link>
 
    <joint name="base_to_right_leg" type="fixed">
      <parent link="base_link"/>
      <child link="right_leg"/>
      <origin xyz="0 -0.22 0.25"/>
    </joint>
 
  </robot>

   ```
未完
参考：[urdf/tutorial](http://wiki.ros.org/urdf/Tutorials)
