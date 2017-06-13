---
title: ROS初级九 创建ROS消息和ROS服务
date: 2017-05-31 13:31:50
categories: ROS
tags: ROS Messages
comments: true
---
本篇学习如何创建并编译 ROS 消息和服务，以及 [rosmsg](http://wiki.ros.org/rosmsg) [rossrv](http://wiki.ros.org/srv) [roscp](http://wiki.ros.org/rosbash#roscp) [rosed](http://wiki.ros.org/rosbash#rosed) 等命令的使用。
<!--more-->
# rosed
[rosed](http://wiki.ros.org/rosbash#rosed) 是 [rosbash](http://wiki.ros.org/rosbash) 的一部分，相当于 ros + edit。利用 [rosed](http://wiki.ros.org/rosbash#rosed) 可以直接通过 package 名称来获取到待编辑的文件而无需指定该文件的存储路径，使用方法：
   ```
 rosed [package_name] [filename]
   ```
示例直接打开 beginner_tutorials 中的 talker.cpp文件：
   ```
 $ rosed beginner_tutoris talker.cpp
   ```
[rosed](http://wiki.ros.org/rosbash#rosed) 默认的编辑器是 [vim](http://www.vim.org/) ，可以将以下命令添加到 ~/.bashrc 文件中来修改默认的编辑器：
   ```
 export EDITOR=/usr/local/bin/vim
   ```
或者修改为使用 [emacs](https://www.gnu.org/software/emacs/) 为默认编辑器：
   ```
 export EDITOR='emacs -nw'
   ```
打开一个新的终端，看看定义的 EDITOR 是否正确:
   ```
 $ echo $EDITOR
   ```
# 消息和服务介绍
消息 msg 文件是一个描述 ROS 中所使用的消息类型的一个简单文本，它们会被用来生成不同语言的源代码。服务 srv 文件用于描述一项服务，它包含两部分，请求和响应。
msg 文件存放在 package 的 msg 目录下，srv 文件存放于package 的 srv 目录下。msg 文件每行声明一个数据类型和变量名，目前支持的数据类型有：
   ```
 int8, int16, int32, int64 (plus uint*) 
 float32, float64 
 string 
 time, duration 
 other msg files 
 variable-length array[] and fixed-length array[C] 
   ```
在 ROS 中有一个特殊的数据类型：Header，它含有时间戳和坐标系信息。在 msg 文件的第一行经常可以看到 Header header的声明。
msg 文件样例：
   ```
 Header header
 string child_frame_id
 geometry_msgs/PoseWithCovariance pose
 geometry_msgs/TwistWithCovariance twist
   ```
srv 文件分为请求和响应两部分，由'---'分隔。样例：
   ```
 int64 A
 int64 B
 ---
 int64 Sum
   ```
A 和 B 是请求，sum 是响应。
# msg 的使用
在之前创建的 beginner_tutorial 里定义新的消息：
   ```
 $ roscd beginner_tutorials/
 $ mkdir msg
 $ echo "int64 num" > msg/Num.msg
   ```
上面是最简单的例子在.msg文件中只有一行数据。当然，也可以仿造上面的形式多增加几行以得到更为复杂的消息：
   ```
 string first_name
 string last_name
 uint8 age
 uint32 score
   ```
接下来需要确保 msg 文件被转换成 C++,Python和其他语言的源代码，查看 package.xml：
   ```
 $ rosed beginner_tutorials  package.xml
   ```
确保它包含以下两条语句：
   ```
 <build_depend>message_generation</build_depend>
 <run_depend>message_runtime</run_depend>
   ```
在构建的时候，我们只需要"message_generation"。然而，在运行的时候，我们只需要"message_runtime"。 打开 CMakeLists.txt 文件，利用 find_packag 函数，增加对 message_generation 的依赖，这样就可以生成消息了。 可以直接在 COMPONENTS 的列表里增加 message_generation ：
   ```
 $ rosed beginner_tutorials CMakeLists.txt
   ```
   ```
// Do not just add this line to your CMakeLists.txt,  modify the existing line
 find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
   ```
即使没有调用 find_package, 也可以编译通过。这是因为 catkin 把所有的 package 都整合在一起，因此，如果其他的 package 调用了 find_package，你的 package 的依赖就会是同样的配置。但是，在单独编译这个 package 时，忘记调用 find_package 会很容易出错。确保设置了运行依赖项：
   ```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
   ```
找到如下代码块: 
   ```
// add_message_files(
//  FILES
//  Message1.msg
//   Message2.msg
// )
   ```
去掉注释符号，用前面定义的.msg文件替代Message*.msg
   ```
 add_message_files(
  FILES
  Num.msg
 )
   ```
手动添加.msg文件后，我们要确保CMake知道在什么时候重新配置我们的project。 确保添加了如下代码：
   ```
 generate_messages()
   ```
查看自定义的消息文件能否被 ROS 识别，使用方法：
   ```
 rosmsg show [message type]
   ```
示例：
   ```
 $ rosmsg show beginner_tutorials/Num 
 $ rosmsg show Num // 也可以省略掉 package 名
   ```
输出：
   ```
 int64 num
 string first_name
 string last_name
 uint8 age
 uint8 score
   ```
# srv 的使用
在 beginner_tutorials 中创建服务，我们从其他 ROS 包中拷贝一个 srv 文件：
   ```
 $ roscd beginner_tutorials
 $ mkdir srv
 $ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
   ```
确保 srv 文件被转换成C++ Python 和其他语言的源代码。CMakeLists.txt 文件中关于 message_generation 的配置对 msg 和 srv 都起作用：
   ```
 //Do not just add this line to your CMakeLists.txt, modify the existing line
 find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
   ```
修改 CMakeLists.txt 文件增加 srv 依赖项：
   ```
 add_service_files(
  FILES
  AddTwoInts.srv
 )
   ```
查看服务能否被 ROS 识别，使用方法：
   ```
 rossrv show <service type>
   ```
示例：
   ```
 $ rossrv show beginner_tutorials/AddTwoInts
 $ rossrv show AddTwoInts //也可以不指定 package 名字
   ```
输出：
   ```
 int64 a
 int64 b
 ---
 int64 sum

   ```
# 编译
通过上面为 beginner_tutorials 包添加的 msg 文件和 srv 文件，接下来就可以编译了。编译之前先在 CMakeLists.txt 文件中添加所有消息文件所依赖的那些含有 .msg 文件的 package ：
   ```
 generate_messages(
  DEPENDENCIES
  std_msgs
 )
   ```
编译：
   ```
 $ cd ~/catkin_ws/
 $ catkin_make
   ```
编译完成后，所有在 msg 路径下的 .msg 文件都将转换为 ROS 所支持语言的源代码。生成的 C++ 头文件将会放置在 ~/catkin_ws/devel/include/beginner_tutorials/目录下；Python脚本语言会在~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg 目录下创建。 lisp 文件会出现在 ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/ 路径下。详细格式参考 [Message_Description_Language](http://wiki.ros.org/ROS/Message_Description_Language) 。

参考 [CreatingMsgAndSrv](http://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv) 