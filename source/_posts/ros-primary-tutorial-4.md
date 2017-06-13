---
title: ROS初级四 编译ROS程序包
date: 2017-05-26 15:10:04
categories: ROS
tags: Catkin_make
comments: true
---
# 编译
ROS 程序包的编译使用命令 [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)。在 catkin 工作空间的根目录下 *~/catkin_ws* 执行该命令即可编译 catkin 程序包：
   ```
 $cd ~/catkin_ws
 $catkin_make
   ```
<!--more-->
该命令会在 *~/catkin_ws* 目录下生成 *build * 和 *devel* 文件夹，*build *是 *cmake* 和 *make* 命令执行的地方，*devel* 文件夹内部用于保存编译生成文件，目标文件以及配置文件 setup.*sh等。该命令等价于：
  ```
 $ cd ~/catkin_ws
 $ cd src
 $ catkin_init_workspace
 $ cd ..
 $ mkdir build
 $ cd build
 $ cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
 $ make
  ```
如果只想编译指定的某个包或几个包可以使用以下命令：
  ```
 $catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"
  ```
比如我只想编译 mypackage 和 beginner_tutorial这两个包，那么命令可写成：
  ```
$catkin_make -DCATKIN_WHITELIST_PACKAGES="mypackage,beginner_tutorial"
  ```
注意命令中的 = 前后不能有空格。
输出：
  ```
Base path: /home/steven/catkin_ws
Source space: /home/steven/catkin_ws/src
Build space: /home/steven/catkin_ws/build
Devel space: /home/steven/catkin_ws/devel
Install space: /home/steven/catkin_ws/install

 Running command: "cmake /home/steven/catkin_ws/src -DCATKIN_WHITELIST_PACKAGES=mypackage,beginner_tutorial -DCATKIN_DEVEL_PREFIX=/home/steven/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/steven/catkin_ws/install -G Unix Makefiles" in "/home/steven/catkin_ws/build"

-- Using CATKIN_DEVEL_PREFIX: /home/steven/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/steven/catkin_ws/devel;/opt/ros/indigo
-- This workspace overlays: /home/steven/catkin_ws/devel;/opt/ros/indigo
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/steven/catkin_ws/build/test_results
-- Found gtest: gtests will be built
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.6.18
-- BUILD_SHARED_LIBS is on
-- Using CATKIN_WHITELIST_PACKAGES: mypackage,beginner_tutorial
-- Configuring done
-- Generating done
-- Build files have been written to: /home/steven/catkin_ws/build

 Running command: "make -j1 -l1" in "/home/steven/catkin_ws/build"

  ```
如果想恢复编译所有包使用命令：
  ```
 $catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```
# 安装
编译完成后生成的文件位于 *devel* 目录下，可以使用以下命令进行安装：
  ```
 $cd ~/catkin_ws
 $catkin_make install
  ```
以上命令等价于：
  ```
 $ cd ~/catkin_ws/build
 //If cmake hasn't already been called
 $ cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
 $ make
 $ make install
  ```
执行完以上命令后会在 *~/catkin_ws* 目录下生成 *install* 文件夹，所有编译完成的包都安装到了 *install* 下。里面包含 *setup.bash* 配置文件，可以使用 *source* 让系统可以找到这些编译完后的可执行包。
可以通过以下命令改变安装位置：
  ```
 $cd ~/catkin_ws
 $catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install
  ```
位置 */opt/ros/indigo* 为系统目录，一般不建议将自己的包安装于此。安装完后记得 *source* 一下才能使用：
  ```
 $source /opt/ros/indigo/setup.bash
  ```
或者：
 ```
 $source ~/catkin_ws/devel/setup.bash
  ```
可以将这句话添加到自己的 *~/.bashrc* 中：
  ```
 source ~/catkin_ws/devel/setup.bash
  ```
参考 [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)。