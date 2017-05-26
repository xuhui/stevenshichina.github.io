---
title: ROS 初级<二> 文件系统介绍
date: 2017-05-26 09:54:15
categories: ROS
tags: ROS File's System
comments: true
---
本篇学习 ROS 的文件系统，包括一些命令行工具 [roscd](http://wiki.ros.org/rosbash#roscd)、 [rosls](http://wiki.ros.org/rosbash#rosls)、 [rospack](http://wiki.ros.org/rospack) 等的使用。
# 安装 tutorial 程序包：
   ```
 $sudo apt-get install ros-indigo-ros-tutorials
   ```
源码位于: [ros_tutorials](https://github.com/ros/ros_tutorials.git)。
 一般一个 ROS 软件包中包括 *CMakeLists.txt*、  *include*、   *package.xml*、  *src* 等几个文件及文件目录。Packages 软件包是ROS 应用程序代码的组织单元，每个软件包都可以包含程序库、可执行文件、脚本或者其它手动创建的东西。 Manifest (package.xml): 清单，是对于软件包相关信息的描述,用于定义软件包相关元信息之间的依赖关系，这些信息包括版本、维护者和许可协议等。 
<!--more-->
# 文件系统工具
## [rospack](http://wiki.ros.org/rospack)
 [rospack](http://wiki.ros.org/rospack) 命令允许获取 ROS 软件包的信息，可以通过 *rospack help* 查看它的使用方法：
   ```
 $rospack help
 USAGE: rospack <command> [options] [package]
  Allowed commands:
    help
    cflags-only-I     [--deps-only] [package]
    cflags-only-other [--deps-only] [package]
    depends           [package] (alias: deps)
    depends-indent    [package] (alias: deps-indent)
    depends-manifests [package] (alias: deps-manifests)
    depends-msgsrv    [package] (alias: deps-msgsrv)
    depends-on        [package]
    depends-on1       [package]
    depends-why --target=<target> [package] (alias: deps-why)
    depends1          [package] (alias: deps1)
    export [--deps-only] --lang=<lang> --attrib=<attrib> [package]
    find [package]
    langs
    libs-only-L     [--deps-only] [package]
    libs-only-l     [--deps-only] [package]
    libs-only-other [--deps-only] [package]
    list
    list-duplicates
    list-names
    plugins --attrib=<attrib> [--top=<toppkg>] [package]
    profile [--length=<length>] [--zombie-only]
    rosdep  [package] (alias: rosdeps)
    rosdep0 [package] (alias: rosdeps0)
    vcs  [package]
    vcs0 [package]
  Extra options:
    -q     Quiets error reports.

 If [package] is omitted, the current working directory
 is used (if it contains a package.xml or manifest.xml).
   ```
 一般我们常用的 *rospack find xxx* 命令查找某个包位于何处，比如我们想知道 *turtlesim* 包位于哪里：
   ```
 $rospack find turtlesim
   ```
 系统会输出turtlesim 的路径 */opt/ros/indigo/share/turtlesim*
## [roscd](http://wiki.ros.org/rosbash#roscd) 
 [roscd](http://wiki.ros.org/rosbash#roscd) 是 [rosbash](http://wiki.ros.org/rosbash) 命令集中的一部分，它可以直接切换(cd)工作目录到某个软件包或者软件包集当中。 
 使用方法：
   ```
 $roscd [本地包名称[/子目录]]
   ```
 比如：
   ```
 $roscd beginner_tutorials/src
   ```
 直接进入到 *beginner_tutorials/src* 目录下，此时查看当前目录
   ```
 $pwd
 /home/yourusername/catkin_ws/src/beginner_tutorials/src
   ```
值得注意的是 [roscd](http://wiki.ros.org/rosbash#roscd) 只能切换到那些路径已经包含在 ROS_PACKAGE_PATH 环境变量中的软件包。
   ```
 $roscd log
   ```
roscd log可以切换到ROS保存日记文件的目录下,如果你没有执行过任何ROS程序，系统会报错说该目录不存在。 此时查看当前目录：
   ```
 $pwd
 /home/yourusername/.ros/log
   ```
## [rosls](http://wiki.ros.org/rosbash#rosls)
[rosls](http://wiki.ros.org/rosbash#rosls) 是 [rosbash](http://wiki.ros.org/rosbash) 命令集中的一部分,允许直接按软件包的名称而不是绝对路径执行ls命令,使用方法：
   ```
 $rosls [本地包名称[/子目录]]
   ```
比如在任意目录下执行，而不需要进入到 beginner_tutorials 所在目录：
   ```
 $rosls beginner_tutorials/
   ```
应输出：
   ```
CMakeLists.txt  include  msg  package.xml  src  srv
   ```
# TAB补全
ROS 命令同样支持TAB补全，这里不再细讲。