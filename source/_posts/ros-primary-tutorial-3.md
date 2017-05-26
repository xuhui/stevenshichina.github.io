---
title: ROS 初级<三> 创建ROS程序包
date: 2017-05-26 11:23:13
categories: ROS
tags: catkin_create_pkg
comments: true
---
本篇学习 ROS 程序包的创建，使用工具 [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) 创建新程序包。
# catkin 程序包的组成
 catkin 程序包必须包含 [package.xml](http://wiki.ros.org/catkin/package.xml) 文件和 [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt) 文件。*package.xml* 提供有关程序包的元信息，每个目录下只能有一个程序包：
   ```
 beginner_tutorials/     
  include/                   
  msg/                       
  src/                        
  srv/                       
  CMakeLists.txt              
  package.xml 
   ```
<!--more-->
# catkin工作空间
一个简单的 [catkin workspace](http://wiki.ros.org/catkin/workspaces) 结构如下：
   ```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
   ```
# 创建catkin程序包
使用 [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) 创建自己的 catkin 程序包，格式：
   ```
 $catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
   ```
实例：
   ```
 $cd ~/catkin_ws/src 
 $catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
   ```
上面命令创建了一个名为 *beginner_tutorials* 的 catkin 程序包，这个程序包依赖于 std_msgs、roscpp 和 rospy。执行这个命令后在 *~/catkin_ws/src* 目录下将会创建一个名为 *beginner_tutorials* 的文件夹，这个文件夹里面包含一个 *package.xml* 文件和一个 *CMakeLists.txt* 文件，这两个文件都已经自动包含了部分在执行 [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) 命令时提供的信息。 
# 程序包依赖关系
可以使用 *rospack* 命令来查看某个程序包的依赖包：
   ```
 $rospack depends1 beginner_tutorials //查看一级依赖
   ```
输出：
   ```
 roscpp
 rospy
 std_msgs
   ```
列出的依赖包正是我们在用 [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) 创建 beginner_tutorials 程序包时指定的依赖包。而这些依赖已经自动添加到 *package.xml* 文件中：
   ```
 $roscd beginner_tutorials
 $cat package.xml
   ```
package.xml内容：
   ```
 <package>
 ...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
 ...
 </package>
   ```
列出所有的间接依赖包：
   ```
 $rospack depends beginner_tutorials
 cpp_common
 rostime
 roscpp_traits
 roscpp_serialization
 genmsg
 genpy
 message_runtime
 rosconsole
 std_msgs
 rosgraph_msgs
 xmlrpcpp
 roscpp
 rosgraph
 catkin
 rospack
 roslib
 rospy
   ```
# 自定义程序包
修改 *package.xml*, 更新必要的信息。
描述标签：
   ```
<description>The beginner_tutorials package</description>
   ```
将描述信息修改为任何喜欢的内容，但是按照约定第一句话应该简短一些，因为它覆盖了程序包的范围。如果用一句话难以描述完全那就需要换行了。
维护者标签：
   ```

  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="user@todo.todo">user</maintainer> 
   ```
这是 *package.xml* 中要求填写的一个重要标签，因为它能够让其他人联系到程序包的相关人员。至少需要填写一个维护者名称，但如果有需要的话你可以添加多个。除了在标签里面填写维护者的名称外，还应该在标签的 *email* 属性中填写邮箱地址。
许可标签：
   ```
 <!-- One license tag required, multiple allowed, one license per tag -->
 <!-- Commonly used license strings: -->
 <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
 <license>TODO</license>
   ```
选择一种许可协议并将它填写到这里。一些常见的开源许可协议有BSD、MIT、Boost Software License、GPLv2、GPLv3、LGPLv2.1和LGPLv3。可以在 [Open Source Initiative](https://opensource.org/licenses/alphabetical) 中获取许可协议的相关信息。我们使用BSD协议，因为ROS核心组件的剩余部分已经使用了该协议。
   ``` 
 <license>BSD</license>
   ```
依赖项标签：
依赖项标签用来描述程序包的各种依赖项，包括 build_depend、buildtool_depend、run_depend、test_depend。
关于这些标签的更详细介绍请参考 [Catkin Dependencies](http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies) 相关的文档。在之前的操作中，因为我们将 std_msgs、 roscpp、 和 rospy 作为 catkin_create_pkg 命令的参数，所以生成的依赖项看起来如下:
   ```
 <!-- The *_depend tags are used to specify dependencies -->
 <!-- Dependencies can be catkin packages or system dependencies -->
 <!-- Examples: -->
 <!-- Use build_depend for packages you need at compile time: -->
 <!--   <build_depend>genmsg</build_depend> -->
 <!-- Use buildtool_depend for build tool packages: -->
 <!--   <buildtool_depend>catkin</buildtool_depend> -->
 <!-- Use run_depend for packages you need at runtime: -->
 <!--   <run_depend>python-yaml</run_depend> -->
 <!-- Use test_depend for packages you need only for testing: -->
 <!--   <test_depend>gtest</test_depend> -->
 <buildtool_depend>catkin</buildtool_depend>
 <build_depend>roscpp</build_depend>
 <build_depend>rospy</build_depend>
 <build_depend>std_msgs</build_depend>
   ```
除了 *catkin* 中默认提供的 *buildtool_depend*，所有我们列出的依赖包都已经被添加到 *build_depend* 标签中。因为在编译和运行时我们需要用到所有指定的依赖包，因此还需要将每一个依赖包分别添加到 *run_depend* 标签中:
   ```
 <buildtool_depend>catkin</buildtool_depend>
 
 <build_depend>roscpp</build_depend>
 <build_depend>rospy</build_depend>
 <build_depend>std_msgs</build_depend>
  
 <run_depend>roscpp</run_depend>
 <run_depend>rospy</run_depend>
 <run_depend>std_msgs</run_depend>
   ```
完整的 [package.xml](http://wiki.ros.org/catkin/package.xml) :
   ```
 <?xml version="1.0"?>
    <package>
      <name>beginner_tutorials</name>
      <version>0.1.0</version>
      <description>The beginner_tutorials package</description>
    
      <maintainer email="you@yourdomain.tld">Your Name</maintainer>
      <license>BSD</license>
      <url type="website">http://wiki.ros.org/beginner_tutorials</url>
     <author email="you@yourdomain.tld">Jane Doe</author>
   
     <buildtool_depend>catkin</buildtool_depend>
   
     <build_depend>roscpp</build_depend>
     <build_depend>rospy</build_depend>
     <build_depend>std_msgs</build_depend>
   
     <run_depend>roscpp</run_depend>
     <run_depend>rospy</run_depend>
     <run_depend>std_msgs</run_depend>
   
   </package>
   ```
