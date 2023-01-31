# NSF_REU_2022
This repository will include the necessary resources for RobSenCom project. You can create issue on any of your questions regarding the materials, installation problems.

# Package for Testing Performance of Ros2 system
https://github.com/irobot-ros/ros2-performance/tree/foxy

Using the foxy branch, performances/performance_test_factory/examples then the subscriber_nodes_main and publisher_nodes_main code on two separate turtlebots

## Installing and running performance package on two Turtlebot3 Burger robots
When running a system using two separate turtlebots in a ROS2 foxy system, the easiest way to separate the nodes and topics is the use of namespaces. This is done by changing a total of 4 files, starting with the turtlebot3_bringup/launch/robot.launch.py
```python
Node(
            package='turtlebot3_node',
            namespace='tb3_0',          // add this line
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
```

# Getting started with ROS
- Start from the link given below to get an idea on ROS basics. This is a great compilation on all the necessary rersources on this field including papers, software, hardware, etc.
http://wiki.ros.org/
- Try installing ROS in your Ubuntu machine as soon as you are confident on the ROS basics.

- You can check the papers from the recent ICRA and IROS conferences, two pioneering venues on robotics research, to get an idea on the current research trend. Use the following link:
https://www.ieee-ras.org/conferences-workshops

- A Google drive is also shared with you containing relevant papers of the project you will be working on. You can also find the course materials of a ROS workshop arraged by UMD in that drive.

# Software
- Physics Simulator (Gazebo)
https://classic.gazebosim.org/tutorials
- Network Simulator (NS-3)
https://www.nsnam.org/docs/tutorial/html/

# Hardware
- Duckiedrone
https://docs.duckietown.org/daffy/opmanual_sky/out/index.html

# Synchronizing Middleware 
-Sample Github repos
1. https://github.com/srikrishna3118/FlyNetSim
2. https://github.com/srikrishna3118/CORNET
3. https://github.com/alelab-upenn/ros-net-sim
