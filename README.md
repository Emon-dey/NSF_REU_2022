# NSF_REU_2022
This repository will include the necessary resources for RobSenCom project. You can create issue on any of your questions regarding the materials, installation problems.

# Package for Testing Performance of Ros2 system
https://github.com/irobot-ros/ros2-performance/tree/foxy

Using the foxy branch, performances/performance_test_factory/examples then the subscriber_nodes_main and publisher_nodes_main code on two separate turtlebots

## Installing GUI for the turtlebot3

After running the following commands the turtlebot3 should now be equipped with a GUI that makes ease of use much easier
'''bash
sudo apt-get install ubuntu-desktop-minimal
sudo reboot
'''
Since the network manager before was netplan, in order to use NetworkManager the netplan config file found in */etc/netplan/50-cloud-init.yaml* has to be modified using the following steps in terminal:
'''bash
sudo nano /etc/netplan/50-cloud-init.yaml
# Convert the file to look like this
'''
network:
            version: 2
            renderer: NetworkManager
'''
sudo netplan apply
sudo reboot
'''
Now the network can be managed using the drop down menu at the top right of the screen

## Installing and running performance package on two Turtlebot3 Burger robots
When running a system using two separate turtlebots in a ROS2 foxy system there are two changes that must be added before the performance package can be successfully installed. 

### Adding a Namespace to the turtlebots
The easiest way to separate the nodes and topics of two turtlebots is the use of namespaces. This is done by changing a total of 3 files in the turtlebot3 and LDS packages provided by robotis, starting with the node declaration found in *turtlebot3/turtlebot3_bringup/launch/robot.launch.py* (*tb3_XXX* represents any namespace that should be added to the node and topics associated with it).
```python
Node(
            package='turtlebot3_node',
            namespace='tb3_XXX',          // add this line
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
```
To complete of adding the namespace the param file must be changed in *turtlebot3/turtlebot3_bringup/param/burger.yaml'
```python
tb3_XXX:                              // add this line at top of file, properly indent following lines
            turtlebot3_node:
                        ...
            
```
Similarly a namespace must be added to the provided lidar package to separate the scan topics completed by changing the node declaration in launch file *ld08_driver/launch/ld08.launch.py*
```python
Node(
            package='ld08_driver',
            namespace='tb3_XXX',    //add this line
            executable='ld08_driver',
            name='ld08_driver',
            output='screen'),
```
After building the package and running the bringup, nodes will now be separated by a namespace ex: *tb3_XXX/scan, tb3_XXX/cmd_vel, tb3_XXX/...*

### Adding virtual memory before running colcon build
Since the raspberry pi's found on the turtlebots don't have a high amount of RAM it's important to add virtual memory before building the performance package or the build process will freeze. This can be done by installing the dphys-swapfile package using the terminal.
```
sudo apt install dphys-swapfile
sudo dphys-swapfile swapoff
sudo dphys-swafile swapon
```
Turning the package off and on usually will suffice however it may require a system reboot before going into affect, the virtual memory can be viewed by running *free -m* which will now display swap memory of around 2.4 GB. To increase or reduce the memory the following steps should be taken:
````
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```
Change the CONF_SWAPSIZE value
CTRL + S, CTRL + X to save and exit
```
sudo dpys-swapfile swapon
````
### Installing the Performance Package
After successfully completing the steps to setup the two turtlebots, the performance package can be successfully built and installed. This may take several minutes to complete however adding the virtual memory will prevent any freezing.

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
