# How To Setup a Turtlebot From Scratch


* make sure ubuntu 16.04.6 is installed
* install ros kinetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
```

note: if apt-get gives ‘Could not get lock /var/lib/dpkg/lock’ error, do the following:

```bash
        $ sudo rm /var/lib/dpkg/lock
        $ sudo dpkg --configure -a
```

```bash
$ sudo apt-get install ros-kinetic-desktop-full
```

```bash
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/kinetic/setup.bash (put in .bashrc)
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## setup workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash (put in .bashrc)
$ echo $ROS_PACKAGE_PATH (should be /home/youruser/catkin_ws/src:/opt/ros/kinetic/share)
```

## install turtlebot software

```bash
$ sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
```

## ensure network is configured

```bash
#export ROS_MASTER_URI=http://192.168.1.125:11311    #trail
#export ROS_HOSTNAME=192.168.1.125
#export ROS_IP=192.168.1.125
export ROS_MASTER_URI=http://10.108.19.59:11311     #tusecure
export ROS_HOSTNAME=10.108.19.59
export ROS_IP=10.108.19.59
```

## bringup turtlebot software

```bash
$ roslaunch turtlebot_bringup minimal.launch
```

note: if robot_state_publisher keeps failing to initialize, update the whole distro using:

```bash
$ sudo apt-get dist-upgrade
```

## install the ZED software

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/stereolabs/zed-ros-wrapper.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```

note: you will most likely get an error saying that you need ZED SDK, install at:
        https://www.stereolabs.com/developers/release/#sdkdownloads_anchor
        if you have CUDA 9.0 (check with $ nvcc --version), install for Jetpack 3.3
once downloaded, install with the following:
```bash
$ cd ~/Downloads
$ chmod +x ZED_SDK_JP3.3_v2.8.2.run
$ ./ZED_SDK_JP3.3_v2.8.2.run
```

to test that it was installed correctly, run the node:

```bash
$ roslaunch zed_display_rviz display_zed.launch
```

## install the YOLO software

```bash
$ cd ~/catkin_ws/src
$ git clone --recursive http://github.com/leggedrobotics/darknet_ros.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```

in order to get YOLO to receive the images from ZED, go into ~/catkin_ws/src/darknet_ros/darknet_ros/config/ros.yaml and change camera_reading to
/zed_node/rgb_raw/image_raw_color


once ZED is running, ensure YOLO is working by running
        roslaunch darknet_ros darknet_ros.launch


## install the hokuyo software

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/urg_node.git
$ git clone https://github.com/ros-perception/laser_proc.git
$ git clone https://github.com/ros-drivers/urg_c.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```

note: if you get the error ‘could not open ethernet port’ you must configure the hokuyo on the network:
1. Open up the Network Manager
2. Edit Connections
3. Find ethernet connection
4. Go to IPv4 Settings
5. Set Method to Manual
6. Click Add button the right
7. Set Address to 192.168.0.15 (IP address of the Hokuyo is 192.168.0.10 so this needs to be something else on the same subnet)
8. Set Netmask to 255.255.255.0
9. Leave Gateway empty
10. Click the routes button in the lower right
11. Check the bottom box for “Use this connection only for resources on its network”
When calling the node (with roscore running): rosrun urg_node urg_node _ip_address:="192.168.0.10" 


note: to fix static robot model, in turtlebot_bringup/launch/includes/robot.launch.xml, add the following:
<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="5.0" />
    <param name="use_tf_static"     type="bool"   value="false" />
</node>


note: if the map is ‘flickering’ it means there are two publishers between odom and map:
```bash
$ roscd zed_wrapper
$ cd params
$ sudo vim common.yaml
```
        within this file, under tracking, change publish_tf and publish_map_tf to false


## setup the costmap parameters so that the algorithms use all available sensors

```bash
$ roscd turtlebot_navigation
$ cd param
$ sudo vim costmap_common_params.yaml
```
look for the section called observation sources and modify to the following (delete obstacle_range and raytrace_range above this line):
          observation_sources:  scan camera bump
  scan:
        data_type: LaserScan
        topic: base_scan
        marking: true
        clearing: true
        min_obstacle_height: 0.0
        max_obstacle_height: 0.35
        obstacle_range: 9.0
        raytrace_range: 10.0
  camera:
        datadata_type: LaserScan
        topic: camera_scan
        marking: true
        clearing: true
        min_obstacle_height: 0.0
        max_obstacle_height: 0.35
        obstacle_range: 1.0
        raytrace_range: 1.5


this should fix the costmaps within rviz.


## setup an ssh key on the new turtlebot so that you don’t need to log in every time
        
make sure that a key was generated on the desktop

```bash
$ ssh-keygen -t rsa
```
copy the key into the machine you want to login to

```bash
$ ssh-copy-id nvidia@192.168.1.125
```


## edit amcl params

```bash
$ roscd amcl
$ cd examples
$ gedit amcl_omni.launch
```

add the following two lines to this code:
<remap from="scan" to="base_scan"/>
<param name="use_map_topic" value="true"/>


## if amcl is not publishing the transform from base -> odom, it is most likely because the ZED is also trying to publish that transform. Edit:

```bash
$ roscd zed_wrapper/params
$ gedit common.yaml
```