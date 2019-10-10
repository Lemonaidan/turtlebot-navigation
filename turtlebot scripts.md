# Turtlebot Scripts

## basic commands

* ensure both robot and computer are connected to TRAIL network

to ssh into the turtlebot from the workstation:

```bash
$ ssh nvidia@192.168.1.122
```

open 3 ssh tabs on computer and run the following:

```bash
$ roslaunch turtlebot_bringup minimal.launch
$ roslaunch turtlebot_teleop keyboard_teleop.launch
$ roslaunch zed_wrapper zed.launch
```

then, in a 4th local terminal, run:

```bash
$ roslaunch darknet_ros darknet_ros.launch
```

in order to record footage from YOLO, run:

```bash
$ rosrun image_view video_recorder image:=/darknet_ros/detection_image
```

to use rviz locally, run:

```bash
$ rosrun rviz rviz
```

to use the controller plugged into the desktop, run locally:

```bash
$ rosrun joy joy_node
$ rostopic echo joy (to see if working)
$ roslaunch turtlebot teleop xbox360_teleop.launch
```

## ZED 3D mapping:
```bash
$ export ROS_NAMESPACE=camera (usually in bashrc but check)
$ roslaunch zed_wrapper zed_camera.launch publish_tf:=false
$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/depth/depth_registered frame_id:=zed_camera_center approx_sync:=false rviz:=false rtabmapviz:=false
(on local terminal) $ rosrun rviz rviz
```

## hokuyo 2D mapping:
```bash
(cd ~/Desktop/launch) $ roslaunch hokuyo_gmapping.launch
(locally) $ rosrun rviz rviz -d home/aidan/Desktop/rviz/robot_navigation.rviz
```

## depth image to laser scan gmapping (zed 2D mapping):
```bash
(cd ~/Desktop/launch) $ roslaunch zed_gmapping.launch
(locally) $ rosrun rviz rviz -d home/aidan/Desktop/rviz/robot_navigation.rviz
```

save map:
```bash
$ rosrun map_server map_saver -f ~/maps/my_map
```
load map:
```bash
$ rosrun map_server map_server ~/maps/my_map.yaml
```
amcl:
```bash
$ roslaunch amcl amcl_omni.launch
```

## AUTONOMOUS MOVEMENT!
```bash
(cd ~/Desktop/launch) $ roslaunch amcl_nav.launch
(locally) $ rosrun rviz rviz -d home/aidan/Desktop/rviz/robot_navigation.rviz
```

## frontier exploration:
```bash
(cd ~/Desktop/launch) $ roslaunch frontier_exploration.launch
(locally) $ rosrun rviz rviz -d home/aidan/Desktop/rviz/frontier_exploration.rviz
```

## run patrol:
```bash
(cd ~/Desktop/launch) $ python patrol.py
```

## in order to get the turtlebot to continue exploring even when it gets out of reach of the router:

* first, ssh into two terminals and execute a screen command on each
* ensure that your patrol route starts and ends within reach of the router
* on the two terminals, run your navigation
```bash
(cd ~/Desktop/launch) $ roslaunch amcl_nav.launch
```
OR
```bash
(cd ~/Desktop/launch) $ roslaunch frontier_exploration.launch
```	
* and the patrol software
```bash
(cd ~/Desktop/launch) python patrol.py
```
* then, locally, run rviz
```bash
(locally) rosrun rviz rviz -d ~/Desktop/rviz/frontier_exploration.rviz
```
* if you were gmapping, when the robot finishes its patrol route, ssh again and run
```bash
rosrun map_server map_saver -f ~/maps/my_map
```

## use rosbag to record data during a run:
```bash
(cd ~/bagfiles) $ rosbag record -a -x "(.*)compressed(.*)|(.*)theora(.*)|(.*)/stereo(.*)|(.*)_rect_(.*)|(.*)/right(.*)|(.*)/rgb(.*)" --lz4
```

## MULTI-ROBOT CONTROL:
```bash	
roslaunch tf_throttle tb_master.launch
rosrun rviz rviz -d home/aidan/Desktop/rviz/frontier_exploration.rviz
```

## Multi Object Tracking Lidar:
```bash
$ roslaunch hokuyo_gmapping.launch viz:=false
(cd ~/Desktop/launch) $ python project_laser.py
$ rosrun multi_object_tracking_lidar kf_tracker
$ rosrun rviz rviz -d home/aidan/Desktop/rviz/robot_navigation.rviz
```

## TO SEND BACK TO START (WEST IS UP):
```bash
$ rostopic pub /turtle22/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
$ rostopic pub /turtle25/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.0, y: 0.4, z: 0.0}, orientation: {w: 1.0}}}'
```

## UP NEXT ON THE TO-DO LIST:

SLAM stuff:
http://wiki.ros.org/rtabmap_ros/Tutorials/StereoHandHeldMapping

built-in turtlebot stuff:
http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM
http://wiki.ros.org/turtlebot_follower/Tutorials/Demo
http://wiki.ros.org/turtlebot_panorama/Tutorials/Demo

filtering stuff:
https://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/
	https://github.com/ros-gbp/bfl-release
http://wiki.ros.org/bfl/Tutorials/Example%20of%20using%20a%20particle%20filter%20for%20localization%20by%20bfl%20library

take a look at this:
http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
http://wiki.ros.org/amcl

autonomous exploration:
http://wiki.ros.org/nav2d
http://wiki.ros.org/frontier_exploration
https://github.com/KumarRobotics/scarab

path optimization & people avoidance:
http://wiki.ros.org/eband_local_planner
http://wiki.ros.org/lidar_tracking

multi-robot navigation:
http://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/

AMCL ISSUE
http://answers.ros.org/question/193435/amcl-not-publishing-map-odom/
https://github.com/gergia/multiple_turtlebots_real_world

voice operation:
https://edu.gaitech.hk/turtlebot/speech-doc.html

people tracking:
https://github.com/praveen-palanisamy/multiple-object-tracking-lidar

