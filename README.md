# Quadrupedal Foothold Planner
Auto planning footholds of quadrupedal robots with user specified gait and gait cycles

## Overview

Author: Lu Chen<br>
Affiliation: AIRS(CUHK-SZ)<br>
Maintainer: chenlucqu@gmail.com<br>


## Mapping pipleline
![image](https://github.com/lukechencqu/quadrupedal_foothold_planner/blob/master/materials/mapping.jpg)


## Foothold planning pipleline
Support planning the footholds of many kinds of gaits, such as walk and trot.

![](https://github.com/lukechencqu/quadrupedal_foothold_planner/blob/master/materials/global_foothold_planning.gif)



## Installation
### Dependencies

This software is built on the Robotic Operating System (ROS), which needs to be installed first. Additionally, the Quadrupedal Foothold Planner depends on following software:

* [Grid Map (grid map library for mobile robots)](https://github.com/anybotics/grid_map)<br>
* [Elevation_Mapping (Robot-Centric Elevation Mapping),](https://github.com/ANYbotics/elevation_mapping#citing)<br>
* [Traversability Estimation (Traversability mapping for mobile rough terrain navigation.)](https://github.com/leggedrobotics/traversability_estimation)<br>
* [kindr (kinematics and dynamics library for robotics),](http://github.com/anybotics/kindr)<br>
* [kindr_ros (ROS wrapper for kindr),](https://github.com/anybotics/kindr_ros)<br>
* [Point Cloud Library (PCL) (point cloud processing),](http://pointclouds.org/)<br>
* [Eigen (linear algebra library).](http://eigen.tuxfamily.org/)<br>

### Building

In order to install the package, clone the latest version from this repository into your catkin workspace and compile the package using ROS.
```
cd catkin_workspace/src
git clone https://github.com/lukechencqu/quadrupedal_foothold_planner.git
cd ../
catkin_make
```
## Nodes
### Node: foothold_planner_node
#### Subscribed Topics
* /traversability_estimation/traversability_map ([grid_map_msgs/GridMap](https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg))<br>
#### Published Topics
* global_footholds (foothold_planner_msgs/GlobalFootholds)<br>
The planned global footholds.

* global_footholds_marker ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))<br>
The global footholds markers for RVIZ visualization.

* RF_search_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html))<br>
The right front foothold search region.

* RH_search_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html))<br>
The right hind foothold search region.

* LH_search_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html))<br>
The left hind foothold search region.

* LF_search_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html))<br>
The left front foothold search region.

#### Services
* plan_global_footholds (foothold_planner::GlobalFootholdPlan::Request)
Call planning global footholds service. User can specify how many gait cycles of footholds to plan.<br>

Request:
```
rosservice call /foothold_planner/plan_global_footholds "gait_cycles: 8"
```
Response:
```
footholds: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: "odom"
  success: True
  gait_cycles: 8
  gait_cycles_succeed: 8
  footholds: 
    - 
      point: 
        x: 0.371730949127
        y: -0.124499998987
        z: 0.0
      foot_id: 0
      gait_cycle_id: 0
    - 
      point: 
        x: 0.133030961596
        y: -0.124499998987
        z: 0.0
      foot_id: 1
      gait_cycle_id: 0
      
    ...
      
    - 
      point: 
        x: 1.84522620246
        y: 0.124499998987
        z: 0.0
      foot_id: 3
      gait_cycle_id: 7
```
### Node: map_tf

## Parameters
