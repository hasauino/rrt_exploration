# rrt_exploration
It is a ROS package that implements a multi-robot exploration algorithm. It is based on the Rapidaly-Exploring Random Tree (RRT) algorithm. The packgae has 5 different ROS nodes:

  - Global RRT frontier point detector node.
  
  - Local RRT frontier point detector node.
  
  - Filter node.
  
  - Assigner node.
  
  - opencv-based frontier detector node.

## 1-Requirements
The package has been tested on both ROS Kinetic and ROS Indigo, it should work on other distributions like Jade. The following requirements are needed before installing the package:

1- You should have installed a ROS distribution (indigo or later. Recommended is either indigo or kinetic).

2- Created a workspace.

3- Installed the "gmapping" ROS package: on Ubuntu, and if you are running ROS Kinectic, you can do that by typing the following command in the terminal:

```sh
$ sudo apt-get install ros-kinetic-gmapping
```
4- Install ROS navigation stack. You can do that with the following command (assuming Ubuntu, ROS Kinetic):
```sh
$ sudo apt-get install ros-kinetic-navigation
```
5- You should have Python 2.7. (it was not tested on Python 3).

6-You should have/install the following python modules:

-OpenCV (cv2)
```sh
$ sudo apt-get install python-opencv
```
-Numpy
```sh
$ sudo apt-get install python-numpy
```
-Sklearn
```sh
$ sudo apt-get install python-scikits-learn
```
## 2-Installation
Download the package and place it inside the ```/src``` folder in your workspace. And then compile using ```catkin_make```.

## 3-Nodes
There are 3 types of nodes; nodes for detecting frontier points in an occupancy grid map, a node for filtering the detected points, and a node for assigning the points to the robots. The following figure shows the structure:
![alt text](https://github.com/hasauino/storage/blob/master/pictures/fullSchematic.png "overview of the exploration strategy")

### 3.1. global_rrt_frontier_detector
The ```global_rrt_frontier_detector``` node takes an occupancy grid and finds frontier points (which are exploration targets) in it. It publishes the detected points so the filter node can process. In multi-robot configuration, it is intended to have only a single instance of this node running. 

#### 3.1.1. Parameters
 - ```~map_topic``` (string, default: "/robot_1/map"): This parameter defines the topic name on which the node will recieve the map.
  - ```~eta``` (float, default: 0.5): This parameter controls the growth rate of the RRT that is used in the detection of frontier points, the unit is in meters. This parameter should be set according to the map size, a very large value will cause the tree to grow faster and  hence detect frontier points faster, but a large growth rate also implies that the tree will be missing small corners in the map.

#### 3.1.2. Subscribed Topics
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

- ```clicked_point``` ([geometry_msgs/PointStamped Message](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html)): The ```global_rrt_frontier_detector``` node requires that the region to be explored is defined. This topic is where the node recieves five points that define the region. The first four points are four defining a square region to be explored, and the last point is the tree starting point. After publishing those five points on this topic, the RRT will start detecting frontier points. The five points are intended to be published from Rviz using ![alt text](https://github.com/hasauino/storage/blob/master/pictures/publishPointRviz_button.png "Rviz publish point button") button.

#### 3.1.3. Published Topics





