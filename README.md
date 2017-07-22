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
There are 3 types of nodes; nodes for detecting frontier points in an occupancy grid map, a node for filtering the detected points, and a node for assigning the points to the robots. 
Inline-style: 
![alt text](https://github.com/hasauino/storage/blob/master/pictures/fullSchematic.png "overview of the exploration strategy")

### 3.1. global_rrt_frontier_detector








