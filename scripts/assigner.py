#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot, informationGain, discount, gridValue
from numpy.linalg import norm
from math import atan2
import time

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
global1 = OccupancyGrid()
global2 = OccupancyGrid()
global3 = OccupancyGrid()
globalmaps = []


def callBack(data):
    global frontiers
    frontiers = []
    for point in data.points:
        frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
    global mapData
    mapData = data
# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps
    rospy.init_node('assigner', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 1.0)
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    # at least as much as the laser scanner range
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)
    # bigger than 1 (biase robot to continue exploring current region
    hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
# ---------------------------------------------------------------------------------------------------------------

# wait if no frontier is received yet
    while len(frontiers) < 1:
        time.sleep(0.1)
        pass
    centroids = copy(frontiers)

# wait if map is not received yet
    while (len(mapData.data) < 1):
        time.sleep(0.1)
        pass

    robots = []
    if len(namespace) > 0:
        for i in range(0, n_robots):
            robots.append(robot(namespace+str(i+namespace_init_count)))
    elif len(namespace) == 0:
        robots.append(robot(namespace))
    for i in range(0, n_robots):
        robots[i].sendGoal(robots[i].getPosition()[0:2])

# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
# -------------------------------------------------------------------------
# Get information gain for each frontier point
        infoGain = []
        for fnt in range(0, len(centroids)):
            infoGain.append(informationGain(
                mapData, [centroids[fnt][0], centroids[fnt][1]], info_radius))
# -------------------------------------------------------------------------
# get number of available/busy robots
        availableRobots = []  # available robots
        busyRobots = []  # busy robots
        for i in range(0, n_robots):
            # rospy.loginfo('Robot State: ' + str(robots[i].getState()))
            if (robots[i].getState() == 1):
                busyRobots.append(i)
                # Check for the current goal grid cell
                x = array([robots[i].goal.target_pose.pose.position.x,
                           robots[i].goal.target_pose.pose.position.y])
                GoalGridValue = gridValue(mapData, x)
                if(GoalGridValue > threshold):
                    robots[i].cancelGoal()
                    rospy.loginfo(str.format(
                        'Canceling Goal for Robot-{0:d} due to threshold: {1:d}', i, GoalGridValue))
            else:
                availableRobots.append(i)
# -------------------------------------------------------------------------
# get dicount and update informationGain
        for i in busyRobots+availableRobots:
            infoGain = discount(
                mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
# -------------------------------------------------------------------------
        reveneus = []
        centroidRecord = []
        idRecord = []
        costN = []
        costI = []
        costO = []
        for rob in availableRobots:
            for fnt in range(0, len(centroids)):
                pose = robots[rob].getPosition()
                costN.append(norm(pose[0:2] - centroids[fnt]))
                information_gain = infoGain[fnt]
                # if a frontier is within the vacinity of a certain radius, give it imporatance
                if (norm(pose[0:2]-centroids[fnt]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                costI.append(information_gain*info_multiplier)
                costO.append(
                    50*(atan2(pose[1]-centroids[fnt][1], pose[0]-centroids[fnt][0])-pose[2]))
                revenue = costI[-1] - costN[-1] - costO[-1]
                reveneus.append(revenue)
                centroidRecord.append(centroids[fnt])
                idRecord.append(rob)

# -------------------------------------------------------------------------
        if (len(idRecord) > 0):
            idx = reveneus.index(max(reveneus))
            robots[idRecord[idx]].sendGoal(centroidRecord[idx])
            rospy.loginfo("Robots:" + str(len(availableRobots)) +
                          ", Frontiers:" + str(len(centroids)))
            rospy.loginfo(str.format("[{:0.2f},{:0.2f}]", *centroidRecord[idx]) +
                          str.format(" assigned to Robot {0:d} -> R:{1:0.2f} I:{2:0.2f} N:{3:0.2f} O:{4:0.2f}",
                                     namespace_init_count + idRecord[idx],
                                     reveneus[idx], costI[idx], costN[idx], costO[idx]))
            rospy.sleep(delay_after_assignement)
# -------------------------------------------------------------------------
        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
