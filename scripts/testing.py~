#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import actionlib_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
import actionlib
import tf


from numpy.linalg import norm
from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import Nearest,Nearest2,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue
from assigner2_functions import robot,informationGain,discount,pathCost
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift
import numpy as np



from assigner2_functions import robot

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data   
# Node----------------------------------------------

def node():

	global frontiers,mapData,centroids,gain_cent
	rospy.init_node('testing', anonymous=False)
	rate = rospy.Rate(100)
		# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map_merge/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	hysteresis_radius=rospy.get_param('~hysteresis_radius',5.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	goals_topic= rospy.get_param('~goals_topic','/exploration_goals')	
	n_robots = rospy.get_param('~n_robots',3)
	global_frame=rospy.get_param('~global_frame','/robot_1/map')

#-------------------------------------------

	myrobot=robot('/robot_3')
	print pathCost(myrobot.makePlan(	myrobot.getPosition(),	(myrobot.getPosition()+array([1.,1.0]))	))



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
