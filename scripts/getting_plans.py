#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
import actionlib_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped 
PointStamped
import actionlib
import tf



from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import Nearest,Nearest2,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift
import numpy as np


start = PoseStamped()
start.header.frame_id = "/robot_1/map"
start.pose.position.x = 0.0

goal = PoseStamped()
goal.header.frame_id = "/robot_1/map"
goal.pose.position.x = 20.0
goal.pose.position.y = 2.0

tolerance=0.0

mapData=OccupancyGrid()

def mapCallBack(data):
    global mapData
    mapData=data






def callBack(Data):
	global goal
	print "ok"
	goal.pose.position.x=Data.point.x
	goal.pose.position.y=Data.point.y
	






def path_length(plan):
	print len(plan.plan.poses)

			



# Node--------------------------------------------------------------------
def node():

	global start,goal,tolerance,mapData
	rospy.init_node('planning', anonymous=False)
	rate = rospy.Rate(100)
	
	rospy.Subscriber("/robot_1/map", OccupancyGrid, mapCallBack)
	while mapData.header.seq<1 or len(mapData.data)<1:
		pass
		
	rospy.Subscriber("/clicked_point", PointStamped, callBack)
	
        rospy.wait_for_service('/robot_1/move_base_node/NavfnROS/make_plan')
        #Get a proxy to execute the service
        make_plan = rospy.ServiceProxy('/robot_1/move_base_node/NavfnROS/make_plan', GetPlan)
	


	while not rospy.is_shutdown():

		plan = make_plan(start = start, goal = goal, tolerance = tolerance)
		path_length(plan)


		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
