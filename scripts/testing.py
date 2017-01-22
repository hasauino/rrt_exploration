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



from time import time
from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import Nearest,Nearest2,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue
from assigner2_functions import robot,informationGain,discount,pathCost,unvalid
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift
import numpy as np
from numpy.linalg import norm


from assigner2_functions import robot

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
pt=[]
def callBack(data):
	global pt
	pt=[data.point.x,data.point.y]
	
    

def mapCallBack(data):
    global mapData
    mapData=data
    
# Node----------------------------------------------

def node():
	k=0
	global mapData,pt
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/robot_1/move_base_node/global_costmap/costmap')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	goals_topic= rospy.get_param('~goals_topic','/exploration_goals')	
	n_robots = rospy.get_param('~n_robots',3)
	global_frame=rospy.get_param('~global_frame','/robot_1/map')

	rate = rospy.Rate(100)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber("/clicked_point", PointStamped, callBack)

#---------------------------------------------------------------------------------------------------------------
	




	points=Marker()
	points_clust=Marker()
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
	points.header.frame_id= "/robot_1/map"
	points.header.stamp= rospy.Time.now()

	points.ns= "markers2"
	points.id = 0
	
	points.type = Marker.POINTS
	
#Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action = Marker.ADD;

	points.pose.orientation.w = 1.0

	points.scale.x=0.2
	points.scale.y=0.2 

	points.color.r = 255.0/255.0
	points.color.g = 255.0/255.0
	points.color.b = 0.0/255.0

	points.color.a=1;
	points.lifetime = rospy.Duration();

	p=Point()


	p.z = 0;

	pp=[]
	pl=[]
    	
    	
    	
    	
    	
	points_clust.header.frame_id= "/robot_1/map"
	points_clust.header.stamp= rospy.Time.now()

	points_clust.ns= "markers3"
	points_clust.id = 4

	points_clust.type = Marker.POINTS

#Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points_clust.action = Marker.ADD;

	points_clust.pose.orientation.w = 1.0;

	points_clust.scale.x=0.2;
	points_clust.scale.y=0.2; 

	points_clust.color.r = 0.0/255.0
	points_clust.color.g = 255.0/255.0
	points_clust.color.b = 0.0/255.0

	points_clust.color.a=1;
	points_clust.lifetime = rospy.Duration();

	robots=[]
	for i in range(0,n_robots):
		robots.append(robot('/robot_'+str(i+1)))
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------

	while not rospy.is_shutdown():
		print gridValue(mapData,pt)

		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
