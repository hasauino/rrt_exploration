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



from assigner2_functions import robot

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]

def callBack(data):
	global frontiers,min_distance
	x=[array([data.x,data.y])]
	if len(frontiers)>0:
		frontiers=vstack((frontiers,x))
	else:
		frontiers=x
    

def mapCallBack(data):
    global mapData
    mapData=data
    
# Node----------------------------------------------

def node():

	global frontiers,mapData
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map_merge/map')
	goals_topic= rospy.get_param('~goals_topic','/exploration_goals')
	n_robots = rospy.get_param('~n_robots',1)
	global_frame=rospy.get_param('~global_frame','/robot_1/map')
	
	rate = rospy.Rate(100)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(goals_topic, Point, callBack)
	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
	pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
#---------------------------------------------------------------------------------------------------------------

    	
	while len(frontiers)<1:
		pass	




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

	points.pose.orientation.w = 1.0;

	points.scale.x=0.2;
	points.scale.y=0.2; 

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
	
#Initalize robot data dictionary list
	robot_data=[]
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------

	robot1=robot('/robot_1')
	while not rospy.is_shutdown():
	
		
#-------------------------------------------------------------------------	
#clearing old frontiers         
		z=0
		while z<len(frontiers):
			if gridValue(mapData,frontiers[z])!=-1:
				frontiers=delete(frontiers, (z), axis=0)
				z=z-1
			z+=1
			
#-------------------------------------------------------------------------
#Clustering frontier points
		centroids=[]
		if len(frontiers)>1:
			ms = MeanShift(bandwidth=0.5)   
			ms.fit(frontiers)
			centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster

		#if there is only one frontier no need for clustering, i.e. centroids=frontiers
		if len(frontiers)==1:
			centroids=frontiers			

			
			
#-------------------------------------------------------------------------        
		#Plotting
		pp=[]	
		for q in range(0,len(frontiers)):
			p.x=frontiers[q][0]
			p.y=frontiers[q][1]
			pp.append(copy(p))
		points.points=pp
	
	
		pp=[]	
		for q in range(0,len(centroids)):
			p.x=centroids[q][0]
			p.y=centroids[q][1]
			pp.append(copy(p))
		points_clust.points=pp
			
		pub.publish(points)
		pub2.publish(points_clust) 
		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
