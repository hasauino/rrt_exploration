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
#-----------------------------------------------------
#functions

def Nearest_centroids(robot_data):
 n=inf
 result=-1
 x=robot_data['position']
 V=robot_data['valid_centroids']
 
 
 
 for i in range(0,len(V)):
    n1=LA.norm(V[i]-x)	
    
    if (n1<n):
	n=n1
        result=i    
 return result
 
 
 


def assign(goal,client,robot_data):
	
	clientstate=client.get_state()
	
	if clientstate==2 or clientstate==3 or clientstate==4 or clientstate==5 or clientstate==9:
		
		if len(robot_data['valid_centroids'])>0:
			
    			row=Nearest_centroids(robot_data)
    			
    			if row>=0:
    				
	    			commanded=robot_data['valid_centroids'][row]
	    			goal.target_pose.pose.position.x=commanded[0]
				goal.target_pose.pose.position.y=commanded[1]
    				goal.target_pose.pose.orientation.w = 1.0
    				print "exploration goal sent"
				client.send_goal(goal)
				robot_data['commanded']=commanded
	
	
	return robot_data


# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]

def callBack(Data):
    data=Data.point
    global frontiers,min_distance
    x=[array([data.x,data.y])]
    
    if len(frontiers)>0:
    	if LA.norm(frontiers[  Nearest2(frontiers,x)  ]-x)>min_distance:
       		frontiers=vstack((frontiers,x))
    else:
       	frontiers=x
	

    
def mapCallBack(data):
    global mapData
    mapData=data
    

# Node----------------------------------------------
def node():

	global frontiers,mapData,min_distance
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	n_robots = rospy.get_param('~n_robots',3)
	min_distance= rospy.get_param('~min_distance',0.10)	
#-------------------------------------------
    	rospy.Subscriber("/robot_1/map", OccupancyGrid, mapCallBack)
    	rospy.Subscriber("/exploration_goals", PointStamped, callBack)
    	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    	pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    	
	while mapData.header.seq<1 or len(mapData.data)<1:
		pass	
	
	
	clients=list()
    	#Actionlib client
    	for i in range(0,n_robots):
    		clients.append( actionlib.SimpleActionClient('/robot_'+str(i+1)+'/move_base', MoveBaseAction) )
    		clients[i].wait_for_server()
    	
    	

    	goal = MoveBaseGoal()
    	goal.target_pose.header.stamp=rospy.Time.now()
    	goal.target_pose.header.frame_id="/robot_1/map"


    	   	
    	rate = rospy.Rate(100)	

	listener = tf.TransformListener()
	listener.waitForTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0),rospy.Duration(10.0))
	#listener.waitForTransform('/robot_1/map', '/robot_2/base_link', rospy.Time(0),rospy.Duration(10.0))
	


    	goal.target_pose.pose.position.z=0
    	goal.target_pose.pose.orientation.w = 1.0
    	


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
        for j in range(0,n_robots):
	     cond=0;	
	     while cond==0:	
	  	try:
           		cond=1
           		(trans,rot) = listener.lookupTransform(mapData.header.frame_id, '/robot_'+str(j+1)+'/base_link', rospy.Time(0))
           		
           		
        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             		cond==0
	  	
	     x_pos=[array(  [    trans[0],trans[1]   ]   )]
	     
	     robot_data=robot_data+[{'ID':j , 'position':x_pos,   'commanded':x_pos, 'valid_centroids':[]  }] 			#initially commnded position = current position

    	
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------

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
          	ms = MeanShift(bandwidth=2.0)   
		ms.fit(frontiers)
		centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster


#-------------------------------------------------------------------------
#Get robot positions
	   	
          

          for j in range(0,n_robots):
	     cond=0;	
	     while cond==0:	
	  	try:
           		cond=1
           		(trans,rot) = listener.lookupTransform(mapData.header.frame_id, '/robot_'+str(j+1)+'/base_link', rospy.Time(0))
           		
           		
        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             		cond==0
	  	
	     x_pos=[array(  [    trans[0],trans[1]   ]   )]
	  	
	     robot_data[j]['position']=x_pos
	
	  	
#-------------------------------------------------------------------------
#Update vaild frontier centroids for each robot
	  centroids_radius=2.0
          print centroids,"\n------------------\n"
          if len(centroids)>0:
          	for j in range(0,n_robots):
          		robot_data[j]['valid_centroids']=[]
          	
          	
          		
          		for k in range(0,len(centroids)):
          		#robot j    centroid k
          			temp=True
          			# check for centroid k, if it is not close to any commanded position of each robot except the current robot j, if so, then this point is valid and can be considered as a candidtae point to be assigned to robot j
          			for i in range (0,n_robots):
          				if LA.norm(centroids[k]-robot_data[i]['commanded'])<centroids_radius and (i !=j):
          					temp=False
          			
          			if temp:
          				
          				robot_data[j]['valid_centroids']=robot_data[j]['valid_centroids']+[	centroids[k]]
          				robot_data[j]=assign(goal,clients[j],robot_data[j])
          		print "r",robot_data[j]['ID'],"   ",robot_data[j]['valid_centroids'],"\n------------------\n"


	  

#-------------------------------------------------------------------------
#Assignment
          #for j in range(0,n_robots):
		#robot_data[j]=assign(goal,clients[j],robot_data[j])

          
          
          
          
          
#-------------------------------------------------------------------------        
#Plotting
	  pp=[]	
	  for q in range(0,len(frontiers)):
	  	
	  	p.x=frontiers[q][0]
          	p.y=frontiers[q][1]
          	pp.append(copy(p))
          	
          points.points=pp
          pub.publish(points) 
          
          pp=[]	
	  for q in range(0,len(centroids)):
	  	
	  	p.x=centroids[q][0]
          	p.y=centroids[q][1]
          	pp.append(copy(p))
	
          

	  	
  	  points_clust.points=pp
          pub2.publish(points_clust) 
		
	  
	  rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
