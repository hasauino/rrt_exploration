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
import actionlib
import tf



from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from functions import Nearest,Nearest2,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue,assigner1rrtfront

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]

def callBack(data):
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
	n_robots = rospy.get_param('~n_robots',1)
	min_distance= rospy.get_param('~min_distance',1.0)	
#-------------------------------------------
    	rospy.Subscriber("/robot_1/map", OccupancyGrid, mapCallBack)
    	rospy.Subscriber("/exploration_goals", Point, callBack)
    	pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    	
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
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
    	points.header.frame_id= "/robot_1/map"
    	points.header.stamp= rospy.Time.now()
	
    	points.ns= "markers2"
    	points.id = 0
    	
    	points.type = Marker.POINTS
    	
#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
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
    	
    
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
#clearing old frontiers         
	  z=0
          while z<len(frontiers):
	  	if gridValue(mapData,frontiers[z])!=-1:
	  		frontiers=delete(frontiers, (z), axis=0)
	  		z=z-1
		z+=1
		
	
#Assignment-----------------------------------------	
	   	
          
          robots_positions=[]
          for j in range(0,n_robots):
	  	(trans,rot) = listener.lookupTransform(mapData.header.frame_id, '/robot_'+str(j+1)+'/base_link', rospy.Time(0))
	  	x_pos=[array(  [    trans[0],trans[1]   ]   )]
	  	robots_positions.append(x_pos)
	  print robots_positions,"\n ------------------------"
	  for j in range(0,n_robots):	
          	frontiers=assigner1rrtfront(goal,frontiers,clients[j],robots_positions[j]) 
          
          
          
          
          
          
          
#-------------------------------------------------------------------------        
          #print rospy.Time.now(),"   ",len(frontiers)	
	  pp=[]	
	  for q in range(0,len(frontiers)):
	  	
	  	p.x=frontiers[q][0]
          	p.y=frontiers[q][1]
          	pp.append(copy(p))
	
          
#Plotting
	  	
  	  points.points=pp
          pub.publish(points) 
		
	  
	  rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
