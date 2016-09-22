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
from functions import Nearest,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue,assigner1rrtfront

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    
    

    

# Node----------------------------------------------
def node():

	rospy.init_node('rrt_frontier_detector', anonymous=False) 
# fetching all parameters
	eta = rospy.get_param('~eta',0.7)	
	init_map_x=rospy.get_param('~init_map_x',20.0)
	init_map_y=rospy.get_param('~init_map_y',20.0)
	ns=rospy.get_namespace()
		
	map_topic=rospy.get_param('~map_topic',ns+'map')
	base_frame_topic=rospy.get_param('~base_frame_topic','base_link')
#-------------------------------------------
	global mapData
	exploration_goal=Point()
	
    	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	targetspub = rospy.Publisher('/exploration_goals', Point, queue_size=10)
    	pub = rospy.Publisher('shapes', Marker, queue_size=10)
    	 	
    	
   		 	   	
    	rate = rospy.Rate(100)	


	

# wait until map is received, when a map is received, mapData.header.seq will not be < 1
	while mapData.header.seq<1 or len(mapData.data)<1:
		pass
	
        	
        
        listener = tf.TransformListener()
	listener.waitForTransform(mapData.header.frame_id, ns+base_frame_topic, rospy.Time(0),rospy.Duration(10.0))
        
        try:
		(trans,rot) = listener.lookupTransform(mapData.header.frame_id, ns+base_frame_topic, rospy.Time(0))
		
		
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		trans=[0,0]
		
	xinx=trans[0]
	xiny=trans[1]	

	x_init=array([xinx,xiny])
	
	V=array([x_init])
	i=1.0
	E=concatenate((x_init,x_init))	

    	points=Marker()
       	line=Marker()
       	
    
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
    	points.header.frame_id=line.header.frame_id=mapData.header.frame_id
    	points.header.stamp=line.header.stamp=rospy.Time.now()
	
    	points.ns=line.ns = "markers"
    	points.id = 0
    	line.id =1
	
    	points.type = Marker.POINTS
    	line.type=Marker.LINE_LIST
#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    	points.action = line.action = Marker.ADD;
	
    	points.pose.orientation.w = line.pose.orientation.w = 1.0;
	
    	line.scale.x = line.scale.y= 0.06;
    	points.scale.x=points.scale.y=0.3; 
   
	
    	line.color.r =0.0#9.0/255.0
	line.color.g= 0.0#91.0/255.0
	line.color.b =0.0#236.0/255.0
    	points.color.r = 255.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 0.0/255.0
   	
    	points.color.a=1;
	line.color.a = 1;#0.6;
    	points.lifetime =line.lifetime = rospy.Duration();
	

    	p=Point()

    	p.x = x_init[0] ;
    	p.y = x_init[1] ;
    	p.z = 0;

    	pp=[]
        pl=[]
    	pp.append(copy(p))
    

	frontiers=[]
	xdim=mapData.info.width
	ydim=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y 
	

#-------------------------------RRT------------------------------------------
	while not rospy.is_shutdown():
	  
# Sample free
          xr=(random()*init_map_x)-(init_map_x*0.5)
	  yr=(random()*init_map_y)-(init_map_y*0.5)
	  x_rand = array([xr,yr])
	  
# Nearest
	  x_nearest=V[Nearest(V,x_rand),:]

# Steer
	  x_new=Steer(x_nearest,x_rand,eta)

# ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
	  checking=ObstacleFree2(x_nearest,x_new,mapData)
	  
	  if checking==-1:
          	
          	exploration_goal.x=x_new[0]
          	exploration_goal.y=x_new[1]
          	exploration_goal.z=0.0
          	targetspub.publish(exploration_goal)	
          	
          	(trans,rot) = listener.lookupTransform(mapData.header.frame_id, ns+base_frame_topic, rospy.Time(0))
	  	xinx=trans[0]
		xiny=trans[1]	
		x_init=array([xinx,xiny])
	  	V=array([x_init])
	  	E=concatenate((x_init,x_init))
	  	pp=[]
        	pl=[]
	  	
	  
	  elif checking==1:
	 	V=vstack((V,x_new))	
	 	temp=concatenate((x_nearest,x_new))        
	        E=vstack((E,temp))

          
#Plotting
	  	
  	  points.points=[exploration_goal]
          pl=prepEdges(E)
          line.points=pl
          pub.publish(line)        
          pub.publish(points) 
		
	  #raw_input("")
	  rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
