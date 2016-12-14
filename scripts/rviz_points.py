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


# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]

def callBack(Data):
    global pub
    pub.publish(Data.point)


# Node----------------------------------------------
def node():

	global pub
	
#-------------------------------------------
	rospy.init_node('rviz_to_goals', anonymous=True)
	
    	rospy.Subscriber("/clicked_point", PointStamped, callBack)
    	pub = rospy.Publisher('/exploration_goals', Point, queue_size=10)
	rospy.spin()
          
    

#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
