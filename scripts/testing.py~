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
from sklearn.cluster import KMeans
from sklearn.cluster import MeanShift
import numpy as np
#-----------------------------------------------------

    




    	
#Initalize robot data dictionary list
robot_data=[]
for j in range(0,3):

	  	x_pos=[array(  [   0,2   ]   )]
	  	
	  	robot_data=robot_data+[{'ID':j , 'position':x_pos,   'commanded':x_pos  }]
   	
robot_data[0]['position']=100  	
print len(robot_data)
