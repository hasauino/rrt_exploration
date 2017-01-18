#!/usr/bin/env python


#--------Include modules---------------
import rospy
from assigner2_functions import robot,informationGain
import thread
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
 
mapData=OccupancyGrid() 
frontiers=[]

def callBack(data):
	global mapData
	point=[data.x,data.y]
	print 'infogain= ',informationGain(mapData,point,5.0)
    
def mapCallBack(data):
    global mapData
    mapData=data     
# Node----------------------------------------------
def node():
	global mapData
	rospy.Subscriber('/map_merge/map', OccupancyGrid, mapCallBack)
	rospy.Subscriber('/exploration_goals', Point, callBack)
	rospy.init_node('test', anonymous=False)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
