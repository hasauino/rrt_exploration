#!/usr/bin/env python


#--------Include modules---------------
import rospy
from assigner2_functions import robot

    
# Node----------------------------------------------

def node():
	rospy.init_node('test', anonymous=False)
	rate = rospy.Rate(100)
	robot1=robot('/robot_1')

	
	

	while not rospy.is_shutdown():
		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
