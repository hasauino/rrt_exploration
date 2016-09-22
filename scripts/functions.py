#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from os import system

from random import random
from numpy import array
from numpy import floor,ceil
from numpy import delete
from numpy import concatenate
from numpy import vstack
from numpy import linalg as LA
from math import copysign
from numpy import where,inf
from numpy import logical_and as AND
from numpy import all as All
from scipy.optimize import minimize

# Nearest function-------------------------------------
def Nearest(V,x):
 n=1000000
 i=0
 for i in range(0,V.shape[0]):
    n1=LA.norm(V[i,:]-x)
    if (n1<n):
	n=n1
        result=i    
 return result

# Steer function-------------------------------------

def myfun(x,x0,x1,eta):
   X=array([x[0],x[1]])
   return LA.norm(X-x1)






def Steer(x0,x1,eta):
	

	def consFun(x):
		X=array([x[0],x[1]])
		x0=p[0]
		eta=p[2]
		return -LA.norm(X-x0)+eta



	cons = ({'type': 'ineq',
                 'fun' : consFun  })
         
       	p=(x0,x1,eta)
	res = minimize(myfun,[x0[0],x0[1]],args=p,constraints=cons, method='COBYLA',options={'disp': False})

	xnew=array([res.x[0],res.x[1]])
		
	return xnew


# gridValue function-------------------------------------
def gridValue(mapData,Xp):
 resolution=mapData.info.resolution
 Xstartx=mapData.info.origin.position.x
 Xstarty=mapData.info.origin.position.y

 width=mapData.info.width
 Data=mapData.data
 # returns grid value at "Xp" location
 #map data:  100 occupied      -1 unknown       0 free
 index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) )
 
 if int(index) < len(Data):
 	return Data[int(index)]
 else:
 	return 100

 
# gridCheck function-------------------------------------
def gridCheck(mapData,Xp):
 resolution=mapData.info.resolution
 Xstartx=mapData.info.origin.position.x
 Xstarty=mapData.info.origin.position.y

 width=mapData.info.width
 Data=mapData.data
 # check if points are in freespace or not
 # c=1 means grid cell occupied
 # c=0 means grid cell is free
 index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) )
 c=1
 if int(index) < len(Data):
 	if Data[int(index)]==0:
  	  c=0
 	else:
 	  c=1 

 return c

# ObstacleFree function-------------------------------------

def ObstacleFree(xnear,xnew,mapsub):
 out=1
 rez=mapsub.info.resolution*0.5
 stepz=int(ceil(LA.norm(xnew-xnear))/rez)
 xi=xnear

 for c in range(0,stepz):
   xi=Steer(xi,xnew,rez)
   if (gridCheck(mapsub,xi) !=0):
     out=0
     
   

 if (gridCheck(mapsub,xnew) !=0):
  out=0
  
 
 return out

# Find function-------------------------------------
def Find(E,x):
 if not All(array([E.shape]).shape==array([1,1])):
	 yy=E==x[1]
	 xx=E==x[0]
	 m=AND(yy[:,3], xx[:,2])
	 m=where(m==True)

	 if len(m[0])>0:
	  return m[0][0]
 else:
	 return 0
# Near function-------------------------------------

def Near(V,xnew,r):
 xnear=array([0,0])
 i=0
 for i in range(0,V.shape[0]):
    n=LA.norm(V[i,:]-xnew)
    if (n<=r):
        p=V[i,:]
        xnear=vstack((xnear,p))


 xnear=delete(xnear, (0), axis=0)
 return xnear

# Cost function-------------------------------------

def Cost(E,xn):
	x=xn
	if All(array([E.shape]).shape==array([1,1])):
		c=0
		
	else:
		xinit=E[0,0:2]

		c=0
		while not All(x==xinit):
	        	xp=E[Find(E,x),0:2]
			c+=LA.norm(x-xp)
			x=xp
		
	return c

# prepEdges function
		
def prepEdges(E):
	p=Point()
	pl=[]
	if not All(array([E.shape]).shape==array([1,1])):	
			
		Ex=delete(E, (1), axis=1)
		Ex=delete(Ex, (2), axis=1)

		Ey=delete(E, (0), axis=1)
		Ey=delete(Ey, (1), axis=1)
		pxs=Ex.flatten()
		pys=Ey.flatten()		
		
		j=0
					
		for j in range(0,pys.shape[0]):			
			p.x=pxs[j]
			p.y=pys[j]
			pl.append(copy(p)) 
					
			
		
	return pl

# Assigner 3 robots------------------------------------------------------------------------------------------------------------------------
def assigner3(goal,x_new,client1,client2,client3,listener):
		clientstate1=client1.get_state()
		clientstate2=client2.get_state()
		clientstate3=client3.get_state()
		
		if clientstate1==2 or clientstate1==3 or clientstate1==4 or clientstate1==5 or clientstate1==9:
			aval1=1
		else:
			aval1=10000000
		
		if clientstate2==2 or clientstate2==3 or clientstate2==4 or clientstate2==5 or clientstate2==9:
			aval2=1
		else:
			aval2=10000000
			
		if clientstate3==2 or clientstate3==3 or clientstate3==4 or clientstate3==5 or clientstate3==9:
			aval3=1
		else:
			aval3=10000000
		
		
	
		
		
		(trans1,rot) = listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))  
		(trans2,rot) = listener.lookupTransform('/robot_1/map', '/robot_2/base_link', rospy.Time(0)) 
		(trans3,rot) = listener.lookupTransform('/robot_1/map', '/robot_3/base_link', rospy.Time(0)) 
		
		dist1=LA.norm(array([ trans1[0],trans1[1] ])-x_new)*aval1
		dist2=LA.norm(array([ trans2[0],trans2[1] ])-x_new)*aval2
		dist3=LA.norm(array([ trans3[0],trans3[1] ])-x_new)*aval3
		
		alldist=[dist1,dist2,dist3]
		
		# if no robot is available wait
		while aval1==aval2==aval3==10000000:
			clientstate1=client1.get_state()
			clientstate2=client2.get_state()
			clientstate3=client3.get_state()
		
			if clientstate1==2 or clientstate1==3 or clientstate1==4 or clientstate1==5 or clientstate1==9:
				aval1=1
			else:
				aval1=10000000
		
			if clientstate2==2 or clientstate2==3 or clientstate2==4 or clientstate2==5 or clientstate2==9:
				aval2=1
			else:
				aval2=10000000
			
			if clientstate3==2 or clientstate3==3 or clientstate3==4 or clientstate3==5 or clientstate3==9:
				aval3=1
			else:
				aval3=10000000
			
		goal.target_pose.pose.position.x=x_new[0]
	    	goal.target_pose.pose.position.y=x_new[1]
    		goal.target_pose.pose.orientation.w = 1.0
			
		#send command to the lowest cost available robot	
		if min(alldist)==dist1 and aval1==1:
			
			client1.send_goal(goal)
			#client1.wait_for_result()
			#client1.get_result()		
		elif min(alldist)==dist2 and aval2==1:
			
			client2.send_goal(goal)
			#client2.wait_for_result()
			#client2.get_result()
		elif min(alldist)==dist3 and aval3==1:
			
			client3.send_goal(goal)
			#client3.wait_for_result()
			#client3.get_result()
		
		return 0
			

# Assigner 1 robots------------------------------------------------------------------------------------------------------------------------
def assigner1(goal,x_new,client1,listener):
	#client1.send_goal(goal)
	#client1.wait_for_result()
        #client1.get_result() 
	clientstate1=client1.get_state()
		
	if clientstate1==2 or clientstate1==3 or clientstate1==4 or clientstate1==5 or clientstate1==9:
		client1.send_goal(goal)
	


		
	return 0
	
# Assigner 1 robots  opecv detector------------------------------------------------------------------------------------------------------------------------
def assigner1new(goal,x_new,client1,listener):
	goal.target_pose.pose.position.x=x_new[0]
	goal.target_pose.pose.position.y=x_new[1]
    	goal.target_pose.pose.orientation.w = 1.0
	clientstate1=client1.get_state()
		
	if clientstate1==2 or clientstate1==3 or clientstate1==4 or clientstate1==5 or clientstate1==9:
		client1.send_goal(goal)
	


		
	return 0
	


#-------------RRT frontier

# oObstacleFree function-------------------------------------

def ObstacleFree2(xnear,xnew,mapsub):
 
 rez=mapsub.info.resolution*0.5
 stepz=int(ceil(LA.norm(xnew-xnear))/rez)

 xi=xnear
 

 obs=0
 unk=0
 for c in range(0,stepz):
   xi=Steer(xi,xnew,rez)
   if (gridValue(mapsub,xi) ==100):
      obs=1
   if (gridValue(mapsub,xi) ==-1):
      unk=1
     
   

 if (gridValue(mapsub,xnew) ==100):
  obs=1
 
 if (gridValue(mapsub,xi) ==-1):
   unk=1
  
 
 
 
 
 if unk==1:
 	out=-1
 	
 if obs==1:
 	out=0
 		
 if obs!=1 and unk!=1:
        out=1
 #print "obs= ",obs,"    unk=    ",unk,"      out=     ",out
 #raw_input(" ")
 return out



#  assigner1rrtfront(goal,frontiers,client1,listener) ----------------------------------------------------

def Nearest2(V,x):
 n=inf
 result=0
 for i in range(0,len(V)):
    n1=LA.norm(V[i]-x)
    
    if (n1<n):
	n=n1
        result=i    
 return result


def assigner1rrtfront(goal,frontiers,client1,xp):

	clientstate1=client1.get_state()
		
	if clientstate1==2 or clientstate1==3 or clientstate1==4 or clientstate1==5 or clientstate1==9:
		if len(frontiers)>0:
    			row=Nearest2(frontiers,xp)
	    		nextfrontier=frontiers[row]
	    		frontiers=delete(frontiers, (row), axis=0)
	    		goal.target_pose.pose.position.x=nextfrontier[0]
			goal.target_pose.pose.position.y=nextfrontier[1]
    			goal.target_pose.pose.orientation.w = 1.0
    			print "exploration goal sent"
			client1.send_goal(goal)
	


		
	return frontiers























