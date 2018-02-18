#!/usr/bin/env python

import rospy
import brickpi3 
import numpy as np
import math
import time
from std_msgs.msg import String, Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class

# TO DO. Set your number of joints here
nrOfJoints=3
qNow=np.array([0.0] * nrOfJoints) # This will be our joints state, global access in this script
# This int can be used to check wether there are pending requests to be attended by the planner or not
pendantRequest = 0
# TO DO. Student definitions, customize this with your data
#Specify ports you are using for your JOINTS in order, size must be the same than nrOfJoints
jointsOrder = [BP.PORT_A, BP.PORT_B, BP.PORT_C]
# TO DO. Again, same size than your kinematic model in the planner or failure will be upon you, conversion from radians or mm to degrees of motors
jointsScale = np.array([180/math.pi, -180/math.pi, 180/math.pi])

def callbackFigure(msg):
        if(msg.data == 1 ):
                task1()
        if(msg.data == 2 ):
                task2()
        if(msg.data == 3 ):
                task3()       


# callbackPlanner(data) This function is triggered when a ROS message is being received while in rate.sleep()
# TO DO. If you don't feel a good control over your robot, you should modify this function
def callbackPlanner(data):
	global BP # We need to specify the globals to be modified
	global qNow 
	global pendantRequest 

	# TO DO. How many degrees of precision for each motor do you want?
	degreesThreshold = np.array([7, 4, 4]) # You can also specify the same for every joint as done in kp and ki
	kp = [100, 25, 30]# * len(data.points[0].positions) #You can also specify a kp for each joint as done in degreesThreshold
	ki = [50, 25, 30]# * len(data.points[0].positions) 
	kd = [85, 20, 20]# * len(data.points[0].positions) 
	w = len(data.points[0].positions)
	h = len(data.points)
	Matrix = [[0 for x in range(w)] for y in range(h)]
	for idy, point in enumerate(data.points):
		#Wait until motors reach each point
		allReached = False
		qReached=[False] * len(data.points[0].positions)		
		timeBase= time.time()
		while not allReached:
                        print("reaching")
                        print(point.positions)
			for idx, jointPos in enumerate(point.positions):
				#Tell the motors to move to the desired position
				K = kp[idx] + ki[idx] * (time.time()-timeBase)
				if K > 1000: K = 1000
				BP.set_motor_position_kp(jointsOrder[idx], K) # We can cheat the control and add an integrator varying kp over time
				BP.set_motor_position_kd(jointsOrder[idx], kd[idx])
				BP.set_motor_position(jointsOrder[idx], jointPos*jointsScale[idx])
			for idx, jointPort in enumerate(jointsOrder):
                                print(BP.get_motor_encoder(jointPort))
                                print(int(round(data.points[idy].positions[idx]*jointsScale[idx])))
				if abs(BP.get_motor_encoder(jointPort) - int(round(data.points[idy].positions[idx]*jointsScale[idx]))) < degreesThreshold[idx]:
					qReached[idx]=True					
				if all(flag == True for flag in qReached):
					allReached=True 


	# Now the request has been attended, let's refresh qNow values
	for idx, q in enumerate(jointsOrder):
		qNow[idx] = float(BP.get_motor_encoder(q))/float(jointsScale[idx])
	#Request has been attended, success!
	pendantRequest = pendantRequest - 1
	rospy.loginfo("Request attended")


def fakeCallbackPlanner(data):
	global BP # We need to specify the globals to be modified
	global qNow 
	global pendantRequest 

	# TO DO. How many degrees of precision for each motor do you want?
	degreesThreshold = np.array([7, 4, 4]) # You can also specify the same for every joint as done in kp and ki
	kp = [100, 25, 30]# * len(data.points[0].positions) #You can also specify a kp for each joint as done in degreesThreshold
	ki = [50, 25, 30]# * len(data.points[0].positions) 
	kd = [85, 20, 20]# * len(data.points[0].positions) 
	w = len(data.points[0].positions)
	h = len(data.points)
	Matrix = [[0 for x in range(w)] for y in range(h)]
	for idy, point in enumerate(data.points):
		#print the response of the planner
		scales=[0.428, 1 ,1]
		print(point.positions)
		THRESH = 3.1
                for i in range(0,len(point.positions)-1):
                        if (point.positions[i]*scales[i]> THRESH) or (point.positions[i]*scales[i]< -THRESH):
                                print("Singularities Detected")
                                os._exit(0)
	# Now the request has been attended, let's refresh qNow values
	qNow = data.points[-1].positions
	#Request has been attended, success!
	pendantRequest = pendantRequest - 1
	rospy.loginfo("Request attended")

#TO DO: Define the callback of the variant topic subscription here

# ROS publisher and subscriber
rospy.init_node('task', anonymous=True)
rate = rospy.Rate(0.5)
pub = rospy.Publisher('request', String, queue_size=100)
subP = rospy.Subscriber('trajectory', JointTrajectory, callbackPlanner)
subV = rospy.Subscriber('figure_code', Int16, callbackFigure)
#TO DO: Subscribe to the variant topic in a similar way as the line before specyfing its callback, check the MATLAB script for the message type

# This function sends the ros message (your request)
	#Format of the request : 
	# A char , L or J for traj type, "space" 
	# Current state of the joints, separated by commas
	# Target position in world frame, separated by commas "space" 
	#rotation in EulerZYX, separated by commas "space" 
	# Number of points to interpolate
def sendRequest(typeTraj, qNow, xyz, eulerzyx, points = 1):
	global pendantRequest
	# Trajectory type, pass it as a 	
	request_str = typeTraj + ' ' 
	# QNow, pass it as a numpy array
	for q in qNow:
		request_str = request_str + str(q) + ','
	request_str = request_str[:-1] + ' ' # Remove last comma, add space for next word
	# XYZ, pass it as a numpy array
	for u in xyz:
		request_str = request_str + str(u) + ','
	request_str = request_str[:-1] + ' ' 
	# EulerZYX, pass it as a numpy array
	for a in eulerzyx:
		request_str = request_str + str(a) + ','
	request_str = request_str[:-1] + ' ' 
	# joint Points to be returned by the planner, pass it as an int number
	request_str = request_str + str(points)
	# Send the request_str
	pub.publish(request_str)
	# If we send request, it is pendant to be answered
	pendantRequest = pendantRequest + 1 

# This is the main function
def task():
	global BP
	#Initialize the state of our joints, let's them be global to be refreshed on every movement
	#This variable should have their value in radians or as interpreted by your model in the planner

	# TO DO. Student code should start here:
	BP.reset_all()
	BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #Reset encoder A
	BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #Reset encoder B
	BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #Reset encoder C
	BP.set_motor_position(BP.PORT_A, 0)
	BP.set_motor_position(BP.PORT_B, 0)
	BP.set_motor_position(BP.PORT_C, 0)
	# Tell the motors not to be in a hurry, slowly is more accurate
	BP.set_motor_limits(BP.PORT_A, 130) 
	BP.set_motor_limits(BP.PORT_B, 130) 
	BP.set_motor_limits(BP.PORT_C, 1000)
	

	# Request the planner to move our robot
	rate.sleep() # Wait a while before requesting
	task3()
	while not rospy.is_shutdown():
		rate.sleep()
		rospy.loginfo("Pendant requests: %s", pendantRequest)

horizontal = 1
vertical = 0
upPosition = 68
downPosition = 15

# _________________
#|   ___________   |
#|  |           |  |
#|  |___________|  |
#|	 ___________   |
#|  |___|___|___|  |
#|  |___|___|___|  |
#|  |___|___|___|  |
#|  |___|___|___|  |
#|_________________|
def task1():
	height = 90
	width  = 60
	xPosition = 80 #bottom right of the outter rectangle
	yPosition = -width/2 # to have x axis centerd
	internalMargin = 10
	topHeight = 20 #height of inner top rectangle
	bottomHeight = 40 #height of inner bottom rectangle
	nPoints = 40
	#external rectangle
	drawRectangle(	height,		
					width, 					
					xPosition, 
					yPosition, 
					nPoints)
	#internal top rectangle
	drawRectangle(	topHeight, 	
					width-internalMargin*2,	
					xPosition+height-internalMargin-topHeight,
					yPosition+internalMargin,
					nPoints)
	#internal bottom rectangle
	drawRectangle(	bottomHeight, 	
					width-internalMargin*2,	
					xPosition+internalMargin,
					yPosition+internalMargin,
					nPoints)
	#draw horizontal lines for internal bottom rectangle
	for i in range(1,4):
		drawLine(	0,
					width-internalMargin*2,
					xPosition+internalMargin+float(i)/4*bottomHeight,
					yPosition+internalMargin,
					horizontal,
					nPoints)
	#draw vertical lines for internal bottom rectangle
	for i in range(1,3):
		drawLine(	bottomHeight,
					0,
					xPosition+internalMargin,
					yPosition+internalMargin+float(i)/3*(width-internalMargin*2),
					vertical,
					nPoints)

        BP.set_motor_position_kp(BP.PORT_A, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_A, 3)
        BP.set_motor_position_kp(BP.PORT_B, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_B, 3)
        BP.set_motor_position(BP.PORT_A, 0)
        BP.set_motor_position(BP.PORT_B, 0)
        BP.set_motor_position(BP.PORT_C, 0)
	
# _________________
#|   ___________   |
#|  |           |  |
#|  |			|  |
#|	| 			|  |
#|  |___________|  |
#|_________________|
#|   ___ ___ ___   |
#|  |___|___|___|  |
#|_________________|
def task2():
	rotation = 0.0# -np.pi/2.0
	height = 90
	width  = 60
	xPosition = 90
	yPosition = -90
	internalMargin = 10
	topHeight = 40 #height of inner top rectangle
	bottomHeight = 10 #height of inner bottom rectangle
	nPoints = 40
	#external rectangle
	drawRectangle(	height,		
					width, 					
					xPosition, 
					yPosition, 
					nPoints
					)
	#internal top rectangle
	drawRectangle(	topHeight, 	
					width-internalMargin*2,	
					xPosition+height-internalMargin-topHeight,
					yPosition+internalMargin,
					nPoints
					)
	#draw dividing line
	drawLine(0,width,xPosition+(internalMargin*2)+bottomHeight,yPosition,horizontal,nPoints,rotation)
	
	#internal bottom rectangle
	drawRectangle(	bottomHeight, 	
					width-internalMargin*2,	
					xPosition+internalMargin,
					yPosition+internalMargin,
					nPoints
					)
	#draw vertical lines for internal bottom rectangle
	for i in range(1,3):
		drawLine(	bottomHeight,
					0,
					xPosition+internalMargin,
					yPosition+internalMargin+float(i)/2*(width-internalMargin*2), #using /2 on purpose for callibration erros
					vertical,
					nPoints
					)

        BP.set_motor_position_kp(BP.PORT_A, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_A, 3)
        BP.set_motor_position_kp(BP.PORT_B, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_B, 3)
        BP.set_motor_position(BP.PORT_A, 0)
        BP.set_motor_position(BP.PORT_B, 0)
        BP.set_motor_position(BP.PORT_C, 0)


# _________________
#|---------------- |
#||  ____________ ||
#||	|		    | ||
#|| |			| ||
#|| | 			| ||
#|| |___________| ||
#||  _    _	   _  ||
#|| |_|  |_|  |_| ||
#||_______________||
#|-----------------|
def task3():
	rotation = np.pi/2.0
	height = 90
	width  = 60
	xPosition = 90
	yPosition = 35
	internalMargin = 10
	internalLittleMargin = 5
	topHeight = 40 #height of inner top rectangle
	bottomHeight = 10 #height of inner bottom rectangle
	nPoints = 40
	#external rectangle
	drawRectangle(	height,		
					width, 					
					xPosition, 
					yPosition, 
					nPoints
					)
	#internal rectangle
	drawRectangle(	height-internalLittleMargin*2, 	
					width-internalLittleMargin*2,	
					xPosition+internalLittleMargin,
					yPosition+internalLittleMargin,
					nPoints
					)
	#internal top rectangle
	drawRectangle(	topHeight, 	
					width-internalMargin*2-internalLittleMargin*2,	
					xPosition+height-internalMargin-internalLittleMargin-topHeight,
					yPosition+internalMargin+internalLittleMargin,
					nPoints
					)
	#internal bottom rectangle first right
	for i in range(0,3):
		localWidth = float(width-internalMargin*2-internalLittleMargin*2)
		drawRectangle(	bottomHeight, 	
						localWidth/4,	
						xPosition+internalMargin+internalLittleMargin,
						yPosition+internalMargin+internalLittleMargin+float(i)/3*localWidth,
						nPoints
						)

        BP.set_motor_position_kp(BP.PORT_A, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_A, 3)
        BP.set_motor_position_kp(BP.PORT_B, 6) # We can cheat the control and add an integrator varying kp over time
        BP.set_motor_position_kd(BP.PORT_B, 3)
        BP.set_motor_position(BP.PORT_A, 0)
        BP.set_motor_position(BP.PORT_B, 0)
        BP.set_motor_position(BP.PORT_C, 0)

	

def waitRequestServed():
	while pendantRequest > 0:
		pass

def drawRectangle(xScale, yScale, xOffset, yOffset, nPoints=5):
	global upPosition
	global downPosition
	global qNow 

	points = np.array([	[0.0,  0.0,  0.0],
						[1.0,  0.0,  0.0],
						[1.0,  1.0,  0.0],
						[0.0,  1.0,  0.0],
                                [0.0,  0.0,  0.0]
						])
			
	for row in points:
		row[0] = row[0]*xScale + xOffset
		row[1] = row[1]*yScale + yOffset


	eulerzyx = np.array([0.0, 0.0, 0.0]) # Don't care about rotation
	posxyz = points[0]
	posxyz[2] = upPosition
	sendRequest('J', qNow, posxyz, eulerzyx, nPoints)
	waitRequestServed()

	for posxyz in points:
		posxyz[2] = downPosition
		print('sending')
		print(row)
		sendRequest('L', qNow, posxyz, eulerzyx, nPoints)
		waitRequestServed()

	posxyz = points[-1]
	posxyz[2] = upPosition
	sendRequest('J', qNow, posxyz, eulerzyx, nPoints)
	waitRequestServed()

# orientation = 1 for horizontal
# orientation = 0 for vertical
def drawLine(xScale, yScale, xOffset, yOffset, orientation, nPoints=5):
	global upPosition
	global downPosition
	global qNow 

	if(orientation == vertical):
		points = np.array([[0.0,  0.0,  0.0],
							[1.0,  0.0, 0.0],
							])
	else:
		points = np.array([[0.0,  0.0,  0.0],
							[0.0,  1.0, 0.0],
							])

	for row in points:
		row[0] = row[0]*xScale + xOffset
		row[1] = row[1]*yScale + yOffset

	eulerzyx = np.array([0.0, 0.0, 0.0]) # Don't care about rotation
	posxyz = points[0]
	posxyz[2] = upPosition
	sendRequest('J', qNow, posxyz, eulerzyx, nPoints)
	waitRequestServed()

	for posxyz in points:
		posxyz[2] = downPosition
		sendRequest('L', qNow, posxyz, eulerzyx, nPoints)
		waitRequestServed()

	posxyz = points[-1]
	posxyz[2] = upPosition
	sendRequest('J', qNow, posxyz, eulerzyx, nPoints)
	waitRequestServed()


if __name__ == '__main__':
	try:
		task()
	except rospy.ROSInterruptException:
		pass

