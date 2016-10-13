#!/usr/bin/env python
# --------2 PLAYERS--------
#################### SETUP #################### 
import rospy
from std_msgs.msg import String, Float64MultiArray, Header
import math
import random
#module setup
rospy.init_node("player_node",anonymous=True)
rate=rospy.Rate(2)
name=rospy.get_param("~name","blue1")
teamChannel=rospy.get_param("~teamchannel","/teamspeak")
partner=rospy.get_param("~partner","blue2")
isLeading=rospy.get_param("~isLeading",False)
rospy.loginfo("Node initialized: name=%s teamchannel=%s partner=%s isLeading=%d",name,teamChannel,partner,isLeading)
#player setup
from soccer.controller import RobotController
player = RobotController(name)
#sleep for 1s (controller sync)
rospy.sleep(1)
#Tuning parameters
turningSpeed = 500
facingSpeed = 200
approachSpeed = 1000
sleepTime = .1
turnThreshold = .09
approachThreshold = .2
collisionThreshold=.12
contactDistancewithBall=.09
minDistanceToGoal=.04
minOrientationToGoal= .6
sleepTimeAvoidance=1.5
maxFieldLength=4
#Surrounding knwoledge
minProximity=.5
players=["blue1","blue2","blue3","yellow1","yellow2","yellow3"]
players.remove(name) #removed myselft from the list of players
#linear and angular speeds for turning around ball
ratio = 5.5
v = 500
omega = -v/ratio
#----------------------------------FUNCTIONS----------------------------------
####################  MOVEMENT #################### 
def checkHeading(location):
	"""If the heading is not correct, stop and turn toward it again"""
	if player.rel[location][1] >turnThreshold:
		rospy.logwarn("Heading incorrect, adjusting trajectory")
		player.move(0,0)
		return True
	else:
		return False
def runMvtCheck(location):
	"""Function holding all the test to perform while on the move"""
	isChanged = False
	isChanged = checkHeading(location)
	el = checkSurrounding()
	if el != "":
		rospy.logwarn("Collision warning: obj %s",el)
		#Avoid collision
		player.move(0,0)
		if random.random()>.5:
			speed=800
		else:
			speed=-800
		while abs(-player.rel[el][1]) < math.pi/2:
			player.move(0,speed)
			rospy.sleep(sleepTime)
		player.move(1000,0)
		rospy.sleep(sleepTimeAvoidance)
		isChanged = True
	return isChanged
def checkSurrounding():
	"""Checks for collision with other player"""
	collider=""
	for el in players:
		distance=player.rel[el][0]
		rospy.logdebug("checked %s d=%f",el,distance)
		if distance<collisionThreshold:
			collider=el
	return collider
def goto(location,appThreshold=approachThreshold):
	"""go to the specified location safely"""
	turnToward(location)
	rospy.logdebug("Turning toward done")
	dist2location=maxFieldLength
	while dist2location>approachThreshold:
		dist2location=approach(location)
		if runMvtCheck(location)==True:
			turnToward(location)
	rospy.logdebug("location reached")
	return
def turnToward(location,threshold=turnThreshold):
	"""Face a specified object using a tunable treshold"""
	relOri = -player.rel[location][1]
	if relOri > 0:
		speed = -turningSpeed
	else:
		speed = turningSpeed
	lowerSpeed=speed/2
	while abs(-player.rel[location][1])>threshold:
		if abs(-player.rel[location][1])<.3:
			speed=lowerSpeed
		player.move(0,speed)
		rospy.sleep(sleepTime)
		player.move(0,0)
	return
def approach(location,threshold=approachThreshold):
	"""Perform one iteration of the approaching sequence (iterative function)"""
	player.move(approachSpeed,0)
	rospy.sleep(sleepTime) 
	player.move(0,0)
	#return the current distance to goal
	return player.rel[location][0]
#passBall allow to pass the ball to a receiver (player or not)
#it suppose that we are close to ball
### ACTIONS ###
def passBall(receiver):
	"""Passes the ball to another element. Sequence:
	-turn from pi/2
	-perform uniform cyclic motion
	-face the ball
	-go full speed till contact
	(contact function may not work because of simulator collision management)"""
	#Turn fron pi/2 
	while -player.rel["ball"][1] < math.pi/2:
		player.move(0,100)
		rospy.sleep(sleepTime)
	#Orbit around
	inPosition=False
	while abs(player.rel["ball"][1]-player.rel[receiver][1])>.2:
		player.move(v,omega)
		if player.rel["ball"][0] > approachThreshold*1.3:
			rospy.logerr("The ball went too far to orbit around. Shoot now !")
			break
		rospy.sleep(sleepTime)
	player.move(0,0)
	turnToward("ball")
	shoot()
	return
def shoot():
	"""Go forward as long as no collision with the ball"""
	while player.rel["ball"][0]>contactDistancewithBall:
		player.move(1000,0)
		if abs(player.rel["ball"][1])>math.pi/2:
			rospy.logerr("Something went wrong while shooting,\n ball not there. ball ori=%f",abs(player.rel["ball"][1]))
			break
	player.move(0,0)
#################### COMMUNICATION #################### 
def behave(msg):
	global once
	if msg.data==name:
		once=False
		rospy.loginfo("My turn")
		goto("ball")
		passBall("yellow_goal")
		pub.publish(partner)
	else:
		rospy.loginfo("It is not my turn. I wait")
	return
### CONDITIONS ###
def updateBall(pos):
	ballx=pos.data[0]
	bally=pos.data[1]
	rospy.loginfo("UPDATE BALL x:%f y:%f",ballx,bally)
def isScored():
	#Yellow goal coor: x= -1.5 -> -1.7 / y= -.3 -> .3
	#blue goal coor: x= 1.5->1.7 / y=-.3 -> .3
	relBall= player.rel["ball"]
	relYellowGoal=player.rel["yellow_goal"]
	relBlueGoal=player.rel["blue_goal"]
	if abs(relYellowGoal[0]-relBall[0])<minDistanceToGoal and abs(relYellowGoal[1]-relBall[1])<minOrientationToGoal:
		rospy.loginfo("Score into YELLOW goal. DiffDist=%f, DiffAngle=%f",relYellowGoal[0]-relBall[0],relYellowGoal[1]-relBall[1])
		return True
	elif abs(relBlueGoal[0]-relBall[0])<minDistanceToGoal and abs(relBlueGoal[1]-relBall[1])<minOrientationToGoal:
		rospy.loginfo("Score into BLUE goal. DiffDist=%f, DiffAngle=%f",relBlueGoal[0]-relBall[0],relBlueGoal[1]-relBall[1])
		return True
	else:
		return False
def stopNode():
	player.move(0,0)
	rospy.logwarn("The node %s will now shutdown",name)
#----------------------------------EXECUTION----------------------------------
player.move(0,0) #player is initially stopped
loop=0
pub=rospy.Publisher(teamChannel,String,queue_size=1)
rospy.Subscriber(teamChannel,String,behave)
rospy.on_shutdown(stopNode)
#Once allow the leader to only push his leading message once
#made to prevent from a leader to read his own published message, denying partner's turn
once=True
#LOOP PART
while not rospy.is_shutdown():
	if isLeading==True and once==True:
		pub.publish(name)
		rospy.loginfo("[%d] %s:Published me",loop,name)
	if isScored()==True:
		rospy.loginfo("[%d] GOAL of %s !!!",loop,name)
		break
	rate.sleep()
	loop+=1
rospy.loginfo("Reached the end of the node. Exitting...")
player.move(0,0)

### SCRAP ###
		#~ if player.rel["yellow_goal"][0]>2:
			#~ rospy.loginfo("I am slowly going close to goal")
			#~ turnToward("yellow_goal")
			#~ player.move(100,0)
		#~ else:
			#~ turnToward("ball")
			#~ player.move(0,0)
			#~ rospy.loginfo("I am in position")
