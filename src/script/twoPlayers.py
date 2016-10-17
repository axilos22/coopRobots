#!/usr/bin/env python
# --------2 PLAYERS--------
#################### SETUP #################### 
import rospy
from std_msgs.msg import String
import math
import random
#module setup
rospy.init_node("player_node",anonymous=True)
rate=rospy.Rate(30)
name=rospy.get_param("~name","blue1")
teamChannel=rospy.get_param("~teamchannel","teamspeak")
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
minDistanceToGoal=.2
minOrientationToGoal= .6
sleepTimeAvoidance=1
maxFieldLength=4
#Surrounding knwoledge
minProximity=.5
players=["blue1","blue2","blue3","yellow1","yellow2","yellow3"]
players.remove(name) #removed myselft from the list of players
#linear and angular speeds for turning around ball
ratio = 5.5
v = 500
omega = -v/ratio
### Proportional gain
Kro= 500
Kalpha= 550
Kbeta= -400
#################### FUNCTIONS #################### 
desiredPose=[0,0,0]
def errorVector():
	[x,y,th] = poseVector()
	deltaX=desiredPose[0]-x
	deltaY=desiredPose[1]-y
	deltaTh=desiredPose[2]-th
	return [deltaX,deltaY,deltaTh]
def poseVector():
	x=player._pose[name].x
	y=player._pose[name].y
	th=player._pose[name].theta
	return [x,y,th]
def ro():
	global deltaX
	global deltaY
	[deltaX,deltaY,deltaTheta] = errorVector()
	return math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2))
def alpha():
	[x,y,th] = poseVector()
	[deltaX,deltaY,deltaTheta] = errorVector()
	#~ rospy.loginfo("robotPose [%f,%f,%f]",x,y,th)
	#~ rospy.loginfo("ErrVect [%f,%f,%f]",deltaX,deltaY,deltaTheta)
	return -th+math.atan2(deltaX,deltaY)
def beta():
	[x,y,th] = poseVector()
	return -th-alpha()
def vw():
	global Kro
	global Kalpha
	global Kbeta
	return [Kro*ro(), \
	Kalpha*alpha()+Kbeta*beta()]
def runMvtCheck():
	el = checkSurrounding()
	rospy.logwarn("Collision warning: obj %s",el)
	if el != "":
		#Avoid collision
		player.move(0,0)
		while -player.rel[el][1] < math.pi/2:
			player.move(0,100)
			rospy.sleep(sleepTime)
		player.move(1000,0)
		rospy.sleep(sleepTimeAvoidance)
		return True
	else:
		return False
def checkSurrounding():
	collider=""
	for el in players:
		distance=player.rel[el][0]
		rospy.loginfo("checked %s d=%f",el,distance)
		if distance<collisionThreshold:
			collider=el
	return collider
def isScored():
	relDGoal= player.rel["yellow_goal"][0]
	relDBall= player.rel["ball"][0]
	relOGoal= player.rel["yellow_goal"][1]
	relOBall= player.rel["ball"][1]
	if abs(relDGoal-relDBall)<minDistanceToGoal and abs(relOGoal-relOBall)<minOrientationToGoal:
		rospy.loginfo("I probably scored. FWI:goal dist: %s",player.rel["yellow_goal"][0])
		return True
	else:
		return False
def stopNode():
	player.move(0,0)
	rospy.logwarn("The node %s will now shutdown",name)
def goto(location):
	turnToward(location)
	rospy.logdebug("Turning toward done")
	dist2location=maxFieldLength
	while dist2location>approachThreshold:
		dist2location=approach(location)
		if runMvtCheck()==True:
			turnToward(location)
	rospy.logdebug("location reached")
	return
def checkObstacle():
	for el in players:
		distToElement=player.rel[el]
		if distToElement<minProximity:
			return [True,players.index(el)]
	return (False,0)
def turnToward(location,threshold=turnThreshold):
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
#approach has to be iterative, we can add additional watchdogs around it
def approach(location,threshold=approachThreshold):
	player.move(approachSpeed,0)
	rospy.sleep(sleepTime) 
	player.move(0,0)
	#return the current distance to goal
	return player.rel[location][0]
#passBall allow to pass the ball to a receiver (player or not)
#it suppose that we are close to ball
def passBall(receiver):
	#Turn fron pi/2 
	while -player.rel["ball"][1] < math.pi/2:
		player.move(0,100)
		rospy.sleep(sleepTime)
	#Orbit around
	while abs(player.rel["ball"][1]-player.rel[receiver][1])>.2:
		diff=player.rel["ball"][1]-player.rel[receiver][1]
		player.move(v,omega)
		rospy.sleep(sleepTime)
	player.move(0,0)
	turnToward("ball")
	shoot()
	rospy.loginfo("Sleeping for 8s for ball intertia")
	rospy.sleep(8)
	return
def shoot():
	#as long as no contact, go forward
	while player.rel["ball"][0]>contactDistancewithBall:
		player.move(1000,0)
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
		#rospy.loginfo("Did nothing because not leading")
		rospy.loginfo("Not my turn, I go toward goal")
	return
#################### EXECUTIVE SECTION #################### 
#INIT PART
player.move(0,0) #player is initially stopped
#~ loop=0
#~ pub=rospy.Publisher(teamChannel,String,queue_size=1)
#~ rospy.Subscriber(teamChannel,String,behave)
#~ rospy.on_shutdown(stopNode)
#~ once=True
#LOOP PART
#~ while not rospy.is_shutdown():
	#~ if isLeading==True and once==True:
		#~ pub.publish(name)
		#~ rospy.loginfo("[%d] %s:Published me",loop,name)
	#~ if isScored()==True:
		#~ rospy.loginfo("[%d] GOAL of %s !!!",loop,name)
		#~ break
	#~ rospy.sleep(1)
	#~ loop+=1
loop=0
rospy.on_shutdown(stopNode)
while not rospy.is_shutdown():
	[v,w] = vw()
	player.move(v,w)
	loop+=1
	rospy.loginfo("[%d] applied V=%f W=%f",loop,v,w)
	rate.sleep()
rospy.loginfo("Reached the end of the node. Exitting...")
