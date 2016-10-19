#!/usr/bin/env python
# --------2 PLAYERS--------
#################### SETUP #################### 
import rospy
from std_msgs.msg import String
import math
import random
import matplotlib.pyplot as plt
#module setup
rospy.init_node("player_node",anonymous=True)
rate=rospy.Rate(30)
name=rospy.get_param("~name","blue1")
teamChannel=rospy.get_param("~teamchannel","teamspeak")
partner=rospy.get_param("~partner","blue2")
opGoal=rospy.get_param("~oppositeGoal","yellow_goal")
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
#~ Kro=500
#~ Kalpha=800
#~ Kbeta=-100
#Nice values
Kro=3.0
Kalpha=8.0
Kbeta=-1.5
## PLOTTING ##
poseData=[]
#################### FUNCTIONS #################### 
#~ desiredPose=[0,0,math.pi]
desiredPose=[0,0,0]
def errorVector(x,y,th):
	deltaX=desiredPose[0]-x
	deltaY=desiredPose[1]-y
	deltaTh=desiredPose[2]-th
	return [deltaX,deltaY,deltaTh]
def poseVector():
	x=player._pose[name].x
	y=player._pose[name].y
	th=player._pose[name].theta
	return [x,y,th]
def ro(deltaX,deltaY):
	return math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2))
def alpha(th,deltaX,deltaY):
	return -th+math.atan2(deltaY,deltaX)
def beta(th,alpha):
	return -th-alpha
def vw():
	global poseData
	[x,y,th]=poseVector()
	poseData.append([x,y,th])
	[deltaX,deltaY,deltaTh]=errorVector(x,y,th)
	ALPHA=alpha(th,deltaX,deltaY)
	RO=ro(deltaX,deltaY)
	BETA=beta(th,ALPHA)
	v=Kro*RO
	w=Kalpha*ALPHA+Kbeta*BETA
	#rescale to coordinates
	[v,w]=rescale(v,w)
	return [v,w]
def rescale(v,w,vMul=7764,vDiv=50,wMul=644,wDiv=50):
	"""rescale(v,w,vMul,vDiv,wMul,wDiv) brings back the mesure of kinematics controller into the proportions of simulator.
	Check https://www.cyberbotics.com/doc/guide/using-the-e-puck-robot for more info about the values."""
	vOut=v*vMul/vDiv
	wOut=w*wMul/wDiv
	return [vOut,wOut]
def isCloseToLocation(location,distThreshold=approachThreshold):
	if player.rel[location][0] <distThreshold:
		return True
	else:
		return False
def getShootingPose():
	[xg,yg]=[player._pose[opGoal].x,player._pose[opGoal].y]
	[xb,yb]=[player._pose["ball"].x,player._pose["ball"].y]
	#y=mx+b
	m=(yb-yg)/(xb-xg)
	b=yg-m*xg
	rospy.loginfo("m=%f b=%f",m,b)
	xs=xb+.1
	ys=m*xs+b
	rospy.loginfo("S=[%f,%f]",xs,ys)
	return [xs,ys]
def filterVW(v,w,minSpeed=200):
	if v>1000:
		v=1000
	if v<-1000:
		v=-1000
	#prevent minimum speed to go too low
	if v<minSpeed:
		v=minSpeed
	#~ if w>1000:
		#~ w=1000
	#~ if w<-1000:
		#~ w=-1000
	return [v,w]
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
def approach(location,threshold=approachThreshold):
	"""approach has to be iterative, we can add additional watchdogs around it"""
	player.move(approachSpeed,0)
	rospy.sleep(sleepTime) 
	player.move(0,0)
	#return the current distance to goal
	return player.rel[location][0]
def passBall(receiver):
	"""passBall allow to pass the ball to a receiver (player or not) it suppose that we are close to ball """
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
	"""as long as no contact, go forward"""
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
	#~ [v,w]=filterVW(v,w)
	rospy.loginfo("[%d] applied V=%f W=%f",loop,v,w)
	player.move(v,w)
	[xs,ys]=getShootingPose()
	if isCloseToLocation("ball")==True:
		break
	loop+=1
	rate.sleep()
rospy.loginfo("Reached the end of the node. Exitting...")
plt.plot(poseData)
plt.legend(("x","y","th"))
plt.show()
