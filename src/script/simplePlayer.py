#!/usr/bin/env python
# --------SIMPLE PLAYER--------
#SETUP
import rospy
from std_msgs.msg import String
import math
import random
#module setup
rospy.init_node("player_node",anonymous=True)
rate=rospy.Rate(10)
name= rospy.get_param("~name")
#player setup
from soccer.controller import RobotController
teamChannel="/teamspeak"
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
contactDistancewithBall=.09
minDistanceToGoal = .4
#linear and angular speeds for turning around ball
ratio = 5.5
v = 500
omega = -v/ratio

def goto(location):	
	turnToward(location)
	print(name,"Turning toward done")
	dist2location=99999
	while dist2location>approachThreshold:
		dist2location=approach(location)
	print(name,"location reached")
	return
def turnToward(location,threshold=turnThreshold):
	relOri = -player.rel[location][1]
	if relOri > 0:
		speed = -turningSpeed
	else:
		speed = turningSpeed
	while abs(-player.rel[location][1])>threshold:
		if abs(-player.rel[location][1])<.3:
			speed=250
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
	print("Sleeping for 8s for ball intertia")
	rospy.sleep(8)#sleeping while the ball is having inertia
	return
def shoot():
	#as long as no contact, go forward
	while player.rel["ball"][0]>contactDistancewithBall:
		player.move(1000,0)
	player.move(0,0)
#### COMMUNICATION ####
#none for the moment

#### EXECUTIVE SECTION #### 
player.move(0,0) #player is initially stopped
loop=0
while player.rel["yellow_goal"][0]>minDistanceToGoal:
	print(loop,name,"Going to the ball")
	goto("ball")
	print(loop,name,"-Shoot 2 kill")
	passBall("yellow_goal")
	loop+=1
print("I probably scored","goal dist: ",player.rel["yellow_goal"][0])
