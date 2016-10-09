#!/usr/bin/env python
#COOPERATIVE PLAYERS
#SETUP
import rospy
from std_msgs.msg import String
import math
import random
#module setup
rospy.init_node("player_node",anonymous=True)
rate=rospy.Rate(10)
name= rospy.get_param("~p1_name")
passTo= rospy.get_param("~p2_name")
#player setup
from soccer.controller import RobotController
teamChannel="/teamspeak"
player = RobotController(name)
#sleep for 1s (controller sync)
rospy.sleep(1)
leaderExist=0
#state 0: leader go to ball and passes / 1: other player shoot to goal
global leader
leader=name

#Tuning parameters
turningSpeed = 500
facingSpeed = 200
approachSpeed = 1000
sleepTime = .1
turnThreshold = .1
approachThreshold = .2
#turning around ball
ratio = 5.5
v = 500
omega = -v/ratio

def goto(player,location):
    turnToward(player,location,turnThreshold)
    approach(player,location,approachThreshold)
    return
def turnToward(player,location,threshold):
    relOri = -player.rel[location][1]
    if relOri > 0:
        speed = -turningSpeed
    else:
        speed = turningSpeed
    while abs(-player.rel[location][1])>threshold:
        player.move(0,speed)
        rospy.sleep(sleepTime)
    player.move(0,0)
    return
def face(player,location):
    if -player.rel[location][1] > 0:
        speed = -facingSpeed
    else:
        speed = facingSpeed
	while abs(-player.rel[location][1])>.1:
		player.move(0,speed)
		rospy.sleep(sleepTime)
	player.move(0,0)
	return
def approach(player,location,threshold):    
    while player.rel[location][0]>abs(threshold):
        #watchdog if player is miss aligned
        if abs(player.rel[location][1])>math.pi/2:
            print("wrong orientation detected")
            break
        player.move(approachSpeed,0)
        rospy.sleep(sleepTime) 
    player.move(0,0)
    dist2location = player.rel[location][0]
    return dist2location
def passBall(player,receiver):
    #suppose that we are close to ball
    #Turn fron pi/2 
    while -player.rel["ball"][1] < math.pi/2:
        player.move(0,100)
        rospy.sleep(sleepTime)
    #Orbit around
    while abs(player.rel["ball"][1]-player.rel[receiver][1])>.2:
		diff=player.rel["ball"][1]-player.rel[receiver][1]
		#rospy.loginfo("orbital end=%s",diff)
		player.move(v,omega)
		rospy.sleep(sleepTime)
    player.move(0,0)
    turnToward(player,"ball",turnThreshold)
    #shoot
    player.move(1000,0)
    rospy.sleep(1.7)
    player.move(0,0)
    rospy.sleep(4)#sleeping while the ball is having inertia
    return
#### COMMUNICATION ####
def defineLeader(): #broadcast on channel that I lead
	global leader
	pub = rospy.Publisher(teamChannel, String, queue_size=2)  
	rospy.loginfo("leader ==> %s",leader)
	rospy.loginfo("name ==> %s",name)
	while not rospy.is_shutdown():
		if leader==name:
			pub.publish(leader)
		rospy.sleep(sleepTime)
	return
def listenTeam():
    rospy.Subscriber(teamChannel,String,behave)
    return
def behave(msg):
	global leader
	rospy.loginfo("team: %s",msg.data)
	if leader==name:
		goto(player,"ball")
		passBall(player,passTo)
		leader=passTo #pass is done, now turn of other
		defineLeader()
	else:#I dont lead i say ok and i go forward
		turnToward(player,"yellow_goal",.5)
		leader=msg.data
		player.move(500,0)
	return

#Communication
#all player ready to listen leader
rospy.sleep(sleepTime)
player.move(0,0)
listenTeam()
if name=="blue1":
	defineLeader()
rate.sleep()
print("reached OUT OF LEADER")
