#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 2546 ]
# Author List:		[ Vyomkesh Mulam, Joel Devasia, Priya Jain, Tanmay Mohod  ]
# Filename:			task_0.py
# Functions:		
# 					[ findEuclideanDistance, callback, delay, moveDown, rotate, move_semicircle, main  ]
# Nodes:		    
#					Publishing: turtle1/cmd_vel
#					Subscribed: turtle1/pose

####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
##############################################################

################# ADD GLOBAL VARIABLES HERE #################

pi = math.pi
vel_msg = Twist()
pose = Pose()
currentDistance = 0
currentTheta = 0.0
currentX = 0.0
currentY = 0.0
initialX = 5.544444561004639
initialY = 5.544444561004639

##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
def findEuclideanDistance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


##############################################################

def callback(data):
	global currentTheta
	currentTheta = data.theta
	global currentX
	currentX = data.x
	global currentY
	currentY = data.y
	#maxY = pose.y
	global currentDistance
	currentDistance =  findEuclideanDistance(initialX, initialY, currentX, currentY)
    # if(currentDistance > currentMaxDistance):
    #     currentMaxDistance = currentDistance

	rospy.loginfo("%s %s %s %s", currentX, currentY, currentDistance, currentTheta)

rospy.init_node("eYRC Task 0 Turtlesim", anonymous=True)
velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1000)
rospy.Subscriber("/turtle1/pose", Pose, callback)

vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

def delay():
	#rate.sleep()
	rospy.sleep(1)
	return

def moveDown():
    vel_msg.linear.x = abs(initialY - currentY)
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    delay()
    return

def rotate():
    vel_msg.linear.x = 0
    vel_msg.angular.z = round(pi, 3)/2
    rospy.loginfo("in rotate")
    while True:
        velocity_publisher.publish(vel_msg)
        delay()
        if(pose.theta > -round(pi, 3)/2):
            break
    return

def move_semicircle():
	vel_msg.linear.x = round(pi, 3)
	vel_msg.angular.z = round(pi, 3)
	rospy.loginfo("in main")
	while currentDistance < 1.998789294040773:
		velocity_publisher.publish(vel_msg)
		delay()
	rospy.loginfo("exiting while")
	return

def main():
		rospy.loginfo("in main")
		move_semicircle()
		rotate()
		moveDown()
		
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########

if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
        
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
