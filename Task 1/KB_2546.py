#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
lscan = LaserScan()
rangeMax = lscan.range_max
velocity = Twist()
regions = {
        'exactFront' : 'inf',
        'right' : 'inf',
        'left' : 'inf'
    }
def laser_callback(msg):
    global regions
    regions = {
        'exactFront' : msg.ranges[359],
        'right' : msg.ranges[0],
        'left' : msg.ranges[719]
    }
    print("Controller message pushed at {}".format(rospy.get_time()))
    #print("left: ",regions['left'], "exactFront: ",regions['exactFront'])

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)

def straighten():
    n1 = float(regions['left'])
    rospy.sleep(0.5)
    n2 = float(regions['left'])
    if((n2 - n1) > 0.05):
        smallLeft()
        pub.publish(velocity)
        rospy.sleep(0.5)

def straightenByRight():
    n1 = float(regions['right'])
    rospy.sleep(0.5)
    n2 = float(regions['right'])
    if((n1 - n2) > 0.05):
        smallLeft()
        pub.publish(velocity)
        rospy.sleep(0.5)

def newStraighten():
        if(float(regions['left']) > 0.7):
            while(float(regions['left']) > 0.7) and (float(regions['exactFront']) > 2):
                smallLeft()
                pub.publish(velocity)
                if(float(regions['left']) > 3):
                    break
        elif(float(regions['left']) < 0.5):
            while(float(regions['left']) < 0.5) and (float(regions['exactFront']) > 2):
                smallRight()
                pub.publish(velocity)

def moveStraight():
    velocity.linear.x = 1
    velocity.angular.z = 0
    pub.publish(velocity)
  
def slowstart():
    velocity.linear.x = 0.5
    velocity.angular.z = 0
    pub.publish(velocity)
    rospy.sleep(0.25)    
  
def moveLeft():
    velocity.linear.x = 0.5
    velocity.angular.z = 1.5
    pub.publish(velocity)

def turnLeft():
    velocity.linear.x = 0
    velocity.angular.z = 1.5
    pub.publish(velocity)

def stop():
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub.publish(velocity)
    rospy.sleep(0.5)
    
def smallLeft():
    velocity.angular.z = 0.2
    pub.publish(velocity)

def smallRight():
    velocity.angular.z = -0.2
    pub.publish(velocity)
    
def control_loop():
    rospy.init_node('ebot_controller') 
    rospy.sleep(1)
    while (rospy.is_shutdown() == False):
        slowstart()
        while(float(regions['exactFront']) > 1.2):
            moveStraight()            
        while(float(regions['exactFront']) != float('inf')): 
            moveLeft()
        slowstart()
        while(float(regions['exactFront']) > 1.3):
            moveStraight()
            newStraighten()
        slowstart()
        while float(regions['exactFront']) <5:
            turnLeft()
        slowstart()
        while float(regions['exactFront']) > 3.5:
            moveStraight()
            straightenByRight()
        slowstart()
        while float(regions['exactFront']) != float('inf'):
            moveLeft()
        while(float(regions['exactFront']) > 1.3) and float(regions['left']) < 3:
            moveStraight()
            newStraighten()
        stop()
        break
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
