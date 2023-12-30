#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
from std_msgs.msg import String
trans_red = []
trans_yellow = []
lscan = LaserScan()
bot_Status = ""
task_Status = "1"
rangeMax = lscan.range_max
velocity = Twist()
regions = {
        'exactFront' : 'inf',
        'right' : 'inf',
        'left' : 'inf'
    }

def tf_listener_and_broadcaster():
    listener = tf.TransformListener()
    # br = tf.TransformBroadcaster()
    try : 
        global trans_yellow
        # listener.waitForTransform("/yellow_bell_pepper", "/ebot_base", rospy.Time(0), rospy.Duration(0.5))
        if(listener.canTransform("/yellow_bell_pepper", "/ebot_base", rospy.Time(0))):
            trans_yellow = listener.lookupTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0))
            # TODO
            # print("trans_yellow: ", trans_yellow)
        # print("lookUp Yellow")

        # global trans_red
        # listener.waitForTransform("/red_bell_pepper", "/ebot_base", rospy.Time(0), rospy.Duration(0.5))
        # print("First of All red")
        # if(listener.canTransform("/red_bell_pepper", "/ebot_base", rospy.Time(0))):
        #     print("Fuckyou",listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0)))
            # trans_red = listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0))
        # print("lookUp Red")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print("Exception Occured: ",e)
        pass
    return 

def laser_callback(msg):
    global regions
    regions = {
        'exactFront' : msg.ranges[359],
        'right' : msg.ranges[0],
        'left' : msg.ranges[719],
        'mid-left': msg.ranges[540]
    }
    # print("Controller message pushed at {}".format(rospy.get_time()))
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
        elif(float(regions['left']) < 0.6):
            while(float(regions['left']) < 0.6) and (float(regions['exactFront']) > 2):
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
    velocity.angular.z = 1
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

def bot_status_updater(status):
    global bot_Status
    bot_Status = status.data
    if bot_Status == "ready":
        slowstart()
    if(bot_Status == "stop"):
        print("Bot Stopped")
        stop()

def task_status_updater(status):
    global task_Status
    task_Status = status.data



##################################################################

def control_loop():
    global bot_Status
    global task_Status
    rospy.init_node('ebot_controller') 
    tf_listener = tf.TransformListener()


    bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
    rospy.Subscriber("bot_status", String, bot_status_updater)

    task_pub = rospy.Publisher("task_status", String, queue_size=1)
    rospy.Subscriber("task_status", String, task_status_updater)

    info_pub = rospy.Publisher("taskInfo", String, queue_size=1)


    print("Control Loop Started")
    rate = rospy.Rate(1)
    # rospy.sleep(1)tf_listener.frameExists("yellow_bell_pepper")
    print("Mission Started!")
    info_pub.publish("Mission Started!")
    slowstart()
    while (rospy.is_shutdown() == False):
        # while(float(regions['exactFront']) > 0.25 and tf_listener.frameExists("yellow_bell_pepper") == False) or bot_Status == "ready":
        while(task_Status == "1" and float(regions['exactFront']) > 1.2):
            while float(regions['exactFront']) > 1.2 and bot_Status != "stop" :
                # moveStraight()
                slowstart()
                # print(bool(trans_red), bool(trans_yellow))
                # if(bool(trans_red) or bool(trans_yellow)):
                #     break

        while(float(regions['exactFront']) != float('inf')):
            moveLeft()
        stop()
        task_Status = "2"
        task_pub.publish(task_Status)

        while(task_Status == "2" and float(regions['exactFront']) > 1.2):
            while float(regions['exactFront']) > 1.2 and bot_Status != "stop":
                slowstart()
                newStraighten()

        while float(regions['exactFront']) <5:
            turnLeft()

        task_Status = "3"
        task_pub.publish(task_Status)

        slowstart()
        while task_Status == "3" and float(regions['exactFront']) > 3.2:
            while float(regions['exactFront']) > 3.2 and bot_Status != "stop":
                slowstart()
                straightenByRight()
        slowstart

        while float(regions['exactFront']) != float('inf'):
            moveLeft()

        task_Status = "4"
        task_pub.publish(task_Status)

        #and float(regions['exactFront']) > 1.3
        while task_Status == "4" and float(regions['exactFront']) > 5:
            while float(regions['exactFront']) > 1 and bot_Status != "stop":
                slowstart()
                newStraighten()

        stop()
        
        # if(float(regions['exactFront']) > 1.5):
        #     while(float(regions['exactFront']) != float('inf')): 
        #         moveLeft()

        
        
        info_pub.publish("Mission Accomplished!")
        print("Mission Accomplished!")

        break

        # while(float(regions['exactFront']) > 1.3):
        #     moveStraight()
        #     newStraighten()
        # slowstart()
        # while float(regions['exactFront']) <5:
        #     turnLeft()
        # slowstart()
        # while float(regions['exactFront']) > 3.5:
        #     moveStraight()
        #     straightenByRight()
        # slowstart()
        # while float(regions['exactFront']) != float('inf'):
        #     moveLeft()
        # while(float(regions['exactFront']) > 1.3) and float(regions['left']) < 3:
        #     moveStraight()
        #     newStraighten()
        # if(bot_Status == "stop"):
        #     print("Bot Stopped")
        #     stop()
            # pub.publish("stopped")
        # print(tf_listener.allFramesAsString())
if __name__ == '__main__':
    try:
        control_loop()
        a = 2 + 2
    except rospy.ROSInterruptException:
        pass