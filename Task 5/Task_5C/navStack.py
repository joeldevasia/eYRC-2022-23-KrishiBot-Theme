#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
from std_msgs.msg import String
from simple_pid import PID

kp = 5
ki = 0
kd = 3

pidVal = 0
pid = PID(kp, ki, kd, setpoint=0.7)
pid.sample_time = 0.01
pid.output_limits = (-0.2, 0.2)
pid.auto_mode = True
trans_red = []
trans_yellow = []
lscan = LaserScan()
bot_Status = ""
task_Status = "1"
rangeMax = lscan.range_max
velocity = Twist()
# regions_ = {
#         'right':  'inf',
#         'fright': 'inf',
#         'front':  'inf',
#         'fleft':  'inf',
#         'left':   'inf',
#     }
regions_ = {
        'exactfront' : 'inf',
        'right' : 'inf',
        'left' : 'inf',
        'mid-left': 'inf'
    }

# def laser_callback(msg):
#     global regions_
#     regions_ = {
#         'exactfront' : msg.ranges[359],
#         'right' : msg.ranges[0],
#         'left' : msg.ranges[719],
#         'mid-left': msg.ranges[540]
#     }
#     # print("Controller message pushed at {}".format(rospy.get_time()))
#     print("left: ",regions_['left'], "exactfront: ",regions_['exactfront'], "right: ",regions_['right'])

def laser_callback(msg):
    global regions_
    laser_data = list(msg.ranges)
    for i in range(len(msg.ranges)):
        if(laser_data[i] <= 0.1):
            laser_data[i] = 100.0
    regions_ = {
        'right':  min(laser_data[0], 8.0),
        'fright': min(laser_data[44], 10),
        'front':  min(min(laser_data[221:309]), 8.0),
        'exactfront':  laser_data[265],
        'fleft':  min(laser_data[134], 10),
        'left':   min(laser_data[530], 8.0),
    }
    # print("left: ",regions_['left'], "exactfront: ",regions_['exactfront'], "right: ",regions_['right'])
    
#     print([i for i, e in enumerate(laser_data) if e == float('inf')])

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)

def straighten():
    n1 = float(regions_['left'])
    rospy.sleep(0.5)
    n2 = float(regions_['left'])
    if((n2 - n1) > 0.05):
        smallLeft()
        pub.publish(velocity)
        rospy.sleep(0.5)

def straightenByRight():
    n1 = float(regions_['right'])
    rospy.sleep(0.5)
    n2 = float(regions_['right'])
    if((n1 - n2) > 0.05):
        smallLeft()
        pub.publish(velocity)
        rospy.sleep(0.5)

def newStraighten():
        if(float(regions_['left']) > 0.75):
            while(float(regions_['left']) > 0.75) and (float(regions_['exactfront']) > 2):
                smallLeft()
                pub.publish(velocity)
                if(float(regions_['left']) > 3):
                    break
        elif(float(regions_['left']) < 0.55):
            while(float(regions_['left']) < 0.55) and (float(regions_['exactfront']) > 2):
                smallRight()
                pub.publish(velocity)

def newStraightenReverse():
        if(float(regions_['left']) > 0.8):
            while(float(regions_['left']) > 0.8) and (float(regions_['exactfront']) > 2):
                smallRight() 
                pub.publish(velocity)
                if(float(regions_['left']) > 3):
                    break
        elif(float(regions_['left']) < 0.8):
            while(float(regions_['left']) < 0.8) and (float(regions_['exactfront']) > 2):
                smallLeft()
                pub.publish(velocity)

def straightenUsingPID(LorR, Lspeed, MaxDistance):
    global pidVal
    velocity.linear.x = Lspeed
    if float(regions_["left"]) <= MaxDistance:
      velocity.angular.z = pid(min(round(float(regions_[LorR]), 2), 1)) * (-1)
    else:
        velocity.angular.z = 0
    # print(pid(min(float(regions_["left"]), 0.6)))
    print(LorR,": ", regions_[LorR], "  PID: ", pidVal, "ExactFront: ", regions_["exactfront"])
    pidVal = velocity.angular.z
    pub.publish(velocity)

def moveStraight():
    velocity.linear.x = 1
    velocity.angular.z = 0
    pub.publish(velocity)
  
def slowstart():
    velocity.linear.x = 0.1
    velocity.angular.z = 0
    pub.publish(velocity)
    # rospy.sleep(0.25)   

def slowstartReverse():
    velocity.linear.x = -0.5
    velocity.angular.z = 0
    pub.publish(velocity)
    # rospy.sleep(0.25)   
  
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
    # rospy.sleep(0.5)
    
def smallLeft():
    velocity.angular.z = 0.05
    pub.publish(velocity)

def smallRight():
    velocity.angular.z = -0.05
    pub.publish(velocity)

def bot_status_updater(status):
    global bot_Status
    bot_Status = status.data
    # if bot_Status == "ready":
    #     slowstart()
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
    rospy.init_node("ebot_controller")
    tf_listener = tf.TransformListener()

    bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
    rospy.Subscriber("bot_status", String, bot_status_updater)

    task_pub = rospy.Publisher("task_status", String, queue_size=10)
    rospy.Subscriber("task_status", String, task_status_updater)

    info_pub = rospy.Publisher("taskInfo", String, queue_size=1)

    # rospy.sleep(2)

    print("Control Loop Started")
    # rate = rospy.Rate(1)
    print("Mission Started!")
    info_pub.publish("Mission Started!")
    rospy.sleep(5)
    print("Starting Slow Start")
    slowstart()
    rospy.sleep(7)
    print("Finished SlowStart")
    while rospy.is_shutdown() == False:
        task_pub.publish("1")
        while float(regions_["exactfront"]) > 5:
            while float(regions_["exactfront"]) > 5 and bot_Status != "stop":
                # slowstart()
                print("inside:",bot_Status)
                straightenUsingPID("left", 0.1, 1)
                # print("Inside 1st while")
            # slowstart()
            # stop()
            # print("outside:",bot_Status)
        stop()
        print("outside:",bot_Status)            

        # bot_pub.publish("stop")
        # print("Completed Successfully")
        # stop()
        # break
        # task_pub.publish("2")
        # rospy.sleep(20)
        # Reverse:
        pid.setpoint = 0.8
        while(float(regions_['exactfront']) != float('inf')):
            straightenUsingPID("left", 0.1, float('inf'))
            print("Moving Left")
        # stop()

        pid.setpoint = 0.8

        while(float(regions_['exactfront']) > 1.2):
            while float(regions_['exactfront']) > 1.2 and bot_Status != "stop":
                straightenUsingPID("left", 0.1, 1)
                # print("Inside 2nd while")
            # slowstart()
            # print("2nd while")
        # stop()
        while float(regions_['exactfront']) < 2:
            straightenUsingPID("left", 0.1, float('inf'))
            print("Turning Left")
        # pid.setpoint = 0.8
        # stop()
        slowstart()
        while float(regions_['exactfront']) > 3.2:
            while float(regions_['exactfront']) > 3.2 and bot_Status != "stop":
                # slowstart()
                straightenUsingPID("right", 1)
                print("Inside 3rd while")
            # slowstart()
            # print("3rd while")
        # slowstart
        # stop()
        while float(regions_['exactfront']) != float('inf'):
            straightenUsingPID("left", 0.1, float('inf'))
            print("Turning Left")

        while(float(regions_['exactfront']) > 2):
            while float(regions_['exactfront']) > 2 and bot_Status != "stop":
                straightenUsingPID("left", 0.1, 1)

        # stop()

        info_pub.publish("Mission Accomplished!")
        print("Mission Accomplished!")

        break
if __name__ == '__main__':
    try:
        control_loop()
        a = 2 + 2
    except rospy.ROSInterruptException:
        pass