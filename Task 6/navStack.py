#!/usr/bin/env python3

""" 
* Team Id : 2546
* Author List : Joel Devasia
* Filename: navStack.py
* Theme: Krishi Bot (KB)
* Functions: [ bot_status_updater, laser_callback, newStraighten, straightenUsingPID, slowstart, turnLeft, stop,  smallLeft, smallRight, control_loop ]
"""

# Initialize Global variables

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

regions_ = {"exactfront": "inf", "right": "inf",
            "left": "inf", "mid-left": "inf"}


def laser_callback(msg):
    """
    * Function Name: laser_callback
    * Input: msg
    * Output: None
    * Logic: This function is used to get the laser scan data from the robot
    *
    * Example Call: laser_callback(msg)
    """
    global regions_
    laser_data = list(msg.ranges)
    for i in range(len(msg.ranges)):
        if laser_data[i] <= 0.1:
            laser_data[i] = 100.0
    regions_ = {
        "right": min(min(laser_data[0:73]), 8.0),
        "fright": min(laser_data[44], 10),
        "front": min(min(laser_data[221:309]), 8.0),
        "exactfront": laser_data[265],
        "fleft": min(laser_data[134], 10),
        "left": min(min(laser_data[426:531]), 8.0),
    }


pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.Subscriber("/ebot/laser/scan", LaserScan, laser_callback)


def newStraighten():
    """
    * Function Name: newStraighten
    * Input: None
    * Output: None
    * Logic:  This function is used to straighten the robot when it is moving in a straight line
    *
    * Example Call: newStraighten()
    """
    if float(regions_["left"]) > 0.75:
        while (float(regions_["left"]) > 0.75) and (float(regions_["exactfront"]) > 2):
            smallLeft()
            pub.publish(velocity)
            if float(regions_["left"]) > 3:
                break
    elif float(regions_["left"]) < 0.55:
        while (float(regions_["left"]) < 0.55) and (float(regions_["exactfront"]) > 2):
            smallRight()
            pub.publish(velocity)


def straightenUsingPID(LorR, Lspeed, MaxDistance):
    """
    * Function Name: straightenUsingPID
    * Input: LorR, Lspeed, MaxDistance
    * Output: None
    * Logic: This function is used to straighten the robot when it is moving in a straight line using PID
    *
    * Example Call: straightenUsingPID(LorR, Lspeed, MaxDistance)
    """
    global pidVal
    velocity.linear.x = Lspeed
    velocity.angular.z = pid(min(round(float(regions_[LorR]), 2), 1)) * (-1)
    # print(pid(min(float(regions_["left"]), 0.6)))
    print(
        LorR,
        ": ",
        regions_[LorR],
        "  PID: ",
        pidVal,
        "ExactFront: ",
        regions_["exactfront"],
    )
    pidVal = velocity.angular.z
    pub.publish(velocity)


def slowstart():
    """
    * Function Name: slowstart
    * Input: None
    * Output: None
    * Logic: This function is used to start the robot slowly
    *
    * Example Call: slowstart()
    """

    velocity.linear.x = 0.1
    velocity.angular.z = 0
    pub.publish(velocity)
    # rospy.sleep(0.25)


def turnLeft():
    """
    * Function Name: turnLeft
    * Input: None
    * Output: None
    * Logic: This function is used to turn the robot left
    *
    * Example Call: turnLeft()
    """
    velocity.linear.x = 0
    velocity.angular.z = 1.5
    pub.publish(velocity)


def stop():
    """
    * Function Name: stop
    * Input: None
    * Output: None
    * Logic: This function is used to stop the robot
    *
    * Example Call: stop()
    """
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub.publish(velocity)
    print("inside stop")
    # print(velocity)
    # rospy.sleep(0.5)


def smallLeft():
    """
    * Function Name: smallLeft
    * Input: None
    * Output: None
    * Logic: This function is used to turn the robot left by a small angle
    *
    * Example Call: smallLeft()
    """
    velocity.angular.z = 0.05
    pub.publish(velocity)


def smallRight():
    """
    * Function Name: smallRight
    * Input: None
    * Output: None
    * Logic: This function is used to turn the robot right by a small angle
    *
    * Example Call: smallRight()
    """
    velocity.angular.z = -0.05
    pub.publish(velocity)


def bot_status_updater(status):
    """
    * Function Name: bot_status_updater
    * Input: status
    * Output: None
    * Logic: This function is used to update the status of the robot
    *
    * Example Call: bot_status_updater(status)
    """
    global bot_Status
    bot_Status = status.data
    # if bot_Status == "ready":
    #     slowstart()
    if bot_Status == "stop":
        print("Bot Stopped")
        stop()


##################################################################


def control_loop():
    """
    * Function Name: control_loop
    * Input: None
    * Output: None
    * Logic: This function is used to control the robot
    *
    * Example Call: control_loop()
    """
    global bot_Status
    global task_Status
    rospy.init_node("ebot_controller")
    tf_listener = tf.TransformListener()

    bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
    rospy.Subscriber("bot_status", String, bot_status_updater)

    info_pub = rospy.Publisher("taskInfo", String, queue_size=1)

    rospy.sleep(2)

    print("Control Loop Started")
    # rate = rospy.Rate(1)
    print("Mission Started!")
    info_pub.publish("Mission Started!")
    print("Starting Slow Start")
    slowstart()
    rospy.sleep(10)
    print("Finished SlowStart")
    while rospy.is_shutdown() == False:
        while float(regions_["exactfront"]) > 2:
            while float(regions_["exactfront"]) > 2:
                slowstart()
                print("inside:", bot_Status)
                newStraighten()
            print("outside:", bot_Status)

        pid.setpoint = 0.7
        while float(regions_["exactfront"]) != float("inf"):
            straightenUsingPID("left", 0.1, float("inf"))
            print("Moving Left")

        pid.setpoint = 0.8
        while float(regions_["exactfront"]) > 1.5:
            while float(regions_["exactfront"]) > 1.5:
                slowstart()
                print("inside:", bot_Status)
                newStraighten()

        pid.setpoint = 0.7
        while float(regions_["exactfront"]) < 3.5:
            straightenUsingPID("left", 0.1, float("inf"))
            print("Turning Left")

        pid.setpoint = 0.6
        while float(regions_["exactfront"]) > 3:
            while float(regions_["exactfront"]) > 3:
                slowstart()
                print("inside:", bot_Status)
                newStraighten()

        while float(regions_["exactfront"]) != float("inf"):
            straightenUsingPID("left", 0.1, float("inf"))
            print("Turning Left")

        pid.setpoint = 0.7
        while float(regions_["exactfront"]) > 2:
            while float(regions_["exactfront"]) > 2:
                slowstart()
                print("inside:", bot_Status)
                newStraighten()

        stop()
        info_pub.publish("Mission Accomplished!")
        print("Mission Accomplished!")

        break


if __name__ == "__main__":
    try:
        control_loop()
        a = 2 + 2
    except rospy.ROSInterruptException:
        pass
