#! /usr/bin/env python3
import math
import sys
import actionlib
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Float32MultiArray, String, Empty
from ur_msgs.srv import SetIO
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

red_Cordinates = []
trans_yellow = []
trans_red = []
yellow_Cordinates = []
bot_Status = ""
temp_yellow = []

def tf_listener_and_broadcaster(tf_listener):
    # listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    try :
        global trans_yellow
        global trans_red
        global bot_Status
        global yellow_Cordinates
        global red_Cordinates


        if(tf_listener.canTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0)) == True or tf_listener.canTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0)) == True):
            
            if(tf_listener.canTransform("/ebot_base", "/yellow_bell_pepper", tf_listener.getLatestCommonTime("/ebot_base", "/yellow_bell_pepper")) == True):
                trans_yellow = tf_listener.lookupTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0))
                # print("trans_yellow: ",trans_yellow)
                br.sendTransform(trans_yellow[0], trans_yellow[1], rospy.Time.now(), "fruit_yellow", "ebot_base")
                global yellow_Cordinates
                yellow_Cordinates = [round(trans_yellow[0][0], 3), round(trans_yellow[0][1], 3), round(trans_yellow[0][2], 3),0,0,0]
                #yellow_Cordinates = [(trans_yellow[0][0]), (trans_yellow[0][1]), (trans_yellow[0][2]),0,0,0]
                # print(yellow_Cordinates)
            else:
                print("Cannot tranform to fruit_yellow")
            
            if(tf_listener.canTransform("/ebot_base", "/red_bell_pepper", tf_listener.getLatestCommonTime("/ebot_base", "/red_bell_pepper")) == True ):
                trans_red = tf_listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0))
                # print("trans_red: ",trans_red)
                br.sendTransform(trans_red[0], trans_red[1], rospy.Time.now(), "fruit_red", "ebot_base")
                global red_Cordinates
                red_Cordinates = [round(trans_red[0][0], 3), round(trans_red[0][1], 3), round(trans_red[0][2], 3),0,0,0]
                # red_Cordinates = [trans_red[0][0], trans_red[0][1], trans_red[0][2],0,0,0]
                # print(red_Cordinates)
            else:
                print("Cannot tranform to fruit_red")

        else:
            print("No Transform Found")
            a = 1+1
        # print(tf_listener.frameExists("yellow_bell_pepper"))

    except Exception as e:
        print("Exception Occured: ",e)
        pass


class Ur5Moveit:
    def __init__(self):
        self.ack_val = 0                # keeping it as default state in zero, as it acknowledges the topic connection
        self.ack_val_poseJoint = 0      # for pose joint val ack
        self._eef_link = "wrist_3_link" # declaring end effector link for the clas manually
        # self._gripper_group = "gripper"
        # self._planning_group = "arm"
        # self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        # self._gripper = moveit_commander.MoveGroupCommander(self._gripper_group)
        
        # Subscriber nodes
        rospy.Subscriber("/joint_pose_ack", Bool, self.ack_clbck, queue_size=1)
        rospy.Subscriber("/joint_ee_pose", String, self.pose_joint_clbck, queue_size=1)

        # acknowledgment log on terminal
        rospy.logwarn("Make sure that the Manipulation Help node initialization is complete.")
        rospy.loginfo("!!!!!!!!!!!!!!!! UR5_Moveit Initialization complete !!!!!!!!!!!!!!!!!")

        # Publisher variable for joint angles
        self.pub_joint_angles = rospy.Publisher("/set_joint_value_target_wait_topic", 
                                        Float32MultiArray, queue_size=10)

        # Publisher variable for pose values
        self.pub_pose_values = rospy.Publisher("/set_pose_value_target_wait_topic", 
                                        Pose, queue_size=10)
        
        # Publisher variable for requesting pose of ee and joint values
        self.req_pose_joint = rospy.Publisher("/joint_ee_pose_req", 
                                        Empty, queue_size=1)
        rospy.sleep(1)

    def ack_clbck(self, msg):
        '''
        callback function for flag plan retrun value
        '''
        self.flag_plan = msg.data # storing the data part (expected: Bool)

    def pose_joint_clbck(self, msg):
        '''
        callback function for pose ee and joint angles
        '''
        self.pose_and_joint_angles = eval(msg.data) # storing the data part (expected: String) 
                                                    # and evaling for python data types
        self.sample_pose = Pose() # creating Pose instance for converting manually the data

        # parsing position 
        self.sample_pose.position.x        = self.pose_and_joint_angles[0][0]
        self.sample_pose.position.y        = self.pose_and_joint_angles[0][1]
        self.sample_pose.position.z        = self.pose_and_joint_angles[0][2] 

        # parsing orientation
        self.sample_pose.orientation.x     = self.pose_and_joint_angles[0][3]
        self.sample_pose.orientation.y     = self.pose_and_joint_angles[0][4]
        self.sample_pose.orientation.z     = self.pose_and_joint_angles[0][5]
        self.sample_pose.orientation.w     = self.pose_and_joint_angles[0][6]

        self.pose_and_joint_angles[0] = self.sample_pose  # Replacing the first element with actual pose format


    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Setting joint angles
        ---
        Input : Joint angles in Float32MultiArray format
        Output: Flag of execution (0 -> failed, 1 -> success)
        '''
        data_for_float32 = Float32MultiArray()        # Float32Array data type has two sub-data types
        data_for_float32.data = arg_list_joint_angles # storing in data part

        self.pub_joint_angles.publish(data_for_float32) # publishes joint angles on /set_joint_value_target_wait_topic
        rospy.wait_for_message("/joint_pose_ack", Bool, timeout=None) # wait till acknowledgment is recieved

        if (self.flag_plan == True): #Logging flag
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return self.flag_plan # returning the flag of plan on std true-false logic

    def set_pose(self, arg_pose):
        '''
        Setting pose values
        ---
        Input : pose values in Pose format
        Output: Flag of execution (0 -> failed, 1 -> success)
        '''
        self.pub_pose_values.publish(arg_pose) # publishes joint angles on /set_joint_value_target_wait_topic
        rospy.wait_for_message("/joint_pose_ack", Bool, timeout=None) # wait till acknowledgment is recieved

        if (self.flag_plan == True): #Logging flag
            rospy.loginfo(
                '\033[94m' + ">>> set_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_pose() Failed." + '\033[0m')

        return self.flag_plan # returning the flag of plan on std true-false logic

    def print_pose_ee_joint(self): # but outputs both pose and joint angles
                             
        '''
        For accessing pose and joint angles of bot
        ---
        Output: List in format of:
                - index 1: pose list of end-effector   
                - index 2: joint angles list in the angle order 
        '''
        
        self.req_pose_joint.publish() # Publishing as signal to request print the pose and joint angles
        rospy.wait_for_message("/joint_ee_pose", String, timeout=None) # wait till acknowledgment is recieved

        try:
            print("End-effector pose: \n", self.pose_and_joint_angles[0])
            print("Joint angle values: \n", self.pose_and_joint_angles[1])

            pose_values = self.pose_and_joint_angles[0] # Storing the first part of the list
            q_x = pose_values.orientation.x             # Parsing the orientation values
            q_y = pose_values.orientation.y
            q_z = pose_values.orientation.z
            q_w = pose_values.orientation.w
            quaternion_list = [q_x, q_y, q_z, q_w]

            (roll, pitch, yaw) = euler_from_quaternion(quaternion_list) # converting the quaternion to euler

            # Logging the end-effector pose and RPY
            rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                        "x: {}\n".format(pose_values.position.x) +
                        "y: {}\n".format(pose_values.position.y) +
                        "z: {}\n\n".format(pose_values.position.z) +
                        "roll: {}\n".format(roll) +
                        "pitch: {}\n".format(pitch) +
                        "yaw: {}\n".format(yaw) +
                        '\033[0m')

        except Exception as e:
            rospy.logerr("Exception occurred while executing print function: ", str(e))
            self.pose_and_joint_angles = [self.sample_pose, [0,0,0,0,0,0]]  # returning empty values

        return self.pose_and_joint_angles

    def gripper_control(self, logic_level):
        '''
        Controlling gripper to open and close where:
        0 -> Open
        1 -> close
        '''
        rospy.wait_for_service('/ur_hardware_interface/set_io')                # wait for service availabilty
        spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO) # proceed to defining proxy service
        spawn_srv(fun=1, pin=16, state=logic_level) # passing the logic to pin 16 (gripper connection)
        rospy.sleep(4) # wait as it non-blocking call
        rospy.logdebug("Gripper control executed successfully: " + str(logic_level))

    # def open_gripper(self):
    #     self._gripper.set_joint_value_target([0.0])
    #     self._gripper.go()

    # def close_gripper(self):
    #     self._gripper.set_joint_value_target([0.8])
    #     self._gripper.go()

    # Destructor
    def __del__(self):
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

ur5 = Ur5Moveit()
def shaky_shaky_1(self):
        self.set_joint_angles(
            [
                math.radians(19),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(-30),
            ]
        )
        self.set_joint_angles(
            [
                math.radians(19),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(30),
            ]
        )
        self.set_joint_angles(
            [
                math.radians(19),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
            ]
        )

def shaky_shaky_2(self):
        self.set_joint_angles(
            [
                math.radians(-25),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(-30),
            ]
        )
        self.set_joint_angles(
            [
                math.radians(-25),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(30),
            ]
        )
        self.set_joint_angles(
            [
                math.radians(-25),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
            ]
        )

def bot_status_updater(status):
    global bot_Status
    bot_Status = status.data

def get_fruit(fruit_cordinates, drop_angles, fruit_Name, scan, tf_listener) :
    global ur5
    # TODO print("inside get_fruit")

    # temp = []
    # # ur5._group.set_planner_id("KPIECE")
    # # ur5.open_gripper()
    # ur5.gripper_control(0)
    # temp = fruit_cordinates.copy()
    # # temp[0] = temp[0] - 0.04
    # # temp[1] = temp[1] - 0.25

    # temp[1] = temp[1] - 0.55
    # # #temp[5] = yaw
    # # #temp[4] = roll
    # # #temp[3] = pitch
    
    # # temp[3] = -math.pi/4
    # # temp[5] = 0
    # # temp[4] = math.pi

    # # temp[3] = 1.57
    # # temp[4] = 3.14
    # # temp[5] = 1.57

    # # temp[3] = 0
    # # temp[4] = 0
    # # temp[5] = 0
    # position = Point()
    # position.x = temp[0]
    # position.y = temp[1]
    # position.z = temp[2]

    # # roll = temp[4]
    # # pitch = temp[3]
    # # yaw = temp[5]
    # roll = 3.14
    # pitch = 1.57
    # yaw = 3.14
    # orientation = quaternion_from_euler(roll, pitch, yaw)

    # pose_goal = Pose()
    # pose_goal.position = position
    # pose_goal.orientation.x = orientation[0]
    # pose_goal.orientation.y = orientation[1]
    # pose_goal.orientation.z = orientation[2]
    # pose_goal.orientation.w = orientation[3]


    # # print(args)
    # # ur5.set_pose(pose_goal)
    # # ur5.print_pose_ee_joint()
    # print("Part 1 Completed")
    # x, y, z, w = False, False, False, False
    # while x == False and y == False and z == False:
    #     ur5.set_pose(pose_goal)
    #     currentPose = ur5.print_pose_ee_joint()
    #     x = True if(round(currentPose[0].position.x, 2)==round(temp[0], 2)) else False
    #     y = True if(round(currentPose[0].position.y, 2)==round(temp[1], 2)) else False
    #     z = True if(round(currentPose[0].position.z, 2)==round(temp[2], 2)) else False 

    # # temp[1] = temp[1] + 0.15 #finally equals to temp - 0.25
    # # position.y = temp[1]
    # # pose_goal.position = position

    # # x, y, z, w = False, False, False, False
    # # while x == False and y == False and z == False:
    # #     ur5.set_pose(pose_goal)
    # #     currentPose = ur5.print_pose_ee_joint()
    # #     x = True if(round(currentPose[0].position.x, 2)==round(temp[0], 2)) else False
    # #     y = True if(round(currentPose[0].position.y, 2)==round(temp[1], 2)) else False
    # #     z = True if(round(currentPose[0].position.z, 2)==round(temp[2], 2)) else False 

    # # temp[3] += math.pi/4
    # # pitch = temp[3]
    # # orientation = quaternion_from_euler(roll, pitch, yaw)
    # # pose_goal.orientation.x = orientation[0]
    # # pose_goal.orientation.y = orientation[1]
    # # pose_goal.orientation.z = orientation[2]
    # # pose_goal.orientation.w = orientation[3]

    # # args.position.x = temp[0]
    # # args.position.y = temp[1]
    # # args.position.z = temp[2]
    # # args.orientation.x = temp[3]
    # # args.orientation.y = temp[4]
    # # args.orientation.z = temp[5]


    # # ur5.set_pose(pose_goal)
    # print("Part 2 Completed")
    # # x, y, z, w = False, False, False, False
    # # while x == False and y == False and z == False and w == False:
    # #     ur5.set_pose(temp)
    # #     currentPose = ur5._group.get_current_pose().pose
    # #     x = True if(round(currentPose.position.x, 2)==round(temp[0], 2)) else False
    # #     y = True if(round(currentPose.position.y, 2)==round(temp[1], 2)) else False
    # #     z = True if(round(currentPose.position.z, 2)==round(temp[3], 2)) else False 
    # #     w = True if(round(currentPose.orientation.x, 2)==round(temp[3], 2)) else False 
    # #     # print("Set Joint Angles",ur5.set_joint_angles(list_joint_values))

    # # ur5.close_gripper()
    # ur5.gripper_control(1)
    # print("Plucked "+fruit_Name)
    ur5.set_joint_angles(drop_angles)
    # ur5.open_gripper()
    ur5.gripper_control(0)
    print("Dropped "+fruit_Name+" in basket")
    # ur5.set_joint_angles(scan)

def main():
    global ur5
    global yellow_Cordinates
    global red_Cordinates
    global temp_yellow
    
    rospy.init_node("maniStack", anonymous=True)
    lst_joint_angles_picking_pose = [
        # math.radians(60),
        # math.radians(-160),
        # math.radians(100),
        # math.radians(-120),
        # math.radians(-60),
        # math.radians(90)

        math.radians(30),
        math.radians(-180),
        math.radians(120),
        math.radians(-120),
        math.radians(-30),
        math.radians(90)

        ]
    
    lst_joint_angles_scan = [
        math.radians(80),
        math.radians(-180),
        math.radians(90),
        math.radians(-90),
        math.radians(-80),
        math.radians(90)

    ]

    red_basket = [
        # math.radians(-30),
        # math.radians(-120),
        # math.radians(100),
        # math.radians(50),
        # math.radians(90),
        # math.radians(-90),
        math.radians(-30),
        math.radians(-141),
        math.radians(131),
        math.radians(39),
        math.radians(90),
        math.radians(-90),
    ]

    yellow_basket = [
        math.radians(10),
        math.radians(-120),
        math.radians(100),
        math.radians(50),
        math.radians(90),
        math.radians(-90),
    ]
    rate = rospy.Rate(5)
    tf_listener = tf.TransformListener()
    rospy.Subscriber("bot_status",String, bot_status_updater)
    bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
    print("Started")
    ur5.set_joint_angles(lst_joint_angles_scan)
    rospy.sleep(7)
    tf_listener_and_broadcaster(tf_listener)
    while not rospy.is_shutdown():
        ur5.set_joint_angles(red_basket)
        ur5.gripper_control(0)
        # tf_listener_and_broadcaster(tf_listener)
        # if((len(yellow_Cordinates) !=0 or len(red_Cordinates) !=0) and bot_Status == "stop"):
        #     try:
        #         # if(len(yellow_Cordinates) !=0):
        #         #     ur5.set_joint_angles(lst_joint_angles_picking_pose)
        #         #     # TODO print("                    tring [YELLOW] get_fruit") 
        #         #     print("yellow cordinates: ", yellow_Cordinates)
        #         #     get_fruit(yellow_Cordinates, red_basket,"yellow_bell_pepper", lst_joint_angles_scan, tf_listener)
                
        #         if(len(red_Cordinates) !=0):
        #             ur5.set_joint_angles(lst_joint_angles_picking_pose)
        #             # TODO print("                    tring [RED] get_fruit")
        #             print("red cordinates: ", red_Cordinates)
        #             get_fruit(red_Cordinates, yellow_basket, "red_bell_pepper", lst_joint_angles_scan, tf_listener)

        #     except Exception as e:
        #         print("Exception in main: ", e)

        #     finally:
        #         yellow_Cordinates = []
        #         red_Cordinates = []
        #         bot_pub.publish("ready")
        #         print(bot_Status)
        rate.sleep()
    del ur5

if __name__ == "__main__":
    main()
    a = 2+3
