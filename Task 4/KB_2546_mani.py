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
        # listener.waitForTransform("/yellow_bell_pepper", "/ebot_base", rospy.Time(0), rospy.Duration(10))
        # trans_yellow = listener.lookupTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0))
        # print("trans_yellow: ",trans_yellow)
        # br.sendTransform(trans_yellow[0], trans_yellow[1], rospy.Time.now(), "fruit_yellow", "ebot_base")
        # global yellow_Cordinates
        # yellow_Cordinates = [round(trans_yellow[0][0], 3), round(trans_yellow[0][1], 3), round(trans_yellow[0][2], 3),0,0,0]
        # print(yellow_Cordinates)

        # if(tf_listener.frameExists("yellow_bell_pepper") == True or tf_listener.frameExists("red_bell_pepper") == True):
        #     if(tf_listener.frameExists("red_bell_pepper") == True):
        #         trans_red = tf_listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0))
        #         # print("trans_red: ",trans_red)
        #         br.sendTransform(trans_red[0], trans_red[1], rospy.Time.now(), "fruit_red", "ebot_base")
        #         global red_Cordinates
        #         # red_Cordinates = [round(trans_red[0][0], 3), round(trans_red[0][1], 3), round(trans_red[0][2], 3),0,0,0]
        #         red_Cordinates = [trans_red[0][0], trans_red[0][1], trans_red[0][2],0,0,0]
        #         # print(red_Cordinates)

        #     elif(tf_listener.frameExists("yellow_bell_pepper") == True):
        #         trans_yellow = tf_listener.lookupTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0))
        #         # print("trans_yellow: ",trans_yellow)
        #         br.sendTransform(trans_yellow[0], trans_yellow[1], rospy.Time.now(), "fruit_yellow", "ebot_base")
        #         global yellow_Cordinates
        #         yellow_Cordinates = [round(trans_yellow[0][0], 3), round(trans_yellow[0][1], 3), round(trans_yellow[0][2], 3),0,0,0]
        #         # print(yellow_Cordinates)




        if(tf_listener.canTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0)) == True or tf_listener.canTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0)) == True):
            # if(tf_listener.canTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0)) == True ):
            #     trans_red = tf_listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0))
            #     # print("trans_red: ",trans_red)
            #     br.sendTransform(trans_red[0], trans_red[1], rospy.Time.now(), "fruit_red", "ebot_base")
            #     global red_Cordinates
            #     # red_Cordinates = [round(trans_red[0][0], 3), round(trans_red[0][1], 3), round(trans_red[0][2], 3),0,0,0]
            #     # red_Cordinates = [trans_red[0][0], trans_red[0][1], trans_red[0][2],0,0,0]
            #     # print(red_Cordinates)

            if(tf_listener.canTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0)) == True):
                trans_yellow = tf_listener.lookupTransform("/ebot_base", "/yellow_bell_pepper", rospy.Time(0))
                # print("trans_yellow: ",trans_yellow)
                br.sendTransform(trans_yellow[0], trans_yellow[1], rospy.Time.now(), "fruit_yellow", "ebot_base")
                global yellow_Cordinates
                yellow_Cordinates = [round(trans_yellow[0][0], 3), round(trans_yellow[0][1], 3), round(trans_yellow[0][2], 3),0,0,0]
                #yellow_Cordinates = [(trans_yellow[0][0]), (trans_yellow[0][1]), (trans_yellow[0][2]),0,0,0]
                # print(yellow_Cordinates)




        else:
            # print("No Transform Found")
            a = 1+1
        # print(tf_listener.frameExists("yellow_bell_pepper"))

        # global trans_red
        # listener.waitForTransform("/red_bell_pepper", "/ebot_base", rospy.Time(0), rospy.Duration(10))
        # trans_red = listener.lookupTransform("/ebot_base", "/red_bell_pepper", rospy.Time(0))
        # br.sendTransform(trans_red[0], trans_red[1], rospy.Time.now(), "fruit_red", "ebot_base")
        
        # global red_Cordinates
        # red_Cordinates = [round(trans_red[0][0], 3), round(trans_red[0][1], 3), round(trans_red[0][2], 3),0,0,0]


    except Exception as e:
        print("Exception Occured: ",e)
        pass

# def tf_broadcaster(X, Y, Z, fruit, parent):
#     br = tf.TransformBroadcaster()
#     br.sendTransform((X, Y, Z),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      fruit,
#                      parent)

class Ur5Moveit:

    # Constructor
    def __init__(self):
        rospy.init_node("node_eg3_set_joint_angles", anonymous=True)

        self._planning_group = "arm"
        self._gripper_group = "gripper"
        # gripper = moveit_commander.MoveGroupCommander('gripper')
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._gripper = moveit_commander.MoveGroupCommander(self._gripper_group)
        # self._group.set_planner_id("RRTConnectkConfigDefault")
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            "\033[94m" + "Planning Group: {}".format(self._planning_frame) + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m" + "End Effector Link: {}".format(self._eef_link) + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m" + "Group Names: {}".format(self._group_names) + "\033[0m"
        )
        rospy.loginfo("\033[94m" + " >>> Ur5Moveit init done." + "\033[0m")

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")
        else:
            rospy.logerr("\033[94m" + ">>> set_joint_angles() Failed." + "\033[0m")

        return flag_plan

    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Current Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)

        if flag_plan == True:
            rospy.loginfo("\033[94m" + ">>> go_to_pose() Success" + "\033[0m")
        else:
            rospy.logerr(
                "\033[94m"
                + ">>> go_to_pose() Failed. Solution for Pose not Found."
                + "\033[0m"
            )

        return flag_plan

    def open_gripper(self):
        self._gripper.set_joint_value_target([0.0])
        self._gripper.go()

    def close_gripper(self):
        self._gripper.set_joint_value_target([0.8])
        self._gripper.go()

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

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo(
            "\033[94m"
            + "\n"
            + "End-Effector ({}) Pose: \n\n".format(self._eef_link)
            + "x: {}\n".format(pose_values.position.x)
            + "y: {}\n".format(pose_values.position.y)
            + "z: {}\n\n".format(pose_values.position.z)
            + "roll: {}\n".format(roll)
            + "pitch: {}\n".format(pitch)
            + "yaw: {}\n".format(yaw)
            + "\033[0m"
        )

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("\033[94m" + "Object of class Ur5Moveit Deleted." + "\033[0m")

    # #temp[5] = yaw
    # #temp[4] = roll
ur5 = Ur5Moveit()


def bot_status_updater(status):
    global bot_Status
    bot_Status = status.data

def get_fruit(fruit_cordinates, drop_angles, fruit_Name, scan, tf_listener) :
    global ur5
    # TODO print("inside get_fruit")
    temp = []
    ur5._group.set_planner_id("KPIECE")
    ur5.open_gripper()
    temp = fruit_cordinates.copy()
    temp[0] = temp[0] - 0.04
    temp[1] = temp[1] - 0.25
    temp[3] = -math.pi/4
    temp[5] = 0
    # temp[4] = math.pi
    x, y, z, w = False, False, False, False
    while x == False and y == False and z == False:
        ur5.go_to_pose(temp)
        currentPose = ur5._group.get_current_pose().pose
        x = True if(round(currentPose.position.x, 2)==round(temp[0], 2)) else False
        y = True if(round(currentPose.position.y, 2)==round(temp[1], 2)) else False
        z = True if(round(currentPose.position.z, 2)==round(temp[3], 2)) else False 

    temp[3] += math.pi/4
    x, y, z, w = False, False, False, False
    while x == False and y == False and z == False and w == False:
        ur5.go_to_pose(temp)
        currentPose = ur5._group.get_current_pose().pose
        x = True if(round(currentPose.position.x, 2)==round(temp[0], 2)) else False
        y = True if(round(currentPose.position.y, 2)==round(temp[1], 2)) else False
        z = True if(round(currentPose.position.z, 2)==round(temp[3], 2)) else False 
        w = True if(round(currentPose.orientation.x, 2)==round(temp[3], 2)) else False 
        # print("Set Joint Angles",ur5.set_joint_angles(list_joint_values))

    ur5.close_gripper()
    ur5.set_joint_angles(drop_angles)
    ur5.open_gripper()
    ur5.set_joint_angles(scan)

    # temp = fruit2.copy()
    # temp[0] = temp[0] + 0.005
    # temp[1] = temp[1] - 0.25
    # temp[3] = -math.pi/4
    # temp[5] = 0
    # # temp[4] = math.pi
    # x, y, z, w = False, False, False, False
    # while x == False and y == False and z == False :
    #     print("Motion plan First",ur5.go_to_pose(temp))
    #     currentPose = ur5._group.get_current_pose().pose
    #     x = True if(round(currentPose.position.x, 2)==round(temp[0], 2)) else False
    #     y = True if(round(currentPose.position.y, 2)==round(temp[1], 2)) else False
    #     z = True if(round(currentPose.position.z, 2)==round(temp[3], 2)) else False 

    # temp[3] += math.pi/4
    # x, y, z, w = False, False, False, False
    # while x == False and y == False and z == False and w == False:
    #     print("Motion plan Second",ur5.go_to_pose(temp))
    #     currentPose = ur5._group.get_current_pose().pose
    #     x = True if(round(currentPose.position.x, 2)==round(temp[0], 2)) else False
    #     y = True if(round(currentPose.position.y, 2)==round(temp[1], 2)) else False
    #     z = True if(round(currentPose.position.z, 2)==round(temp[3], 2)) else False 
    #     w = True if(round(currentPose.orientation.x, 2)==round(temp[3], 2)) else False 
    #     # print("Set Joint Angles",ur5.set_joint_angles(list_joint_values))
    # ur5.close_gripper()
    # ur5.set_joint_angles(drop2)
    # ur5.open_gripper()

def main():
    global ur5
    global yellow_Cordinates
    global red_Cordinates
    global temp_yellow

    lst_joint_angles_picking_pose = [
        # math.radians(-90),
        # math.radians(-10),
        # math.radians(-20),
        # math.radians(30),
        # math.radians(0),
        # math.radians(0),

        # math.radians(0),
        # math.radians(0),
        # math.radians(-45),
        # math.radians(45),
        # math.radians(-90),
        # math.radians(0),

        math.radians(90),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
    ]
    

    lst_joint_angles_scan = [
        math.radians(56),
        math.radians(-45),
        math.radians(-45),
        math.radians(85),
        math.radians(-35),
        math.radians(0)
        # math.radians(90),
        # math.radians(-45),
        # math.radians(-45),
        # math.radians(90),
        # math.radians(0),
        # math.radians(0),
        # 90 -30 -30 60 0 0 
        # math.radians(-90),
        # math.radians(-30),
        # math.radians(-30),
        # math.radians(60),
        # math.radians(0),
        # math.radians(0),
    ]

    lst_joint_angles_3 = [
        math.radians(25),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
    ]

    lst_joint_angles_6 = [
        math.radians(-25),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
        math.radians(0),
    ]
    rate = rospy.Rate(5)
    tf_listener = tf.TransformListener()
    rospy.Subscriber("bot_status",String, bot_status_updater)
    bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
    ur5.set_joint_angles(lst_joint_angles_scan)
    tf_listener_and_broadcaster(tf_listener)
    while not rospy.is_shutdown():
        tf_listener_and_broadcaster(tf_listener)
        if((len(yellow_Cordinates) !=0 or len(red_Cordinates) !=0) and bot_Status == "stop"):
            try:
                if(len(yellow_Cordinates) !=0):
                    ur5.set_joint_angles(lst_joint_angles_picking_pose)
                    # TODO print("                    tring [YELLOW] get_fruit") 
                    temp_yellow = yellow_Cordinates
                    get_fruit(yellow_Cordinates, lst_joint_angles_3,"yellow_bell_pepper", lst_joint_angles_scan, tf_listener)
                
                # if(len(red_Cordinates) !=0):
                #     ur5.set_joint_angles(lst_joint_angles_picking_pose)
                #     # TODO print("                    tring [RED] get_fruit")
                #     print("red cordinates: ", red_Cordinates)
                #     print("is red Cordinates  = previous yellow cordinates? ", bool(temp_yellow == red_Cordinates))
                #     get_fruit(red_Cordinates, lst_joint_angles_6, "red_bell_pepper", lst_joint_angles_scan, tf_listener)
            except Exception as e:
                print("Exception in main: ", e)

            finally:
                yellow_Cordinates = []
                red_Cordinates = []
                bot_pub.publish("ready")
                print(bot_Status)
            
        # if(len(red_Cordinates) !=0):
        #     print("                    tring [RED] get_fruit")
        #     get_fruit(red_Cordinates, lst_joint_angles_6)
        # rospy.sleep(2)
        rate.sleep()
    del ur5


if __name__ == "__main__":
    main()
    a = 2+3

