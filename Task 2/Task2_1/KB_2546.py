#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from control_msgs.msg import GripperCommand


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm"
        self._gripper_group = "gripper"
        # gripper = moveit_commander.MoveGroupCommander('gripper')
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._gripper = moveit_commander.MoveGroupCommander(self._gripper_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def open_gripper(self):
        self._gripper.set_joint_value_target([0.0])
        self._gripper.go()

    #close the gripper in UR5 arm
    def close_gripper(self):
        self._gripper.set_joint_value_target([0.8])
        self._gripper.go()

    def shaky_shaky_1(self):
        self.set_joint_angles([math.radians(19),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(-30)])
        self.set_joint_angles([math.radians(19),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(30)])
        self.set_joint_angles([math.radians(19),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)])

    def shaky_shaky_2(self):
        self.set_joint_angles([math.radians(-25),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(-30)])
        self.set_joint_angles([math.radians(-25),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(30)])
        self.set_joint_angles([math.radians(-25),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)])
    
    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    lst_joint_angles_0 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]


    lst_joint_angles_1 = [math.radians(94),
                          math.radians(50),
                          math.radians(-29),
                          math.radians(-20),
                          math.radians(1),
                          math.radians(1)]

    lst_joint_angles_2 = [math.radians(93),
                          math.radians(58),
                          math.radians(-41),
                          math.radians(-14),
                          math.radians(-1),
                          math.radians(0)]

    lst_joint_angles_3 = [math.radians(25),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_4 = [math.radians(72),
                          math.radians(90),
                          math.radians(-169),
                          math.radians(74),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_5 = [math.radians(74),
                          math.radians(93),
                          math.radians(-163),
                          math.radians(50),
                          math.radians(2),
                          math.radians(0)]

    lst_joint_angles_6 = [math.radians(-25),
                          math.radians(0),
                          math.radians(0),  
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

   
    while not rospy.is_shutdown():
        
        #Go near Fruit 1
        ur5.set_joint_angles(lst_joint_angles_1)
        
        #Get Close to Fruit 1 
        ur5.set_joint_angles(lst_joint_angles_2)
        
        #Close Gripper
        ur5.close_gripper()
        
        #Get Back
        ur5.set_joint_angles(lst_joint_angles_1)
        
        #Go to Yellow Basket
        ur5.set_joint_angles(lst_joint_angles_3)
        
        #Open Gripper
        ur5.open_gripper()

        #Shaky-Shaky 1
        # ur5.shaky_shaky_1()
        
        #Get near Fruit 2
        ur5.set_joint_angles(lst_joint_angles_4)      
        
        #Get Close to Fruit 2
        ur5.set_joint_angles(lst_joint_angles_5)      
        
        #Close Gripper
        ur5.close_gripper()      
        
        #Get Back
        ur5.set_joint_angles(lst_joint_angles_4)     
        
        #Go to Red Basket
        ur5.set_joint_angles(lst_joint_angles_6)     
        
        #Open Gripper
        ur5.open_gripper()

        #Shaky-Shaky 2
        # ur5.shaky_shaky_2()        
        
        #Rest Position
        ur5.set_joint_angles(lst_joint_angles_0)      
        break

    del ur5


if __name__ == '__main__':
    main()