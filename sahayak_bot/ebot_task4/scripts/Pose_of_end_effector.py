#! /usr/bin/env python

import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('Pose_of_End-Effector', anonymous=True)

        self._planning_group = "arm_controller"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

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

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w


        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "x1: {}\n".format(q_x) +
                      "y1: {}\n".format(q_y) +
                      "z1: {}\n".format(q_z) +
                      "w1: {}\n".format(q_w) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.print_pose_ee()
        ur5.print_joint_angles()
        rospy.sleep(1)
        break

    del ur5


if __name__ == '__main__':
    main()


