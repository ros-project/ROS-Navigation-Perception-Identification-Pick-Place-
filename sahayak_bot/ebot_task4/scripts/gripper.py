#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit_help:

    # Constructor
    def __init__(self):

        rospy.init_node('set_joint_angles', anonymous=True)

        self._planning_group = "gripper_controller"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
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

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit_help init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit_help Deleted." + '\033[0m')


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('set_joint_angles', anonymous=True)

        self._planning_group = "arm_controller"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
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

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    ur51 = Ur5Moveit_help()


    soap = [math.radians(-11),
                          math.radians(-43),
                          math.radians(80),
                          math.radians(50),
                          math.radians(89),
                          math.radians(-53)]


    container_1= [math.radians(88),
                          math.radians(-54),
                          math.radians(80),
                          math.radians(-111),
                          math.radians(-85),
                          math.radians(0)]


    soap_2=  [math.radians(-141),
                          math.radians(-137),
                          math.radians(-84),
                          math.radians(129),
                          math.radians(-88),
                          math.radians(-79)]

    container_2= [math.radians(-92),
                          math.radians(-54),
                          math.radians(80),
                          math.radians(-111),
                          math.radians(-85),
                          math.radians(0)]
    
    pos1= [math.radians(-93),
                          math.radians(-58),
                          math.radians(60),
                          math.radians(-87),
                          math.radians(-85),
                          math.radians(-1)]
    
    pos2= [math.radians(-38),
                          math.radians(-74),
                          math.radians(88),
                          math.radians(-105),
                          math.radians(-82),
                          math.radians(54)]

    pos3= [math.radians(-37),
                          math.radians(-67),
                          math.radians(99),
                          math.radians(-123),
                          math.radians(-82),
                          math.radians(55)]

    pos4= [math.radians(-36),
                          math.radians(-60),
                          math.radians(103),
                          math.radians(-134),
                          math.radians(-82),
                          math.radians(55)]
    pos5= [math.radians(-36),
                          math.radians(-59),
                          math.radians(104),
                          math.radians(-136),
                          math.radians(-82),
                          math.radians(56)]

    biscuit= [math.radians(323),
                          math.radians(-55),
                          math.radians(99),
                          math.radians(226),
                          math.radians(268),
                          math.radians(-307)]

    container= [math.radians(5),
                          math.radians(-10),
                          math.radians(-6),
                          math.radians(-195),
                          math.radians(-94),
                          math.radians(186)]


    container1= [math.radians(-26),
                          math.radians(-16),
                          math.radians(1),
                          math.radians(-196),
                          math.radians(-68),
                          math.radians(187)]
    
    while not rospy.is_shutdown():
        ur5.set_joint_angles(container)
        #ur51.go_to_predefined_pose("open")
        break
    del ur5
    del ur51


if __name__ == '__main__':
    main()
