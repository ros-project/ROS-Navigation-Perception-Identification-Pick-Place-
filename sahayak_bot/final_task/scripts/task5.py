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
    ur52=Ur5Moveit_help()

    #right one
    right = [math.radians(-99),
            math.radians(-29),
            math.radians(-23),
            math.radians(-58),
            math.radians(-37),
            math.radians(74)]

    left = [math.radians(-104),
            math.radians(-32),
            math.radians(-24),
            math.radians(-26),
            math.radians(-60),
            math.radians(54)]

    special_case = [math.radians(17),
                    math.radians(3),
                    math.radians(-58),
                    math.radians(-58),
                    math.radians(-131),
                    math.radians(260)]

# previous point cloud pose


    store_room = [math.radians(-120),
                    math.radians(-20),
                    math.radians(-23),
                    math.radians(-8),
                    math.radians(-1),
                    math.radians(1)]

    pos4 = [math.radians(-111),
              math.radians(-5),
              math.radians(-12),
              math.radians(-10),
              math.radians(-55),
              math.radians(2)]

#pantry pick pose

    
    detect_pose = [math.radians(11),
                    math.radians(-97),
                    math.radians(-9),
                    math.radians(265),
                    math.radians(254),
                    math.radians(185)]

    detect_pose1 = [math.radians(80),
                    math.radians(-60),
                    math.radians(21),
                    math.radians(-145),
                    math.radians(192),
                    math.radians(206)]


    zero = [math.radians(-0),
              math.radians(1),
              math.radians(-36),
              math.radians(16),
              math.radians(0),
              math.radians(19)]

# pick before pose
    pick_before = [math.radians(-99),
                  math.radians(-30),
                  math.radians(-19),
                  math.radians(-74),
                  math.radians(-90),
                  math.radians(90)]

    pick_after = [math.radians(2),
                  math.radians(-11),
                  math.radians(-35),
                  math.radians(-78),
                  math.radians(-97),
                  math.radians(157)]

    starting = [math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]

    right_pick_before = [math.radians(122),
                          math.radians(-3),
                          math.radians(-50),
                          math.radians(-66),
                          math.radians(-92),
                          math.radians(57)]

    pos = [math.radians(-91),
            math.radians(-27),
            math.radians(-24),
            math.radians(-63),
            math.radians(-14),
            math.radians(81)]

    pantry_point_cloud = [math.radians(108),
                          math.radians(37),
                          math.radians(-60),
                          math.radians(-13),
                          math.radians(-48),
                          math.radians(6)]

    store_point_cloud_left = [math.radians(-120),
                                  math.radians(-4),
                                  math.radians(-42),
                                  math.radians(-4),
                                  math.radians(-40),
                                  math.radians(26)]

    store_point_cloud_right = [math.radians(37),
                                  math.radians(-30),
                                  math.radians(9),
                                  math.radians(-172),
                                  math.radians(-115),
                                  math.radians(193)]

    store_point_cloud_right1 = [math.radians(37),
                                  math.radians(-30),
                                  math.radians(9),
                                  math.radians(-172),
                                  math.radians(-115),
                                  math.radians(193)]


    store_left_pick_before = [math.radians(-42),
                          math.radians(-28),
                          math.radians(-26),
                          math.radians(-82),
                          math.radians(-89),
                          math.radians(180)]

    temp = [math.radians(-42),
              math.radians(-4),
              math.radians(-55),
              math.radians(-86),
              math.radians(-89),
              math.radians(180)]

    pos_tensor = [math.radians(-10),
            math.radians(-16),
            math.radians(-21),
            math.radians(-66),
            math.radians(-28),
            math.radians(69)]

    while not rospy.is_shutdown():
        # ur52.go_to_predefined_pose("close")
        ur52.go_to_predefined_pose("mid4")
        # ur5.set_joint_angles(pos_tensor)
        break
        # ur5.set_joint_angles(store_left_pick_before)
        # break
    del ur5
    del ur52


if __name__ == '__main__':
    main()

