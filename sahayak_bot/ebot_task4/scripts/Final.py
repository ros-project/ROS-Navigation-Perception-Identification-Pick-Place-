#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from ebot_task4.msg import legends

x1=-1
y1=-1
z1=-1
x2=-1
y2=-1
z2=-1
x3=-1
y3=-1
z3=-1
def func_callback(myMsg):
    global x1,y1,z1,x2,y2,z2,x3,y3,z3
    x1=myMsg.x1
    y1=myMsg.y1
    z1=myMsg.z1


    x2=myMsg.x2
    y2=myMsg.y2
    z2=myMsg.z2

    x3=myMsg.x3
    y3=myMsg.y3
    z3=myMsg.z3


class Ur5Moveit_helper:

    # Constructor
    def __init__(self):

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
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit_helper init done." + '\033[0m')

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

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit_helper Deleted." + '\033[0m')
class Ur5Moveit_help:

    # Constructor
    def __init__(self):

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

    rospy.init_node('Final', anonymous=True)
    rospy.Subscriber('/legends', legends, func_callback)
    # Set Joint Angles
    ur5 = Ur5Moveit()
    # Pre Define Pose
    ur51 = Ur5Moveit_help()
    # Going to a pose
    ur52=Ur5Moveit_helper()

    container= [math.radians(5),
                  math.radians(-10),
                  math.radians(-6),
                  math.radians(-195),
                  math.radians(-94),
                  math.radians(186)]

    container_pos = geometry_msgs.msg.Pose()

    container_pos.position.x = 0.29446163014
    container_pos.position.y = 0.13535375479
    container_pos.position.z = 1.0284651074
    container_pos.orientation.x = -0.699562157391
    container_pos.orientation.y = -0.712554927569
    container_pos.orientation.z = 0.0433310382576
    container_pos.orientation.w = 0.0316335939707

    container1=[math.radians(-26),
              math.radians(-16),
              math.radians(1),
              math.radians(-196),
              math.radians(-68),
              math.radians(157)]
    pos3 = [math.radians(-4),
              math.radians(-27),
              math.radians(-24),
              math.radians(-63),
              math.radians(-27),
              math.radians(81)]

    ur52_pose_0 = geometry_msgs.msg.Pose()
    ur52_pose_1 = geometry_msgs.msg.Pose()
    ur52_pose_2 = geometry_msgs.msg.Pose()
    ur52_pose_3 = geometry_msgs.msg.Pose()
    ur52_pose_4 = geometry_msgs.msg.Pose()
    ur52_pose_5 = geometry_msgs.msg.Pose()
    while 1:
        if(x1>0):
            #Coca Cola----------------------------------------- 
            ur52_pose_0.position.x = x1+0.016
            ur52_pose_0.position.y = y1-0.300
            ur52_pose_0.position.z = 1.021254198886
            ur52_pose_0.orientation.x = 0.033404070776
            ur52_pose_0.orientation.y = -0.979504461711
            ur52_pose_0.orientation.z = 0.198085360143
            ur52_pose_0.orientation.w = 0.0147433931491

            ur52_pose_1 = geometry_msgs.msg.Pose()
            ur52_pose_1.position.x = x1+0.016
            ur52_pose_1.position.y =y1-0.200
            ur52_pose_1.position.z = 0.921254198886
            ur52_pose_1.orientation.x = 0.033404070776
            ur52_pose_1.orientation.y = -0.979504461711
            ur52_pose_1.orientation.z = 0.198085360143
            ur52_pose_1.orientation.w = 0.0147433931491

            #Battery-----------------------------------------    
            ur52_pose_2.position.x = x2+0.016
            ur52_pose_2.position.y = y2-0.300
            ur52_pose_2.position.z = 1.021254198886
            ur52_pose_2.orientation.x = 0.033404070776
            ur52_pose_2.orientation.y = -0.979504461711
            ur52_pose_2.orientation.z = 0.198085360143
            ur52_pose_2.orientation.w = 0.0147433931491

            ur52_pose_3 = geometry_msgs.msg.Pose()
            ur52_pose_3.position.x = x2+0.016
            ur52_pose_3.position.y =y2-0.200
            ur52_pose_3.position.z = 0.921254198886
            ur52_pose_3.orientation.x = 0.033404070776
            ur52_pose_3.orientation.y = -0.979504461711
            ur52_pose_3.orientation.z = 0.198085360143
            ur52_pose_3.orientation.w = 0.0147433931491

            #Glue-----------------------------------------    
            ur52_pose_4 = geometry_msgs.msg.Pose()
            ur52_pose_4.position.x = x3+0.016
            ur52_pose_4.position.y = y3-0.300
            ur52_pose_4.position.z = 1.019192397391
            ur52_pose_4.orientation.x = 0.033404070776
            ur52_pose_4.orientation.y = -0.979504461711
            ur52_pose_4.orientation.z = 0.198085360143
            ur52_pose_4.orientation.w = 0.0147433931491

            ur52_pose_5 = geometry_msgs.msg.Pose()
            ur52_pose_5.position.x = x3+0.016
            ur52_pose_5.position.y =y3-0.200
            ur52_pose_5.position.z = 0.919192397391
            ur52_pose_5.orientation.x = 0.033404070776
            ur52_pose_5.orientation.y = -0.979504461711
            ur52_pose_5.orientation.z = 0.198085360143
            ur52_pose_5.orientation.w = 0.0147433931491
            break
    
    while not rospy.is_shutdown():
            ur52.go_to_pose(ur52_pose_0)
            ur52.go_to_pose(ur52_pose_1)
            ur51.go_to_predefined_pose("mid3")
            ur52.go_to_pose(ur52_pose_0)
            ur52.go_to_pose(container_pos)
            ur51.go_to_predefined_pose("open")
            ur52.go_to_pose(ur52_pose_2)
            ur52.go_to_pose(ur52_pose_3)
            ur51.go_to_predefined_pose("mid4")
            ur52.go_to_pose(ur52_pose_2)
            ur52.go_to_pose(container_pos)
            ur51.go_to_predefined_pose("open")
            ur52.go_to_pose(ur52_pose_4)
            ur52.go_to_pose(ur52_pose_5)
            ur51.go_to_predefined_pose("mid3")
            ur52.go_to_pose(container_pos)
            ur51.go_to_predefined_pose("open")
            break
    del ur5
    del ur51
    del ur52


if __name__ == '__main__':
    main()
