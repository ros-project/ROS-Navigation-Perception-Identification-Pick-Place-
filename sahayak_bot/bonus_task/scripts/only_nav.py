#!/usr/bin/env python

# All header Files for Navigation
import rospy
import roslaunch
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2
import math
PI = 3.1415926535897


# All header Files for Object Detection
import rospy
import numpy
import tf
from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from dodo_detector.detection import SingleShotDetector, KeypointObjectDetector
from dodo_detector_ros.msg import DetectedObject, DetectedObjectArray

# All Header Files to control Moveit
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global Variables for odom_callback
sar=0
viv=0
rar=0
roll = pitch = yaw = 0.0
# Global Variables for laser_callback
regions = {
        'right':  0,
        'fright': 0,
        'front':  0,
        'fleft':  0,
        'left':   0,
    }

# Global Dictionary For Object Detection
rospy.sleep(10)

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


    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose

        list_joint_values = self._group.get_current_joint_values()

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()


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



    def go_to_predefined_pose(self, arg_pose_name):
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()

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


    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()

        pose_values = self._group.get_current_pose().pose

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()


class Ur5Moveit_info:

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



    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w


    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()

# For Recieving Data from Odometry
def odom_callback(msg):
    global sar,roll, pitch, yaw, viv, rar
    x  = msg.pose.pose.orientation.x;
    y  = msg.pose.pose.orientation.y;
    z = msg.pose.pose.orientation.z;
    w = msg.pose.pose.orientation.w;
    sar=msg.pose.pose.position.x
    viv=msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    rar=euler_from_quaternion([x,y,z,w])[2]


# For Recieving Data from Lidar Sensor
# 'front':  min(min(msg.ranges[288:431]), 100),
def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[:143]), 100),
        'fright': min(min(msg.ranges[144:287]), 100),
        'front':  min(min(msg.ranges[358:361]), 100),
        'fleft':  min(min(msg.ranges[432:575]), 100),
        'left':   min(min(msg.ranges[576:713]), 100),
    }
    # rospy.loginfo(regions['front'])


# Goint to point (x,y) without orientation
# For Forward state=1
# For Backward state=0
def move(point,state):
    speed = 0.5
    distance = point
    isForward = state
    if(isForward):
        velocity_msg.linear.x =0.25
    else:
        velocity_msg.linear.x =-0.25
    #Since we are moving just in x-axis
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance < distance):
        #Publish the velocity
        pub.publish(velocity_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    velocity_msg.linear.x = 0
    #Force the robot to stop
    pub.publish(velocity_msg)


def rotate_using_lidar_anticlockwise(target_val):
    velocity_msg.linear.x=0
    velocity_msg.linear.y=0
    velocity_msg.linear.z=0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    while not rospy.is_shutdown():
        velocity_msg.angular.z=0.5
        cur_value=regions['front']
        min_value=cur_value-0.1
        max_value=cur_value+0.1
        if max_value>=target_val and min_value<=target_val:
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            break
        pub.publish(velocity_msg)

def rotate_using_lidar_clockwise(target_val):
    velocity_msg.linear.x=0
    velocity_msg.linear.y=0
    velocity_msg.linear.z=0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    while not rospy.is_shutdown():
        velocity_msg.angular.z=-0.5
        cur_value=regions['front']
        min_value=cur_value-0.1
        max_value=cur_value+0.1
        if max_value>=target_val and min_value<=target_val:
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            break
        pub.publish(velocity_msg)

def movebase_client(x1,y1,z1):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =x1
    goal.target_pose.pose.position.y =y1
    goal.target_pose.pose.orientation.w =1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



if __name__ == '__main__':
    try:
        rospy.init_node('navigation')
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        velocity_msg = Twist()
        ur5 = Ur5Moveit()
        ur51 = Ur5Moveit_help()
        ur52 = Ur5Moveit_helper()

        # 1
        print("Started Run!")

# ---------------------Going In Center of Pantry---------------------------------        
        result = movebase_client(13.1,1.05,1.0)
        rotate_using_lidar_clockwise(3.3)
        result = movebase_client(13.1,-0.9,1.0)

        # 2
        print("Pantry Reached")
# ---------------------All Joint Angles For Pantry---------------------------------

        zero = [math.radians(-0),
                  math.radians(1),
                  math.radians(-36),
                  math.radians(16),
                  math.radians(0),
                  math.radians(19)]
        all_zeros = [math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]

        pos = [math.radians(-91),
                math.radians(-27),
                math.radians(-24),
                math.radians(-63),
                math.radians(-14),
                math.radians(81)]

        pos1 = [math.radians(89),
                math.radians(-27),
                math.radians(-24),
                math.radians(-63),
                math.radians(-14),
                math.radians(81)]

        right = [math.radians(-131),
                math.radians(-18),
                math.radians(-24),
                math.radians(-31),
                math.radians(-51),
                math.radians(24)]

        left = [math.radians(-73),
                math.radians(-47),
                math.radians(-17),
                math.radians(-59),
                math.radians(-40),
                math.radians(95)]

        left_point_cloud = [math.radians(-98),
                              math.radians(33),
                              math.radians(-72),
                              math.radians(-13),
                              math.radians(-48),
                              math.radians(20)]

        left_pick_before = [math.radians(-99),
                              math.radians(-30),
                              math.radians(-19),
                              math.radians(-74),
                              math.radians(-90),
                              math.radians(90)]

        right_point_cloud = [math.radians(108),
                              math.radians(37),
                              math.radians(-60),
                              math.radians(-13),
                              math.radians(-48),
                              math.radians(6)]

        right_pick_before = [math.radians(122),
                              math.radians(-3),
                              math.radians(-50),
                              math.radians(-66),
                              math.radians(-92),
                              math.radians(57)]

# # ---------------------Scanning For Coke---------------------------------

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos)
            break

        result = movebase_client(13.1,-0.9,1.0)       

# ---------------------Going To Meeting Room[Drop Box 1]-------------------------------
        rotate_using_lidar_anticlockwise(13)
        move(1.7,1)
        result = movebase_client(6.9,2.4,0.5)

        # 5
        print("Meeting Room Reached")

# # # # ---------------------Dropping The Coke----------------------------------

        pos0 = [math.radians(-4),
                math.radians(-27),
                math.radians(-24),
                math.radians(-63),
                math.radians(-27),
                math.radians(81)]
        pos00 = [math.radians(82),
                math.radians(22),
                math.radians(-73),
                math.radians(-127),
                math.radians(-82),
                math.radians(178)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos0)
            ur5.set_joint_angles(pos00)
            ur51.go_to_predefined_pose("open")
            ur5.set_joint_angles(pos0)
            break

        # 6
        print("Coke Dropped in DropBox2")

        result = movebase_client(6.9,2.4,0.5)
# # # ---------------------Going To Detect Glue----------------------------------

        move(1.4,1)
        # result = movebase_client(7.55,2.35,0.5)

        pos_tensor = [math.radians(-10),
                math.radians(-16),
                math.radians(-21),
                math.radians(-66),
                math.radians(-28),
                math.radians(69)]

        pos_center = [math.radians(-7),
                math.radians(-35),
                math.radians(-24),
                math.radians(-55),
                math.radians(-46),
                math.radians(76)]

        pos_left = [math.radians(-6),
                math.radians(-47),
                math.radians(-23),
                math.radians(-35),
                math.radians(-32),
                math.radians(71)]

        pos_right = [math.radians(6),
                math.radians(-2),
                math.radians(-32),
                math.radians(-89),
                math.radians(-31),
                math.radians(94)]

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos_tensor)
            break

        # result = movebase_client(7.55,2.35,0.5)

        # if ind==3:
            # while not rospy.is_shutdown():
                # ur5.set_joint_angles(pos_right)
                # break

        # if ind==1:
            # while not rospy.is_shutdown():
                # ur5.set_joint_angles(pos_left)
                # break



# # # ---------------------Going To Research Lab[Drop Box 2]----------------------------------
        result = movebase_client(8.1,2.4,0.5)
        rotate_using_lidar_clockwise(2.18)
        pos2 = [math.radians(-45),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos2)
            break
        rotate_using_lidar_clockwise(2.3)
        move(3,1)
        result = movebase_client(10.7,9.4,1.0)

        # 9
        print("Research lab Reached")
# # # ---------------------Dropping The Glue----------------------------------
        pos3 = [math.radians(-90),
            math.radians(-27),
            math.radians(-24),
            math.radians(-63),
            math.radians(-27),
            math.radians(81)]
        pos4 = [math.radians(-11),
                math.radians(36),
                math.radians(-96),
                math.radians(-123),
                math.radians(-77),
                math.radians(169)]
        pos5 = [math.radians(0),
                math.radians(-19),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos3)
            ur5.set_joint_angles(pos4)
            ur51.go_to_predefined_pose("open")
            break

        # 10
        print("Glue Dropped in DropBox3")

# ---------------------Going To Store Room---------------------------------
        rotate_using_lidar_clockwise(8.8)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos5)
            break
        rotate_using_lidar_clockwise(8.8)
        move(10,1)
        result = movebase_client(25.0,-2.6,1.0)

        # 11
        print("Store Room Reached")
# # # # # # ---------------------Picking fgpa-board----------------------------------
        store_0 = [math.radians(-120),
                  math.radians(-20),
                  math.radians(-23),
                  math.radians(8),
                  math.radians(-1),
                  math.radians(1)]

        store_left = [math.radians(-129),
                  math.radians(-15),
                  math.radians(-44),
                  math.radians(-5),
                  math.radians(-49),
                  math.radians(23)]

        store_right = [math.radians(-75),
                  math.radians(-31),
                  math.radians(-57),
                  math.radians(-4),
                  math.radians(-46),
                  math.radians(73)]


        store_left_pick_before = [math.radians(-66),
                          math.radians(-41),
                          math.radians(-22),
                          math.radians(-71),
                          math.radians(-83),
                          math.radians(156)]


        store_right_pick_before = [math.radians(46),
                                  math.radians(-13),
                                  math.radians(-44),
                                  math.radians(-65),
                                  math.radians(-106),
                                  math.radians(268)]

        store_right_pick_before_index_4 = [math.radians(37),
                                          math.radians(-13),
                                          math.radians(-42),
                                          math.radians(-68),
                                          math.radians(-105),
                                          math.radians(257)]


        while not rospy.is_shutdown():
            ur5.set_joint_angles(store_0)
            break


# For moving left
        # while not rospy.is_shutdown():
            # ur5.set_joint_angles(store_left_pick_before)
            # ur52.go_to_pose(ur52_pose_left)
            # ur52.go_to_pose(ur52_pose_final_left)
            # ur51.go_to_predefined_pose("mid23")
            # rospy.sleep(1)
            # ur52.go_to_pose(ur52_pose_left)
            # break

# For moving right
        # while not rospy.is_shutdown():
            # ur5.set_joint_angles(store_right_pick_before)
            # if index==4:
                # ur5.set_joint_angles(store_right_pick_before_index_4)
            # ur52.go_to_pose(ur52_pose_right)
            # ur52.go_to_pose(ur52_pose_final_right)
            # ur51.go_to_predefined_pose("mid3")
            # rospy.sleep(1)
            # ur52.go_to_pose(ur52_pose_right)
            # break

        # 13
        print("FPGA board Picked")
# ---------------------Going In Conference Room----------------------------------

        move(2.0,0)
        rotate_using_lidar_clockwise(17.6)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros)
            break
        result = movebase_client(5.4,-0.6,1.0)

        # 14
        print("Conference Room Reached")

        con_room = [math.radians(0),
                  math.radians(0),
                  math.radians(-39),
                  math.radians(-180),
                  math.radians(-37),
                  math.radians(181)]

        con_room1 = [math.radians(-95),
                  math.radians(0),
                  math.radians(-39),
                  math.radians(-180),
                  math.radians(-37),
                  math.radians(181)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(con_room)
            ur5.set_joint_angles(con_room1)
            ur51.go_to_predefined_pose("open")
            break

        # 15
        print("FPGA board Dropped in DropBox1")
# ---------------------Going to Origin----------------------------------
        rotate_using_lidar_anticlockwise(0.8)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros)
            break

        move(0.7,1)
        rotate_using_lidar_anticlockwise(2.2)
        move(2.8,1)
        rotate_using_lidar_anticlockwise(17.6)
        move(15,1)
        rotate_using_lidar_anticlockwise(7.1)
        move(1,1)
        rotate_using_lidar_anticlockwise(3.9)
        result = movebase_client(0,0,1.0)

        # 16
        print("Mission Accomplished!")



        del ur5
        del ur51
        del ur52

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
