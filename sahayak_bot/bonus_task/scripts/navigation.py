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
from task5_msgs.msg import Data_msg
from task5_msgs.msg import Flag_msg
from std_msgs.msg import Int64
from std_msgs.msg import String
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
object_data=dict()
rospy.sleep(10)

class Detector:

    def __init__(self):
       
#  get label map and inference graph from params
        detector_type = 'ssd'
        frozen_graph = '~/dodo_detector_task5/frozen_inference_graph.pb'
        label_map = '~/dodo_detector_task5/label_map.pbtxt'
        confidence = 0.5
        min_points = 10
        database_path = ''
        filters = {}
        image_topic = "/camera/color/image_raw2"
        point_cloud_topic = "/camera2/depth/points2"

        self._global_frame = "ebot_base"
        self._tf_prefix = rospy.get_name()

        # create a transform listener so we get the fixed frame the user wants
        # to publish object tfs related to
        self._tf_listener = tf.TransformListener()

        if detector_type == 'ssd':
            if len(frozen_graph) == 0:
                raise ValueError('Parameter \'frozen_graph\' must be passed')
            if len(label_map) == 0:
                raise ValueError('Parameter \'label_map\' must be passed')
            if confidence <= 0 or confidence > 1:
                raise ValueError('Parameter \'confidence\' must be between 0 and 1')

            frozen_graph = expanduser(frozen_graph)
            label_map = expanduser(label_map)

            self._detector = SingleShotDetector(frozen_graph, label_map, confidence=confidence)

            # count number of classes from label map
            label_map_contents = open(label_map, 'r').read()
            num_classes = label_map_contents.count('name:')

        elif detector_type in ['sift', 'rootsift']:
            if min_points <= 0:
                raise ValueError('Parameter \'min_points\' must greater than 0')
            if len(database_path) == 0:
                raise ValueError('Parameter \'database_path\' must be passed')

            database_path = expanduser(database_path)

            detector_type = 'SIFT' if detector_type == 'sift' else 'RootSIFT'
            self._detector = KeypointObjectDetector(database_path, detector_type, min_points=min_points)

        # create detector
        self._bridge = CvBridge()

        # image and point cloud subscribers
        # and variables that will hold their values
        rospy.Subscriber(image_topic, Image, self.image_callback)

        if point_cloud_topic is not None:
            rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
        else:
            rospy.loginfo('No point cloud information available. Objects will not be placed in the scene.')

        self._current_image = None
        self._current_pc = None

        # publisher for frames with detected objects
        self._imagepub = rospy.Publisher('labeled_image', Image, queue_size=10)

        # this package works with a dynamic list of publishers
        # if no filter is configured via parameters to the package,
        # one default, unfiltered publisher will publish every object
        if len(filters) == 0:
            self._publishers = {None: (None, rospy.Publisher('~detected', DetectedObjectArray, queue_size=10))}

        # else, for each filter created in the yaml config file, a new publisher is created
        else:
            self._publishers = {}
            for key in filters:
                cat_ok = False
                for cat in self._detector.categories:
                    if cat in filters[key]:
                        cat_ok = True
                        break

                if not cat_ok:
                    rospy.logwarn('Key ' + filters[key] + ' is not detected by this detector!')
                else:
                    self._publishers[key] = (filters[key], rospy.Publisher('~detected_' + key, DetectedObjectArray, queue_size=10))

        self._tfpub = tf.TransformBroadcaster()

    def image_callback(self, image):
        """Image callback"""
        # Store value on a private attribute
        self._current_image = image

    def pc_callback(self, pc):
        """Point cloud callback"""
        # Store value on a private attribute
        self._current_pc = pc

    def run(self,variable):
        flag=0
        # run while ROS runs
        count=0
        while not rospy.is_shutdown():
            # only run if there's an image present
            count=count+1
            if self._current_image is not None:
                try:

                    # if the user passes a fixed frame, we'll ask for transformation
                    # vectors from the camera link to the fixed frame
                    if self._global_frame is not None:
                        (trans, _) = self._tf_listener.lookupTransform('/' + self._global_frame, '/camera_link2', rospy.Time(0))

                    # convert image from the subscriber into an OpenCV image
                    scene = self._bridge.imgmsg_to_cv2(self._current_image, 'rgb8')
                    marked_image, objects = self._detector.from_image(scene)  # detect objects
                    self._imagepub.publish(self._bridge.cv2_to_imgmsg(marked_image, 'rgb8'))  # publish detection results

                    # well create an empty msg for each publisher
                    msgs = {}
                    for key in self._publishers:
                        msgs[key] = DetectedObjectArray()

                    # iterate over the dictionary of detected objects
                    flag=0
                    for obj_class in objects:
                        flag=flag+1
                        rospy.logdebug('Found ' + str(len(objects[obj_class])) + ' object(s) of type ' + obj_class)

                        for obj_type_index, coordinates in enumerate(objects[obj_class]):
                            rospy.logdebug('...' + obj_class + ' ' + str(obj_type_index) + ' at ' + str(coordinates['box']))

                            ymin, xmin, ymax, xmax = coordinates['box']
                            y_center = ymax - ((ymax - ymin) / 2)
                            x_center = xmax - ((xmax - xmin) / 2)

                            detected_object = DetectedObject()
                            detected_object.type.data = obj_class
                            detected_object.image_x.data = xmin
                            detected_object.image_y.data = ymin
                            detected_object.image_width.data = xmax - xmin
                            detected_object.image_height.data = ymax - ymin
                            # TODO the timestamp of image, depth and point cloud should be checked
                            # to make sure we are using synchronized data...

                            publish_tf = False
                            if self._current_pc is None:
                                rospy.loginfo('No point cloud information available to track current object in scene')

                            # if there is point cloud data, we'll try to place a tf
                            # in the object's location
                            else:
                                # this function gives us a generator of points.
                                # we ask for a single point in the center of our object.
                                pc_list = list(pc2.read_points(self._current_pc, skip_nans=True, field_names=('x', 'y', 'z'), uvs=[(x_center, y_center)]))

                                if len(pc_list) > 0:
                                    publish_tf = True
                                    # this is the location of our object in space
                                    tf_id = obj_class + '_' + str(obj_type_index)

                                    # if the user passes a tf prefix, we append it to the object tf name here
                                    if self._tf_prefix is not None:
                                        tf_id = self._tf_prefix + '/' + tf_id

                                    detected_object.tf_id.data = tf_id

                                    point_x, point_y, point_z = pc_list[0]

               

                            for key in self._publishers:
                                # add the object to the unfiltered publisher,
                                # as well as the ones whose filter include this class of objects
                                if key is None or obj_class in self._publishers[key][0]:
                                    msgs[key].detected_objects.append(detected_object)

                            # we'll publish a TF related to this object only once
                            if publish_tf:
                                # kinect here is mapped as camera_link
                                # object tf (x, y, z) must be
                                # passed as (z,-x,-y)
                                object_tf = [point_z, -point_x, -point_y]
                                frame = 'camera_link2'

                                # translate the tf in regard to the fixed frame
                                if self._global_frame is not None:
                                    object_tf = numpy.array(trans) + object_tf
                                    frame = self._global_frame

                                # this fixes #7 on GitHub, when applying the
                                # translation to the tf creates a vector that
                                # RViz just can'y handle
                                if object_tf is not None:
                                    self._tfpub.sendTransform((object_tf), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), tf_id, frame)
                                object_data[obj_class]=object_tf

                    # publish all the messages in their corresponding publishers
                    for key in self._publishers:
                        self._publishers[key][1].publish(msgs[key])
                except CvBridgeError as e:
                    print(e)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)
            if count==1 and variable==1:
                image_popup()
            if count==1 and variable==11:
                image_popup_1()
            if count==1 and variable==2:
                image_popup1()
            if count==1 and variable==3:
                image_popup2()

            if variable is 1 or variable is 11:
                if flag==2 or flag==1:
                    break
            if variable is 2:
                if flag==3 or flag==4:
                    break
            if variable is 3:
                if flag==6:
                    break
            if count==100:
                break





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
def move_anywhere_1(x2, y2):
    while not rospy.is_shutdown():
            if viv>=y2:
                break
            x1=sar
            y1=viv
            theta_goal= math.atan((y2-y1)/(x1-x2))
            e_theta= rar-theta_goal
            velocity_msg.linear.x = 0.6
            velocity_msg.angular.z = (0.5)*e_theta
            pub.publish(velocity_msg)
            rate.sleep()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)


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


def rotate(value,state):
    speed = 0.5
    angle = value
    clockwise = state

    #Converting from angles to radians
    angular_speed = 0.5
    relative_angle = angle*2*PI/360

    #We wont use linear components
    velocity_msg.linear.x=0
    velocity_msg.linear.y=0
    velocity_msg.linear.z=0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        velocity_msg.angular.z = -abs(angular_speed)
    else:
        velocity_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < relative_angle):
        pub.publish(velocity_msg)
        t1 = rospy.Time.now().to_sec()
        #print(t1)
        current_angle = angular_speed*(t1-t0)
    #Forcing our robot to stop
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    rospy.spin()

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



def rotate(value,state):
    speed = 0.5
    angle = value
    clockwise = state

    #Converting from angles to radians
    angular_speed = 0.5
    relative_angle = angle*2*PI/360

    #We wont use linear components
    velocity_msg.linear.x=0
    velocity_msg.linear.y=0
    velocity_msg.linear.z=0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        velocity_msg.angular.z = -abs(angular_speed)
    else:
        velocity_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < relative_angle):
        pub.publish(velocity_msg)
        t1 = rospy.Time.now().to_sec()
        #print(t1)
        current_angle = angular_speed*(t1-t0)
    #Forcing our robot to stop
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    rospy.spin()


# Set Joint Angles
# ---Ur5Moveit()
# Pose of End Effector
# ---Ur5Moveit_info()
# Pre Define Pose
# ---Ur5Moveit_help()
# Going to a pose
# ---Ur5Moveit_helper()


# How to use moveit classes

# 1-Pose of end Effector
# ur53=Ur5Moveit_info()
# while not rospy.is_shutdown():
#     ur53.print_pose_ee()
#     break
# del ur53

# 2-Set Joint Angles
# ur5 = Ur5Moveit()
# pos =  [math.radians(-4),
#           math.radians(-27),
#           math.radians(-24),
#           math.radians(-63),
#           math.radians(-27),
#           math.radians(81)]
# while not rospy.is_shutdown():
#      ur5.set_joint_angles(pos)
#      break
# del ur5


# 3-Going to Pre Define Pose
# ur51 = Ur5Moveit_help()
# while not rospy.is_shutdown():
#     ur51.go_to_predefined_pose("close")
#     break
# del ur51

# 4-Going to a pose
# ur52=Ur5Moveit_helper()
# ur52_pose= geometry_msgs.msg.Pose()
# ur52_pose.position.x = x1+0.016
# ur52_pose.position.y = y1-0.300
# ur52_pose.position.z = 1.021254198886
# ur52_pose.orientation.x = 0.033404070776
# ur52_pose.orientation.y = -0.979504461711
# ur52_pose.orientation.z = 0.198085360143
# ur52_pose.orientation.w = 0.0147433931491
# while not rospy.is_shutdown():
#     ur52.go_to_predefined_pose("ur52_pose")
#     break
# del ur52

x1=0
y1=0
z1=0
x2=0
y2=0
z2=0
x3=0
y3=0
z3=0
c=8
r=8


def coordinates_callback(myMsg):
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


node_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch = roslaunch.parent.ROSLaunchParent(node_uuid, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/bonus_perception.launch"])
roslaunch.configure_logging(node_uuid)

node_uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch1 = roslaunch.parent.ROSLaunchParent(node_uuid1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/bonus_perception.launch"])
roslaunch.configure_logging(node_uuid1)

node_uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch2= roslaunch.parent.ROSLaunchParent(node_uuid2, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/bonus_perception.launch"])
roslaunch.configure_logging(node_uuid2)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid)

uuid_1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch_1 = roslaunch.parent.ROSLaunchParent(uuid_1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid_1)

uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch1 = roslaunch.parent.ROSLaunchParent(uuid1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid1)

uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch2 = roslaunch.parent.ROSLaunchParent(uuid2, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid2)

def image_popup():
    global launch

    launch.start()

def image_popup_1():
    global launch

    launch_1.start()

def image_popup1():
    global launch1

    launch1.start()

def image_popup2():
    global launch2

    launch2.start()

if __name__ == '__main__':
    try:
        rospy.init_node('navigation')
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        rospy.Subscriber('/object_data',Data_msg,coordinates_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub_flag = rospy.Publisher('testing', Int64 , queue_size=1)
        pub_flag1 = rospy.Publisher('testing_1', Int64 , queue_size=1)
        pub_flag2 = rospy.Publisher('testing_2', Int64 , queue_size=1)
        rate = rospy.Rate(10)
        velocity_msg = Twist()
        ur5 = Ur5Moveit()
        ur51 = Ur5Moveit_help()
        ur52 = Ur5Moveit_helper()

# ---------------------Going In Center of Pantry--------------------------------- 
        
        print("Started Run!")       
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

# # ---------------------Scanning For Items---------------------------------

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos)
            break

        Detector().run(1)

        coke="coke"
        glass="glass"


        if coke in object_data.keys() and glass in object_data.keys():
            helper=1
        
        else:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(pos1)
                break

            Detector().run(11)


# ---------------------Going To Store Room----------------------------------

        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros)
            break
        result = movebase_client(13.1,-0.9,1.0)
        rotate_using_lidar_anticlockwise(12.9)
        move(1.5,1)
        result = movebase_client(25.0,-2.6,1.0)
        print("Store Room Reached")

# # # # # # ---------------------Picking Battery----------------------------------

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

        pos2 = [math.radians(90),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]

        while not rospy.is_shutdown():
            ur5.set_joint_angles(store_0)
            break
        Detector().run(3)
        # 12
        print("Glue Identified")
        print("Battery Identified")
        print("Adhesive Identified")

        fpga="fpga_board"
        key1="battery"
        key2="eyfi_board"
        key3="adhesive"
        key4="glue"
        key5="pair_of_wheels"
        def Sort(tup):
            return(sorted(tup, key = lambda x: float(x[1]), reverse = True))
    
        tup = [('fpga_board',float(object_data[fpga][1])), 
                ('battery', float(object_data[key1][1])),
                ('eyfi_board', float(object_data[key2][1])),  
                ('adhesive', float(object_data[key3][1])), 
                ('glue',float(object_data[key4][1])),
                ('pair_of_wheels',float(object_data[key5][1]))] 

        # print(Sort(tup))

        # Glue
        index=-1
        # Battery
        index1=-1
        
        tup1=Sort(tup)

        if tup1[0][0]=="glue":
            index=0
        if tup1[1][0]=="glue":
            index=1
        if tup1[2][0]=="glue":
            index=2
        if tup1[3][0]=="glue":
            index=3
        if tup1[4][0]=="glue":
            index=4
        if tup1[5][0]=="glue":
            index=5

        if tup1[0][0]=="battery":
            index1=0
        if tup1[1][0]=="battery":
            index1=1
        if tup1[2][0]=="battery":
            index1=2
        if tup1[3][0]=="battery":
            index1=3
        if tup1[4][0]=="battery":
            index1=4
        if tup1[5][0]=="battery":
            index1=5


        # For Moving Left
        if index1<=2:
            rotate_using_lidar_clockwise(2.4)
            move(2.45,1)
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left)
                break

        # For Moving Right
        else:
            rotate_using_lidar_clockwise(2.8)
            move(2.5,1)
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_right)
                break


        launch2.shutdown()
        perception_launch.start()

        x3=0
        while x3 is 0:
            flag2_object=Int64()
            flag2_object.data=2+index1
            pub_flag2.publish(flag2_object)

        flag2_object=Int64()
        flag2_object.data=0
        pub_flag2.publish(flag2_object)

        ur52_pose_left= geometry_msgs.msg.Pose()
        ur52_pose_right= geometry_msgs.msg.Pose()
        ur52_pose_final_left = geometry_msgs.msg.Pose()
        ur52_pose_final_right= geometry_msgs.msg.Pose()


# Left Side Orientation

        ur52_pose_left.orientation.x = -0.582391486097
        ur52_pose_left.orientation.y = -0.535701401349
        ur52_pose_left.orientation.z = 0.43839372127
        ur52_pose_left.orientation.w = 0.42621017194

        ur52_pose_final_left.orientation.x = -0.582391486097
        ur52_pose_final_left.orientation.y = -0.535701401349
        ur52_pose_final_left.orientation.z = 0.43839372127
        ur52_pose_final_left.orientation.w = 0.42621017194

# Right Side Orientation
        ur52_pose_right.orientation.x = -0.422645692441
        ur52_pose_right.orientation.y = -0.639800363598
        ur52_pose_right.orientation.z = 0.593623689359
        ur52_pose_right.orientation.w = 0.244206938542

        ur52_pose_final_right.orientation.x = -0.422645692441
        ur52_pose_final_right.orientation.y = -0.639800363598
        ur52_pose_final_right.orientation.z = 0.593623689359
        ur52_pose_final_right.orientation.w = 0.244206938542
        
        ur52_pose_left.position.x = x3-0.05
        ur52_pose_left.position.y = y3+0.03
        ur52_pose_left.position.z = 1.24832900075
        ur52_pose_final_left.position.x = x3-0.05
        ur52_pose_final_left.position.y = y3+0.03
        ur52_pose_final_left.position.z = 1.18432900075


        ur52_pose_right.position.x = x3-0.05
        ur52_pose_right.position.y = y3+0.03
        ur52_pose_right.position.z = 1.24832900075
        ur52_pose_final_right.position.x = x3-0.05
        ur52_pose_final_right.position.y = y3+0.03
        ur52_pose_final_right.position.z = 1.18432900075


        if index1==0:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left_pick_before)
                ur52.go_to_pose(ur52_pose_left)
                ur52.go_to_pose(ur52_pose_final_left)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_left)
                break

        elif index1<=2:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left_pick_before)
                ur52.go_to_pose(ur52_pose_left)
                ur52.go_to_pose(ur52_pose_final_left)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_left)
                break

        else:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_right_pick_before)
                if index==4:
                    ur5.set_joint_angles(store_right_pick_before_index_4)
                ur52.go_to_pose(ur52_pose_right)
                ur52.go_to_pose(ur52_pose_final_right)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_right)
                break

        # 5
        print("Battery Picked")


# ---------------------Going In Research Lab----------------------------------

        move(2.0,0)
        rotate_using_lidar_clockwise(17.6)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos2)
            break

        result = movebase_client(10.7,9.4,1.0)

        # 6
        print("Research Lab Reached")

# ---------------------Dropping the Battery----------------------------------


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

        pos3 = [math.radians(-90),
            math.radians(-27),
            math.radians(-24),
            math.radians(-63),
            math.radians(-27),
            math.radians(81)]

        pos4 = [math.radians(-8),
                math.radians(36),
                math.radians(-96),
                math.radians(-123),
                math.radians(-77),
                math.radians(169)]

        pos5 = [math.radians(-50),
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

        # 7
        print("Battery Dropped in DropBox3")


# ---------------------Going To Store room---------------------------------
        
        pos5 = [math.radians(-50),
                math.radians(-19),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]

        rotate_using_lidar_clockwise(8.8)

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos5)
            break
        result = movebase_client(13.1,1.05,1.0)
        result = movebase_client(25.0,-2.6,1.0)

        while not rospy.is_shutdown():
            ur5.set_joint_angles(store_0)
            break

                
# For Moving Left
        if index<=2:
            rotate_using_lidar_clockwise(2.4)
            move(2.45,1)
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left)
                break

        # For Moving Right
        else:
            rotate_using_lidar_clockwise(2.8)
            move(2.5,1)
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_right)
                break

        if index1<=2 and index<=2:
            if index1<index:
                index=index-1
        elif index1>=3 and index>=3:
            if index1<index:
                index=index-1

        print(index)

        perception_launch2.start()
        launch2.shutdown()

        x3=0
        while x3 is 0:
            flag2_object=Int64()
            flag2_object.data=2+index
            pub_flag2.publish(flag2_object)

        flag2_object=Int64()
        flag2_object.data=0
        pub_flag2.publish(flag2_object)


        ur52_pose_left= geometry_msgs.msg.Pose()
        ur52_pose_left_1= geometry_msgs.msg.Pose()
        ur52_pose_right= geometry_msgs.msg.Pose()
        ur52_pose_final_left = geometry_msgs.msg.Pose()
        ur52_pose_final_left_1 = geometry_msgs.msg.Pose()
        ur52_pose_final_right= geometry_msgs.msg.Pose()


        ur52_pose_left_1.position.x = x3-0.05
        ur52_pose_left_1.position.y = y3+0.03
        ur52_pose_left_1.position.z = 1.26832900075
        ur52_pose_final_left_1.position.x = x3-0.05
        ur52_pose_final_left_1.position.y = y3+0.03
        ur52_pose_final_left_1.position.z = 1.20432900075

        ur52_pose_left_1.orientation.x = -0.675090862175
        ur52_pose_left_1.orientation.y = -0.409646897222
        ur52_pose_left_1.orientation.z = 0.275639066023
        ur52_pose_left_1.orientation.w = 0.548146743752

        ur52_pose_final_left_1.orientation.x = -0.675090862175
        ur52_pose_final_left_1.orientation.y = -0.409646897222
        ur52_pose_final_left_1.orientation.z = 0.275639066023
        ur52_pose_final_left_1.orientation.w = 0.548146743752

# Left Side Orientation

        ur52_pose_left.orientation.x = -0.675090862175
        ur52_pose_left.orientation.y = -0.409646897222
        ur52_pose_left.orientation.z = 0.275639066023
        ur52_pose_left.orientation.w = 0.548146743752

        ur52_pose_final_left.orientation.x = -0.675090862175
        ur52_pose_final_left.orientation.y = -0.409646897222
        ur52_pose_final_left.orientation.z = 0.275639066023
        ur52_pose_final_left.orientation.w = 0.548146743752

# Right Side Orientation

        ur52_pose_right.orientation.x = -0.57128583654
        ur52_pose_right.orientation.y = -0.559533050606
        ur52_pose_right.orientation.z = 0.450785262963
        ur52_pose_right.orientation.w = 0.396671028616

        ur52_pose_final_right.orientation.x = -0.57128583654
        ur52_pose_final_right.orientation.y = -0.559533050606
        ur52_pose_final_right.orientation.z = 0.450785262963
        ur52_pose_final_right.orientation.w = 0.396671028616
        
        ur52_pose_left.position.x = x3-0.05-0.006
        ur52_pose_left.position.y = y3+0.03-0.020
        ur52_pose_left.position.z = 1.26832900075
        ur52_pose_final_left.position.x = x3-0.05-0.006
        ur52_pose_final_left.position.y = y3+0.03-0.025
        ur52_pose_final_left.position.z = 1.21432900075


        ur52_pose_right.position.x = x3-0.05-0.02
        ur52_pose_right.position.y = y3+0.03-0.01
        ur52_pose_right.position.z = 1.26832900075
        ur52_pose_final_right.position.x = x3-0.05-0.02
        ur52_pose_final_right.position.y = y3+0.03-0.01
        ur52_pose_final_right.position.z = 1.21432900075


        if index<=2:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left_pick_before)
                ur52.go_to_pose(ur52_pose_left)
                ur52.go_to_pose(ur52_pose_final_left)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_left)
                break
        else:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_right_pick_before)
                if index==3:
                    ur5.set_joint_angles(store_right_pick_before_index_4)
                ur52.go_to_pose(ur52_pose_right)
                ur52.go_to_pose(ur52_pose_final_right)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_right)
                break
        # 13
        print("Glue Picked")


# ---------------------Going In Meeting Room----------------------------------
        
        move(2.0,0)
        rotate_using_lidar_clockwise(17.6)
        left_pick_before = [math.radians(-99),
                              math.radians(-30),
                              math.radians(-19),
                              math.radians(-74),
                              math.radians(-90),
                              math.radians(90)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(left_pick_before)
            break

        result = movebase_client(6.9,2.4,0.5)
        print("Meeting Room Reached")

# ---------------------Dropping The Glue----------------------------------
        
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
        print("Glue Dropped in DropBox2")

# ---------------------Going To pick Adhesive----------------------------------


        pos_origin = [math.radians(-30),
                math.radians(-25),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]
        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos_origin)
            break


        result = movebase_client(6.9,2.4,0.5)
        move(5.7,0)
        rotate_using_lidar_anticlockwise(2.5)
        move(4.2,1)
        rotate_using_lidar_clockwise(3.0)

        result = movebase_client(5.70,5.0,1.0)

        pos_tensor = [math.radians(-180),
                math.radians(-16),
                math.radians(-21),
                math.radians(-66),
                math.radians(-25),
                math.radians(69)]

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos_tensor)
            break
        Detector().run(2)

        gl="glue"
        key11="battery"
        key22="adhesive"
        def Sort(tup):
            return(sorted(tup, key = lambda x: float(x[1])))
    
        tup_glue = [('glue',float(object_data[gl][1])), 
                ('battery', float(object_data[key11][1])),  
                ('adhesive', float(object_data[key22][1]))] 

        ind=-1
        tup1_glue=Sort(tup_glue)
        if tup1_glue[0][0]=="adhesive":
            ind=1
        if tup1_glue[1][0]=="adhesive":
            ind=2
        if tup1_glue[2][0]=="adhesive":
            ind=3


        perception_launch1.start()

        x2=0
        while x2 is 0:
            flag1_object=Int64()
            flag1_object.data=1+ind
            pub_flag1.publish(flag1_object)


        flag1_object=Int64()
        flag1_object.data=0
        pub_flag1.publish(flag1_object)

        launch1.shutdown()

        ur52_pose_4 = geometry_msgs.msg.Pose()
        ur52_pose_5 = geometry_msgs.msg.Pose()
        ur52_pose_4.position.x = x2-0.014
        ur52_pose_4.position.y = y2+0.245
        ur52_pose_4.position.z = 1.069192397391


        ur52_pose_4.orientation.x = -0.977797789392
        ur52_pose_4.orientation.y = -0.0311517182184
        ur52_pose_4.orientation.z = 0.0423739397005
        ur52_pose_4.orientation.w = 0.202843542533

        ur52_pose_5.position.x = x2-0.014
        ur52_pose_5.position.y = y2+0.195
        ur52_pose_5.position.z = 1.069192397391

        ur52_pose_5.orientation.x = -0.977797789392
        ur52_pose_5.orientation.y = -0.0311517182184
        ur52_pose_5.orientation.z = 0.0423739397005
        ur52_pose_5.orientation.w = 0.202843542533


        while not rospy.is_shutdown():
            ur52.go_to_pose(ur52_pose_4)
            ur52.go_to_pose(ur52_pose_5)
            ur51.go_to_predefined_pose("mid5")
            rospy.sleep(1)
            ur52.go_to_pose(ur52_pose_4)
            ur5.set_joint_angles(pos_tensor)
            # ur5.set_joint_angles(left_pick_before)
            break


# ---------------------Going To conference Room----------------------------------

        result = movebase_client(5.70,5.0,1.0)
        move(3.8,0)
        rotate_using_lidar_clockwise(6)
        move(1,1)
        rotate_using_lidar_clockwise(2.7)
        move(3.0,1)
        result = movebase_client(8.3,2.4,0.5)
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
        result = movebase_client(11.2,1.2,1.0)

        left_pick_before = [math.radians(-99),
                              math.radians(-30),
                              math.radians(-19),
                              math.radians(-74),
                              math.radians(-90),
                              math.radians(90)]

        while not rospy.is_shutdown():
            ur5.set_joint_angles(left_pick_before)
            break
        rotate_using_lidar_anticlockwise(100)
        result = movebase_client(5.4,-0.6,1.0)
        print("Conference Room Reached")


# ---------------------Dropping the Adhesive----------------------------------



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
        print("Adhesive Dropped in DropBox1")




# ---------------------Going To origin----------------------------------


        con_room = [math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]
        rotate_using_lidar_anticlockwise(0.8)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(con_room)
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
