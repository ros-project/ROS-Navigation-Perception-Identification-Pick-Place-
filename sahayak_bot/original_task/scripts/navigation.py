#!/usr/bin/env python

''' Our Custom Trained Model Drive link-
https://drive.google.com/drive/folders/13qFv1eiPHBcCQA6BYXUfVsxauSY7FIZU?usp=sharing'''

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

# We have Used Dodo_Detector_Ros package
# We have taken out script from there and have directly used it as a class in our code for Object Detection.
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
                if flag==2:
                    break
            if variable is 2:
                if flag==2:
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
def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[:143]), 100),
        'fright': min(min(msg.ranges[144:287]), 100),
        'front':  min(min(msg.ranges[358:361]), 100),
        'fleft':  min(min(msg.ranges[432:575]), 100),
        'left':   min(min(msg.ranges[576:713]), 100),
    }

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

# Callback function for storing object coordinates.
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

# This node launches a task5_perception launch file
node_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch = roslaunch.parent.ROSLaunchParent(node_uuid, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/original_perception.launch"])
roslaunch.configure_logging(node_uuid)

# This node launches a task5_perception launch file
node_uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch1 = roslaunch.parent.ROSLaunchParent(node_uuid1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/original_perception.launch"])
roslaunch.configure_logging(node_uuid1)

# This node launches a task5_perception launch file
node_uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
perception_launch2= roslaunch.parent.ROSLaunchParent(node_uuid2, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/original_perception.launch"])
roslaunch.configure_logging(node_uuid2)

# This node launches a rqt image view node which is written inside some.launch file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid)

# This node launches a rqt image view node which is written inside some.launch file
uuid_1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch_1 = roslaunch.parent.ROSLaunchParent(uuid_1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid_1)

# This node launches a rqt image view node which is written inside some.launch file
uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch1 = roslaunch.parent.ROSLaunchParent(uuid1, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid1)

# This node launches a rqt image view node which is written inside some.launch file
uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch2 = roslaunch.parent.ROSLaunchParent(uuid2, ["/home/sarthak/catkin_ws/src/sahayak_bot/ebot_description/launch/some.launch"])
roslaunch.configure_logging(uuid2)

# This function start the launch file
def image_popup():
    global launch

    launch.start()

# This function start the launch file
def image_popup_1():
    global launch

    launch_1.start()

# This function start the launch file
def image_popup1():
    global launch1

    launch1.start()

# This function start the launch file
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

        # It publishes flag value which helps in determining the object's coordinates in PCL script.
        pub_flag = rospy.Publisher('testing', Int64 , queue_size=1)
        pub_flag1 = rospy.Publisher('testing_1', Int64 , queue_size=1)
        pub_flag2 = rospy.Publisher('testing_2', Int64 , queue_size=1)

        rate = rospy.Rate(10)
        velocity_msg = Twist()
        ur5 = Ur5Moveit()
        ur51 = Ur5Moveit_help()
        ur52 = Ur5Moveit_helper()

        # 1
        print("Started Run!")
# ---------------------Going In Meeting Room----------------------------------
        result = movebase_client(8.6,1.24,1.0)
        rotate_using_lidar_anticlockwise(4.1)
        move(1,1)
        result = movebase_client(7.55,2.35,0.5)

        # 2
        print("Meeting Room Reached")

# ---------------------Meeting Room Detection----------------------------------

        pos_tensor = [math.radians(-10),
                math.radians(-16),
                math.radians(-21),
                math.radians(-66),
                math.radians(-28),
                math.radians(69)]

        while not rospy.is_shutdown():
            ur5.set_joint_angles(pos_tensor)
            break

        Detector().run(2)

# ---------------------Going To Store Room----------------------------------
        
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
        result = movebase_client(25.0,-2.6,1.0)

        # 3
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

        # 4
        print("Battery Identified")

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

        index=-1
        tup1=Sort(tup)
        if tup1[0][0]=="battery":
            index=1
        if tup1[1][0]=="battery":
            index=2
        if tup1[2][0]=="battery":
            index=3
        if tup1[3][0]=="battery":
            index=4
        if tup1[4][0]=="battery":
            index=5
        if tup1[5][0]=="battery":
            index=6

        # For Moving Left
        if object_data[key1][1]>=-0.3:
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

        perception_launch2.start()
        launch2.shutdown()

        # Publishing Flag

        x3=0
        while x3 is 0:
            flag2_object=Int64()
            flag2_object.data=1+index
            pub_flag2.publish(flag2_object)

        flag2_object=Int64()
        flag2_object.data=0
        pub_flag2.publish(flag2_object)
        

        ur52_pose_left= geometry_msgs.msg.Pose()
        ur52_pose_right= geometry_msgs.msg.Pose()
        ur52_pose_final_left = geometry_msgs.msg.Pose()
        ur52_pose_final_right= geometry_msgs.msg.Pose()


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


        if index<=3:
            while not rospy.is_shutdown():
                ur5.set_joint_angles(store_left_pick_before)
                ur52.go_to_pose(ur52_pose_left)
                ur52.go_to_pose(ur52_pose_final_left)
                ur51.go_to_predefined_pose("mid5")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose_left)
                break

        if index>=4:
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

        pos4 = [math.radians(0),
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

# ---------------------Going to Pantry Room----------------------------------

        all_zeros = [math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]
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

        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros)
            break

        rotate_using_lidar_clockwise(3.3)
        move(1,1)
        result = movebase_client(13.1,-0.9,1.0)
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
        Detector().run(1)

        coke="coke"
        glass="glass"
        result = movebase_client(13.1,-0.9,1.0)
        x1=0

# ---------------------Picking The Coke From Left Table & taking to Center---------------------------------
        if coke in object_data.keys() and glass in object_data.keys():

            # 3
            print("Coke Identified")

            result = movebase_client(14.65,-0.9,1.0) 
            launch.shutdown()
            while not rospy.is_shutdown():
                ur5.set_joint_angles(left_point_cloud)
                break
            result = movebase_client(14.65,-0.9,1.0) 
            perception_launch.start()

            # Publishing Flag 
            
            x1=0
            while x1 is 0:
                flag_object=Int64()
                flag_object.data=1
                pub_flag.publish(flag_object)

            # some_count=0
            # while some_count<100:
            flag_object=Int64()
            flag_object.data=0
            pub_flag.publish(flag_object)
                # some_count=some_count+1

            while not rospy.is_shutdown():
                ur5.set_joint_angles(left_pick_before)
                break

            result = movebase_client(14.65,-0.9,1.0) 
            
            ur52_pose= geometry_msgs.msg.Pose()
            ur52_pose_final = geometry_msgs.msg.Pose()

            ur52_pose.position.x = x1+0.026
            ur52_pose.position.y = y1+0.012+0.01
            ur52_pose.position.z = 1.151254198886

            # Down Side Hand orientation

            ur52_pose.orientation.x = -0.658432725945
            ur52_pose.orientation.y = -0.283516159515
            ur52_pose.orientation.z = 0.303956753806
            ur52_pose.orientation.w = 0.627451372231


            ur52_pose_final.position.x = x1+0.026
            ur52_pose_final.position.y = y1+0.012+0.01
            ur52_pose_final.position.z = 1.081254198886

            # Down Side Hand orientation

            ur52_pose_final.orientation.x = -0.658432725945
            ur52_pose_final.orientation.y = -0.283516159515
            ur52_pose_final.orientation.z = 0.303956753806
            ur52_pose_final.orientation.w = 0.627451372231

            while not rospy.is_shutdown():
                ur52.go_to_pose(ur52_pose)
                ur52.go_to_pose(ur52_pose_final)
                ur51.go_to_predefined_pose("mid3")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose)
                ur5.set_joint_angles(left_pick_before)
                break

            # 4
            print("Coke Picked")

            result = movebase_client(14.65,-0.9,1.0) 

            move(3.5,0)
            result = movebase_client(13.1,-0.9,1.0) 

        elif coke in object_data.keys():

            # 3
            print("Coke Identified")

            result = movebase_client(14.65,-0.9,1.0) 
            launch.shutdown()
            while not rospy.is_shutdown():
                ur5.set_joint_angles(left_point_cloud)
                break
            result = movebase_client(14.65,-0.9,1.0) 
            perception_launch.start()

            # Publishing Flag 

            x1=0
            while x1 is 0:
                flag_object=Int64()
                flag_object.data=1
                pub_flag.publish(flag_object)

            # some_count=0
            flag_object=Int64()
            flag_object.data=0
            pub_flag.publish(flag_object)
                # some_count=some_count+1

            while not rospy.is_shutdown():
                ur5.set_joint_angles(left_pick_before)
                break

            result = movebase_client(14.65,-0.9,1.0) 
            
            ur52_pose= geometry_msgs.msg.Pose()
            ur52_pose_final = geometry_msgs.msg.Pose()

            ur52_pose.position.x = x1+0.026
            ur52_pose.position.y = y1+0.012+0.01
            ur52_pose.position.z = 1.151254198886

            # Down Side Hand orientation

            ur52_pose.orientation.x = -0.658432725945
            ur52_pose.orientation.y = -0.283516159515
            ur52_pose.orientation.z = 0.303956753806
            ur52_pose.orientation.w = 0.627451372231


            ur52_pose_final.position.x = x1+0.026
            ur52_pose_final.position.y = y1+0.012+0.01
            ur52_pose_final.position.z = 1.071254198886

            # Down Side Hand orientation

            ur52_pose_final.orientation.x = -0.658432725945
            ur52_pose_final.orientation.y = -0.283516159515
            ur52_pose_final.orientation.z = 0.303956753806
            ur52_pose_final.orientation.w = 0.627451372231

            while not rospy.is_shutdown():
                ur52.go_to_pose(ur52_pose)
                ur52.go_to_pose(ur52_pose_final)
                ur51.go_to_predefined_pose("mid3")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose)
                ur5.set_joint_angles(left_pick_before)
                break
            

            # 4
            print("Coke Picked")

            result = movebase_client(14.65,-0.9,1.0) 

            move(3.5,0)
            result = movebase_client(13.1,-0.9,1.0) 


# # ---------------------Picking The Coke From Right Table & taking to Center----------------------------
        else:

            while not rospy.is_shutdown():
                ur5.set_joint_angles(pos1)
                break
            
            Detector().run(11)

            # 3
            print("Coke Identified")

            move(4.0,0)
            
            launch_1.shutdown()
            launch.shutdown()

            result = movebase_client(11.15,-0.9,1.0)

            while not rospy.is_shutdown():
                ur5.set_joint_angles(right_point_cloud)
                break

            result = movebase_client(11.15,-0.9,1.0) 
            perception_launch.start()

            # Publishing Flag 

            while x1 is 0:
                flag_object=Int64()
                flag_object.data=1
                pub_flag.publish(flag_object)

            flag_object=Int64()
            flag_object.data=0
            pub_flag.publish(flag_object)

            while not rospy.is_shutdown():
                ur5.set_joint_angles(right_pick_before)
                break


            ur52_pose= geometry_msgs.msg.Pose()
            ur52_pose_final= geometry_msgs.msg.Pose()

            ur52_pose.position.x =x1+0.026-0.033
            ur52_pose.position.y = y1+0.012-0.025
            ur52_pose.position.z = 1.151254198886

            # Down Side Hand orientation

            ur52_pose.orientation.x = -0.65261539674
            ur52_pose.orientation.y = -0.245263681051
            ur52_pose.orientation.z = 0.269674799438
            ur52_pose.orientation.w = 0.664239695625

            ur52_pose_final.position.x = x1+0.026-0.033
            ur52_pose_final.position.y = y1+0.012-0.025
            ur52_pose_final.position.z = 1.081254198886

            # Down Side Hand orientation

            ur52_pose_final.orientation.x = -0.65261539674
            ur52_pose_final.orientation.y = -0.245263681051
            ur52_pose_final.orientation.z = 0.269674799438
            ur52_pose_final.orientation.w = 0.664239695625


            result = movebase_client(11.15,-0.9,1.0)

            while not rospy.is_shutdown():
                ur52.go_to_pose(ur52_pose)
                ur52.go_to_pose(ur52_pose_final)
                ur51.go_to_predefined_pose("mid3")
                rospy.sleep(1)
                ur52.go_to_pose(ur52_pose)
                ur5.set_joint_angles(left_pick_before)
                break 


            # 4
            print("Coke Picked")

            result = movebase_client(13.1,-0.9,1.0) 


# ---------------------Going To Conference Room[Drop Box 1]-------------------------------
        rotate_using_lidar_anticlockwise(13)
        move(2.5,1)

        result = movebase_client(5.4,-0.6,1.0)

        # 8
        print("Conference Room Reached")

        all_zeros = [math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]
        all_zeros_1 = [math.radians(-30),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]
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

        # 9
        print("Coke Dropped in DropBox1")
# ---------------------Going to Origin----------------------------------
        
        rotate_using_lidar_anticlockwise(0.8)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros_1)
            break

        move(0.7,1)
        rotate_using_lidar_anticlockwise(2.5)
        move(2.8,1)
        rotate_using_lidar_anticlockwise(17.6)
        move(15,1)
        rotate_using_lidar_anticlockwise(7.1)
        move(1,1)
        rotate_using_lidar_anticlockwise(3.9)
        while not rospy.is_shutdown():
            ur5.set_joint_angles(all_zeros)
            break
        result = movebase_client(0,0,1.0)

        # 10
        print("Mission Accomplished!")

        del ur5
        del ur51
        del ur52

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
