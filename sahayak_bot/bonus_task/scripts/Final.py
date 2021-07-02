#!/usr/bin/env python

# All header Files for Navigation
import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2
from ebot_task4.msg import legends
import math
from task5_msgs.msg import Data_msg
from task5_msgs.msg import Flag_msg
from std_msgs.msg import Int64
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

# Passing Navigation Points
def Waypoints(t):
    if t == 0:
        h = 13.1
        k = -0.9
    elif t == 1:
        h = 26.01
        k = -2.89
    elif t == 2:
        h = 6.9
        k = 2.5
    return [h,k]




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

        self._global_frame = "base_link"
        self._tf_prefix = rospy.get_name()

        # create a transform listener so we get the fixed frame the user wants
        # to publish object tfs related to
        self._tf_listener = tf.TransformListener()

        if detector_type == 'ssd':
            rospy.loginfo('Chosen detector type: Single Shot Detector')
            if len(frozen_graph) == 0:
                raise ValueError('Parameter \'frozen_graph\' must be passed')
            if len(label_map) == 0:
                raise ValueError('Parameter \'label_map\' must be passed')
            if confidence <= 0 or confidence > 1:
                raise ValueError('Parameter \'confidence\' must be between 0 and 1')

            frozen_graph = expanduser(frozen_graph)
            label_map = expanduser(label_map)

            self._detector = SingleShotDetector(frozen_graph, label_map, confidence=confidence)
            rospy.loginfo('Path to inference graph: ' + frozen_graph)
            rospy.loginfo('Path to label map: ' + label_map)

            # count number of classes from label map
            label_map_contents = open(label_map, 'r').read()
            num_classes = label_map_contents.count('name:')
            rospy.loginfo('Number of classes: ' + str(num_classes))

        elif detector_type in ['sift', 'rootsift']:
            rospy.loginfo('Chosen detector type: Keypoint Object Detector')
            if min_points <= 0:
                raise ValueError('Parameter \'min_points\' must greater than 0')
            if len(database_path) == 0:
                raise ValueError('Parameter \'database_path\' must be passed')

            database_path = expanduser(database_path)

            detector_type = 'SIFT' if detector_type == 'sift' else 'RootSIFT'
            self._detector = KeypointObjectDetector(database_path, detector_type, min_points=min_points)
            rospy.loginfo('Database path: ' + database_path)
            rospy.loginfo('Min. points: ' + str(min_points))

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
        self._imagepub = rospy.Publisher('~labeled_image', Image, queue_size=10)

        # this package works with a dynamic list of publishers
        # if no filter is configured via parameters to the package,
        # one default, unfiltered publisher will publish every object
        if len(filters) == 0:
            rospy.loginfo('No filter configured, publishing every detected object in a single topic')
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
                    rospy.loginfo('Created topic for filter [' + key + ']')

        self._tfpub = tf.TransformBroadcaster()
        rospy.loginfo('Ready to detect!')

    def image_callback(self, image):
        """Image callback"""
        # Store value on a private attribute
        self._current_image = image

    def pc_callback(self, pc):
        """Point cloud callback"""
        # Store value on a private attribute
        self._current_pc = pc

    def run(self):
        # run while ROS runs
        count=0
        while not rospy.is_shutdown():
            # only run if there's an image present
            count=count+1
            flag=0
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
                        flag=1
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
                                frame = self._global_frame

                                # translate the tf in regard to the fixed frame
                                if self._global_frame is not None:
                                    object_tf = numpy.array(trans) + object_tf
                                    frame = "base_link"

                                # this fixes #7 on GitHub, when applying the
                                # translation to the tf creates a vector that
                                # RViz just can'y handle
                                if object_tf is not None:
                                    self._tfpub.sendTransform((object_tf), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), tf_id, frame)
                                object_data[obj_class]=object_tf

                                arr=["coke", "battery" , "glue"]
                                for key in arr:
                                    if key in object_data.keys():
                                        print(key)
                                        print(object_data[key])

                    # publish all the messages in their corresponding publishers
                    for key in self._publishers:
                        self._publishers[key][1].publish(msgs[key])
                except CvBridgeError as e:
                    print(e)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)
            rospy.loginfo(count)
            if flag==1:
                break
            if count==1000:
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


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit_info init done." + '\033[0m')

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

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit_info Deleted." + '\033[0m')





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
        'front':  min(min(msg.ranges[288:431]), 100),
        'fleft':  min(min(msg.ranges[432:575]), 100),
        'left':   min(min(msg.ranges[576:713]), 100),
    }
    # print(regions['front'])



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
        velocity_msg.linear.x =0.5
    else:
        velocity_msg.linear.x =-0.5
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



def movebase_client(x1,y1):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =x1
    goal.target_pose.pose.position.y =y1
    goal.target_pose.pose.orientation.w = 1.0
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
# ur52_pose.position.x = x1-0.018
# ur52_pose.position.y = y1-0.018
# ur52_pose.position.z = 1.121254198886
# ur52_pose.orientation.x = -0.305329878462
# ur52_pose.orientation.y = -0.618070624129
# ur52_pose.orientation.z = 0.641678006177
# ur52_pose.orientation.w = 0.336172136406
# while not rospy.is_shutdown():
#     ur52.go_to_predefined_pose("ur52_pose")
#     break
# del ur52

  # x: -0.305329878462
  # y: -0.618070624129
  # z: 0.641678006177
  # w: 0.336172136406


def define_var():
    global x1,y1,z1
    x1=-1
    y1=-1
    z1=-1


def func_callback(myMsg):
    global x1,y1,z1
    x1=myMsg.x1
    y1=myMsg.y1
    z1=myMsg.z1

x1=-1
y1=-1
z1=-1
x2=-1
y2=-1
z2=-1
x3=-1
y3=-1
z3=-1
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

if __name__ == '__main__':
    try:
        rospy.init_node('dodo_detector')
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        rospy.Subscriber('/object_data',Data_msg,coordinates_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub_flag = rospy.Publisher('testing', Int64 , queue_size=1)
        rate = rospy.Rate(10)
        velocity_msg = Twist()
        define_var()

        # [a,b]=Waypoints(0)
        # result = movebase_client(14.65,-0.9)
        # rospy.loginfo("Reached Center of Pantry")

        ur5 = Ur5Moveit()
        ur52 = Ur5Moveit_helper()
        ur51 = Ur5Moveit_help()

        # # Going Infront of Pantry 
        # result = movebase_client(13.1,1.05)
        # rospy.loginfo("Done Part 1")
        # # Rotating To Go Inside
        # rotate_using_lidar_clockwise(3.3)
        # rospy.loginfo("Done Part 2")
        # # Going inside the pantry
        # result = movebase_client(13.1,-0.9)
        # rospy.loginfo("Done Part 3")


        pos1 = [math.radians(89),
                math.radians(-27),
                math.radians(-24),
                math.radians(-63),
                math.radians(-14),
                math.radians(81)]

        # while not rospy.is_shutdown():
            # ur5.set_joint_angles(pos1)
            # break

        right_pick_before = [math.radians(122),
                              math.radians(-3),
                              math.radians(-50),
                              math.radians(-66),
                              math.radians(-92),
                              math.radians(57)]
        
        pantry_point_cloud = [math.radians(108),
                              math.radians(37),
                              math.radians(-60),
                              math.radians(-13),
                              math.radians(-48),
                              math.radians(6)]

        pick_before = [math.radians(-99),
                      math.radians(-30),
                      math.radians(-19),
                      math.radians(-74),
                      math.radians(-90),
                      math.radians(90)]

        pos2= [math.radians(-90),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0),
                  math.radians(0)]

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
        
        ur52_pose_left.position.x = 0.342382-0.05
        ur52_pose_left.position.y = -0.366521+0.03
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



        while not rospy.is_shutdown():
            ur52.go_to_pose(ur52_pose_left)
            break


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

