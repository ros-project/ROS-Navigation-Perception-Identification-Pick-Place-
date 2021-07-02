#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

sar=0
viv=0
rar=0
regions = {
        'right':  0,
        'fright': 0,
        'front':  0,
        'fleft':  0,
        'left':   0,
    }

def odom_callback(data):
    global sar,viv,rar
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    sar=data.pose.pose.position.x
    viv=data.pose.pose.position.y
    rar=euler_from_quaternion([x,y,z,w])[2]

def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[:143]), 100),
        'fright': min(min(msg.ranges[144:287]), 100),
        'front':  min(min(msg.ranges[288:431]), 100),
        'fleft':  min(min(msg.ranges[432:575]), 100),
        'left':   min(min(msg.ranges[576:713]), 100),
    }
    print(regions['right'])
    print(regions['fright'])
    print(regions['front'])
    print(regions['fleft'])
    print(regions['left'])

def Waypoints(t):
    if t == 0:
        h = 0.74
        k = 0.488
    elif t == 1:
        h = 1.42
        k = 1.289  
    elif t == 2:
        h = 1.911
        k = 1.54
    elif t == 3:
        h = 2.45
        k = 1.2
    elif t == 4:
        h = 3.141
        k = 0
    elif t == 5:
        h = 3.91
        k = -1.289
    elif t == 6:
        h = 4.373
        k = -1.54
    elif t == 7:
        h = 5.02
        k = -1.125
    elif t == 8:
        h = 5.72
        k = -0.297
    elif t == 9:
        h = 6.283
        k = 0
    elif t == 10:
        h = 7.3
        k = 0
    elif t == 11:
        h = 7.5
        k = 0
    else:
        pass
 
    return [h,k]

def control_loop():
    rospy.init_node('ebot_controller',anonymous=True)
   
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
   
    rate = rospy.Rate(10)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    for i in range(0,12,1):
        [x2,y2]=Waypoints(i)
        while not rospy.is_shutdown():
            if sar>=x2:
                break
            x1=sar
            y1=viv
            theta_goal= math.atan((y2-y1)/(x2-x1))
            e_theta= rar-theta_goal
            velocity_msg.linear.x = 0.2
            velocity_msg.angular.z = (-1)*e_theta
            pub.publish(velocity_msg)
            print("Controller message pushed at {}".format(rospy.get_time()))
            rate.sleep()
            if i == 11:
                break
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    linear_x = 0 
    angular_z = 0
    while not rospy.is_shutdown():
        if regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            linear_x = 0.6
            angular_z = 0
        elif regions['front'] < 1 and regions['fleft'] <1 and regions['fright'] > 1:
            linear_x = 0
            angular_z = -1
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            linear_x = 0
            angular_z = -1
        elif regions['front'] <1 and regions['fleft'] < 1 and regions['fright'] > 1:
            linear_x = 0
            angular_z = 1
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            linear_x = 0
            angular_z = -1
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            linear_x = 0
            angular_z = 1
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            linear_x = 0
            angular_z = -1
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            linear_x = 0
            angular_z = -1

        x1=sar
        y1=viv
        [x2,y2]=[7.31,-1]
        theta_goal= math.atan((y2-y1)/(x2-x1))
        e_theta= rar-theta_goal
        linear_x= 0.2
        angular_z= (-1)*e_theta
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
        velocity_msg.linear.x = linear_x
        velocity_msg.angular.z =angular_z
        pub.publish(velocity_msg)

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
