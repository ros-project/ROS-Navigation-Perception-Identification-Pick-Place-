import rospy
from ebot_task4.msg import legends

def main():
    pub = rospy.Publisher('legends', legends, queue_size=10)

    # 2. Initializes the ROS node for the process.
    rospy.init_node('talker', anonymous=True)

    # 3. Set the Loop Rate
    r = rospy.Rate(1) # 1 Hz : Loop will its best to run 1 time in 1 second

    # 4. Write the infinite Loop
    while not rospy.is_shutdown():
        obj_msg = legends()

        obj_msg.a=12.123
        obj_msg.b=123123.123123

        rospy.loginfo("Publishing: ")
        rospy.loginfo(obj_msg)

        pub.publish(obj_msg)

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
