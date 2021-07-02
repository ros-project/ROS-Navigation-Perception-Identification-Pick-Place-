import rospy
from object_msgs.msg import ObjectPose


def func_callback_topic_my_topic(myMsg):

    x=myMsg.x1;
    rospy.loginfo(x)


def main():

    # 1. Initialize the Subscriber Node.
    rospy.init_node('some', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("/detection_info", ObjectPose, func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
