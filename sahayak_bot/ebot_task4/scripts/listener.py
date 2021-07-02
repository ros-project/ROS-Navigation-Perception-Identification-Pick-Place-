import rospy
from ebot_task4.msg import legends


def func_callback_topic_my_topic(myMsg):

    rospy.loginfo(myMsg.a)

def main():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("legends", legends, func_callback_topic_my_topic)

    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

