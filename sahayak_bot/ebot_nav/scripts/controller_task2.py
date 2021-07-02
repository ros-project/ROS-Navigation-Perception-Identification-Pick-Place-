#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
def Waypoints(t):
    if t == 0:
        h = -9.1
        k = -1.2
    elif t == 1:
        h = 10.7
        k = 10.5
    elif t == 2:
        h = 13.1
        k = -1.4
    elif t == 3:
        h = 18.2
        k = -1.4
    elif t == 4:
        h = -2
        k = 4.0
    return [h,k]
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

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.loginfo("Reached Goal 0")
        [a,b]=Waypoints(0)
        result = movebase_client(a,b)
        result1=False
        result2=False
        result3=False
        result4=False
        if result:
            rospy.loginfo("Reached Goal 1")
            [a,b]=Waypoints(1)
            result1 = movebase_client(a,b)
        if result1:
            rospy.loginfo("Reached Goal 2")
            [a,b]=Waypoints(2)
            result2 = movebase_client(a,b)
        if result2:
            rospy.loginfo("Reached Goal 3")
            [a,b]=Waypoints(3)
            result3 = movebase_client(a,b)
        if result3:
            rospy.loginfo("Reached Goal 4")
            [a,b]=Waypoints(4)
            result4= movebase_client(a,b)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

