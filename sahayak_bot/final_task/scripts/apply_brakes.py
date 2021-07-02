
#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

if __name__ == '__main__':
    rospy.init_node('brake')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Applying brakes to ebot")
    req = AttachRequest()
    req.model_name_1 = "ebot"
    req.link_name_1 = "ebot_base"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"

    attach_srv.call(req)
