#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from geometry_msgs.msg import *
import numpy
import tf
from jsk_rviz_plugins.srv import *
from jsk_interactive_marker.srv import *
def request_handle_marker():
    rospy.wait_for_service("/drive/handle_server/request_marker_operate")
    handle_request = rospy.ServiceProxy('/drive/handle_server/request_marker_operate', RequestMarkerOperate)
    try:
        handle_request(jsk_rviz_plugins.msg.TransformableMarkerOperate(2, 0, "BODY", "handle_marker", ""))
    except rospy.ServiceException as e:
        print("Service fail: %s" % e)
        return
    rospy.wait_for_service("/drive/handle_server/set_dimensions")
    handle_size_request = rospy.ServiceProxy('/drive/handle_server/set_dimensions', SetMarkerDimensions)
    try:
        handle_size_request("", jsk_interactive_marker.msg.MarkerDimensions(0, 0, 0, 0.14, 0.01, 0))
    except rospy.ServiceException as e:
        print("Service fail: %s" % e)
        return


if __name__ == "__main__":
    rospy.init_node('revice_button_pose', anonymous=True)
    #posepub = rospy.Publisher('', PoseStamped)
    request_handle_marker()
    
