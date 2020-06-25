#!/usr/bin/env python

# amcl and move_base use "/map" topic and "/map" frame_id.
# visualization tools use other maps.

# USAGE (rviz)
# Select target floor frame_id as "Fixed Frame",
# Set "initialpose3d" as Estimate Topic in "Tool Propertied" Tab
# then set initialpose

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from topic_tools.srv import MuxSelect


def change_map(frame):
    global tf_select, map_select

    if frame != '/map':
        map_select.publish(frame)

    # call /mux/select "tf"
    try:
        if frame != '/map':
            resp = tf_select(frame+"_tf")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

    rospy.set_param('/amcl/initial_map', frame)


def callback(pose):
    global pub

    # change the map
    frame = pose.header.frame_id
    change_map(frame)

    rospy.sleep(1) # this is bad

    # set initialpose
    pose.header.frame_id = '/map'
    pub.publish(pose)


def listener():
    global pub, tf_select, map_select

    rospy.init_node('initialpose', anonymous=True)

    pub = rospy.Publisher('initialpose_out', PoseWithCovarianceStamped, queue_size=1)
    tf_select = rospy.ServiceProxy('map_tf_mux/select', MuxSelect)
    map_select = rospy.Publisher('map_reload', String, queue_size=1)
    rospy.Subscriber("initialpose_in", PoseWithCovarianceStamped, callback)

    initial_map = rospy.get_param('/amcl/initial_map', None)

    if initial_map:
        change_map(initial_map)

    rospy.spin()

if __name__ == '__main__':
    listener()
