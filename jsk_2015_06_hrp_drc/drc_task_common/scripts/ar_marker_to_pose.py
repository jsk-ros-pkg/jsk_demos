#!/usr/bin/env python

# Script to convert ar_pose/ARMarker into geometry_msgs/PoseStamped

import rospy
from ar_pose.msg import ARMarker
from geometry_msgs.msg import PoseStamped 

def callback(msg):
    global pub
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose
    pub.publish(pose_stamped)


if __name__ == "__main__":
    rospy.init_node("ar_pose_to_pose")
    pub = rospy.Publisher("~output", PoseStamped)
    sub = rospy.Subscriber("~input", ARMarker, callback)
    rospy.spin()
