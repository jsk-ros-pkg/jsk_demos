#!/usr/bin/env python
import rospy

PKG='jsk_recognition_msgs'

import imp
import numpy
import tf
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import *
from geometry_msgs.msg import *
if __name__ == "__main__":
    rospy.init_node('hand_box_publisher', anonymous=True)
    box_pub = rospy.Publisher('hand_box', BoundingBox)
    box_array_pub = rospy.Publisher('hand_box_array', BoundingBoxArray)
    listener = tf.TransformListener()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        pose_stamped = PoseStamped(Header(stamp=now, frame_id="larm_end_coords"), Pose(Point(0.045, -0.015, 0), Quaternion(0, 0, 0, -1)))
        try:
            listener.waitForTransform('left_camera_optical_frame', pose_stamped.header.frame_id,  now, rospy.Duration(1))
            transed_pose = listener.transformPose('left_camera_optical_frame', pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), e:
            print "tf error: %s" % e
            r.sleep()
            continue
        box = BoundingBox(Header(stamp=rospy.Time.now(), frame_id='left_camera_optical_frame'), transed_pose.pose, Vector3(0.15, 0.15, 0.32))
        box_pub.publish(box)
        box_array = BoundingBoxArray(Header(stamp=rospy.Time.now(), frame_id="rarm_end_coords"), [box])
        box_array_pub.publish(box_array)
        r.sleep()
