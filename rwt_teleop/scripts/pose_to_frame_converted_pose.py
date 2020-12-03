#!/usr/bin/env python                                                                                                                                    

import rospy
import tf
from geometry_msgs.msg import *

def cb(msg):
    stamp = rospy.Time.now()
    src_frame = msg.header.frame_id

    br = tf.TransformBroadcaster()
    br.sendTransform( (msg.point.x, msg.point.y, msg.point.z), (0, 0, 0, 1), rospy.Time.now(), src_frame + "_new", src_frame)


    try:
        # listener.waitForTransform(src_frame+"_new", dst_frame, stamp, timeout=rospy.Duration(1))
        listener.waitForTransform(dst_frame,src_frame+"_new",  stamp, timeout=rospy.Duration(1))
    except Exception as e:
        rospy.logerr(e)
        return

    # dst_pose = listener.lookupTransform(src_frame+"_new", dst_frame, stamp)
    dst_pose = listener.lookupTransform(dst_frame,src_frame+"_new",  stamp)

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = dst_frame
    pose_msg.header.stamp = stamp
    pose_msg.pose.position.x = dst_pose[0][0]
    pose_msg.pose.position.y = dst_pose[0][1]
    pose_msg.pose.position.z = dst_pose[0][2]
    pose_msg.pose.orientation.x = dst_pose[1][0]
    pose_msg.pose.orientation.y = dst_pose[1][1]
    pose_msg.pose.orientation.z = dst_pose[1][2]
    pose_msg.pose.orientation.w = dst_pose[1][3]

    pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('pose_to_frame_converted_pose')
    pub = rospy.Publisher('~output', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('~input', PointStamped, cb)
    dst_frame = rospy.get_param('~dst_frame')
    listener = tf.TransformListener()
    rospy.spin()

