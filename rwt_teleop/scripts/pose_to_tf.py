#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf

class PoseToTf():
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'default_frame_id')
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber('~input', PoseStamped, self.cb)

    def cb(self, msg):
        pos  = msg.pose.position
        quat = msg.pose.orientation
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              msg.header.stamp, self.frame_id, msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('pose_to_tf', anonymous=True)
    PoseToTf()
    rospy.spin()
