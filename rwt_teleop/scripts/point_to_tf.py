#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import tf

class PointToTf():
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'default_frame_id')
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber('~input', PointStamped, self.cb)

    def cb(self, msg):
        pos  = msg.point
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (0,0,0,1),
                              msg.header.stamp, self.frame_id, msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('point_to_tf', anonymous=True)
    PointToTf()
    rospy.spin()
