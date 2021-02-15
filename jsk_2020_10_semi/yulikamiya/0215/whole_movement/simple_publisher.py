#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Vector3Stamped

def talker():
    pub = rospy.Publisher('matched_face', Vector3Stamped, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    r = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        #FIX later
        msg = Vector3Stamped()
        msg.header.frame_id = "YuliKamiya"
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

