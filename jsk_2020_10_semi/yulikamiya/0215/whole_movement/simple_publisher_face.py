#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/go_back', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    r = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        #FIX later
        pub.publish("go")
        print("published")
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

