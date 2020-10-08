#!/usr/bin/env python
import rospy
from std_msgs import Float64, String

class UpDownDetectPublisher:
    def __init__(self):
        rospy.init_node('updown_detector', anonymous=True)
        self.sub = rospy.Subscriber('/imu_filtered/z', Float64, self.callback)
        self.pub = rospy.Publisher('/updown_state', String, queue_size=1)
        self.

    def callback(self, data):
        self.publisher(data)
        
    def publisher(self, data):
        state = updownstate_detector(data)
        self.pub.publish(state)

    def updownstate_detector(self, data, acceleration_th):
        
        

if __name__ == '__main__':
    try:
        node = UpDownDetectPublisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException: pass