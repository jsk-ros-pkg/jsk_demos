#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

class UpDownDetectPublisher:
    def __init__(self):
        rospy.init_node('updown_detector', anonymous=True)
        self.sub = rospy.Subscriber('imu', Imu, self.callback)

    def callback(self, data):
        rospy.loginfo("imu.z is = {}".format(data.linear_acceleration.z))

if __name__ == '__main__':

    try:
        node = UpDownDetectPublisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException: pass