#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class ImuFilteredPublisher:
    def __init__(self):
        rospy.init_node('imu_filtered', anonymous=True)
        self.sub = rospy.Subscriber('imu', Imu, self.callback)
        self.pub = rospy.Publisher('/imu_filtered/z', Float64, queue_size=1)
        self._filtered = 0

    def callback(self, data):
        self.publisher(data.linear_acceleration.z)

    def publisher(self, x):
        self._filtered = self.fir_filter(x=x, filtered_x_prev=self._filtered)
        self.pub.publish(self._filtered)

    def fir_filter(self, x, filtered_x_prev, alpha=0.05):
        """
        FIR digital filter
        """
        filtered_x = alpha * x + (1 - alpha) * filtered_x_prev
        return filtered_x

if __name__ == '__main__':
    try:
        node = ImuFilteredPublisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException: pass