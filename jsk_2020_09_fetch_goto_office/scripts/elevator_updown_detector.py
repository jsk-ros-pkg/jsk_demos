#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, String

class UpDownDetectPublisher:
    def __init__(self):
        rospy.init_node('updown_detector', anonymous=True)
        self.sub = rospy.Subscriber('/imu_filtered/z/passthroughed', Float64, self.callback)
        self.pub = rospy.Publisher('/updown_state', String, queue_size=1)

        self.elevator_state = "stopping"
        self.speed_state = "stopping"
        
    def callback(self, data):
        self.publisher(data)
        
    def publisher(self, data):
        self.updownstate_detector(data)
        self.pub.publish(self.elevator_state)

    def updownstate_detector(self, data, acceleration_th_max=10, acceleration_th_min=9.3):
        if self.elevator_state=="stopping" and self.speed_state=="stopping" and data.data>=acceleration_th_max:
            self.elevator_state = "ascending"
            self.speed_state = "increasing"
        elif self.elevator_state=="ascending" and self.speed_state=="increasing" and data.data<=acceleration_th_min:
            self.speed_state = "reducing"
        elif self.elevator_state=="ascending" and self.speed_state=="reducing" and data.data>=acceleration_th_min:
            self.elevator_state = "stopping"
            self.speed_state = "stopping"
        elif self.elevator_state=="stopping" and self.speed_state=="stopping" and data.data<=acceleration_th_min:
            self.elevator_state = "descending"
            self.speed_state = "increasing"
        elif self.elevator_state=="descending" and self.speed_state=="increasing" and data.data>=acceleration_th_max:
            self.speed_state = "reducing"
        elif self.elevator_state=="descending" and self.speed_state=="reducing" and data.data<=acceleration_th_max:
            self.elevator_state = "stopping"
            self.speed_state = "stopping"
        else:
            pass
            
            
if __name__ == '__main__':
    try:
        node = UpDownDetectPublisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException: pass
