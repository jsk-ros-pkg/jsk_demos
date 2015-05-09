#! /usr/bin/env python
# license removed for brevity
import numpy
import rospy
from std_msgs.msg import Float64
from visualization_msgs.msg import *
from tf.transformations import quaternion_from_euler
from drc_task_common.msg import Int8Float64

class SteeringAngleMarker:
    def __init__(self):
        rospy.init_node("SteeringAngleMarker", anonymous=True)
        rospy.Subscriber("cmd", Int8Float64, self.steering_marker_callback)
        self.pub_marker = rospy.Publisher("marker_array", MarkerArray, queue_size=10)
        self.r = rospy.Rate(30) # 30hz
        
    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()
            
    def steering_marker_callback(self, msg):
        # ang = msg.data / 4.0
        ang = msg.data
        quaternion = quaternion_from_euler(0, 0, ang)
        text = str(msg.index)

        marker_array_msg = MarkerArray()

        # show arrow for visualizing steering angle direction
        marker_arrow = Marker()
        marker_arrow.header.stamp = rospy.Time.now()
        marker_arrow.header.frame_id = "car_center"
        marker_arrow.type = Marker.ARROW
        marker_arrow.id = 3
        marker_arrow.pose.orientation.x = quaternion[0]
        marker_arrow.pose.orientation.y = quaternion[1]
        marker_arrow.pose.orientation.z = quaternion[2]
        marker_arrow.pose.orientation.w = quaternion[3]
        marker_arrow.scale.x = 1.0
        marker_arrow.scale.y = 0.02
        marker_arrow.scale.z = 0.02
        marker_arrow.color.g = 1.0
        marker_arrow.color.b = 1.0
        marker_arrow.color.a = 1.0

        marker_array_msg.markers.append(marker_arrow)

        # show the number of index
        marker_string = Marker()
        marker_string.header.stamp = rospy.Time.now()
        marker_string.header.frame_id = "car_center"
        marker_string.type = Marker.TEXT_VIEW_FACING
        marker_string.id = 4
        marker_string.text = text
        marker_string.scale.z = 0.15
        marker_string.pose.position.x = -0.2
        marker_string.pose.position.y = 0
        marker_string.pose.position.z = 0.1
        marker_string.color.g = 1.0
        marker_string.color.b = 1.0
        marker_string.color.a = 1.0

        marker_array_msg.markers.append(marker_string)

        self.pub_marker.publish(marker_array_msg)
        
if __name__ == '__main__':
    try:
        node = SteeringAngleMarker()
        node.execute()
    except rospy.ROSInterruptException: pass
