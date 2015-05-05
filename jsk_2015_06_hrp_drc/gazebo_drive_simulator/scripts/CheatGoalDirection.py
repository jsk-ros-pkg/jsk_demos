#! /usr/bin/env python
# license removed for brevity
import numpy
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from tf.transformations import *

from math import pi

class GoalDirectionPublisher:
    def __init__(self):
        rospy.init_node("CheatGoalDirectionPublisher", anonymous=True)
        rospy.Subscriber("/ground_truth_odom", Odometry, self.cheat_odom_callback)
        self.pub_ang = rospy.Publisher("cheat_goal_dir/ang", Float64, queue_size=10)
        self.pub_goalpoint = rospy.Publisher("cheat_goal/point", PoseStamped, queue_size=10)
        self.pub_marker = rospy.Publisher("cheat_goal_dir/marker", Marker, queue_size=10)
        self.r = rospy.Rate(1) # 1hz
        self.start_ang = 0.0
        self.flag = False
        self.goal_msg = None
        
    def execute(self):
        while not rospy.is_shutdown():
            if self.goal_msg != None:
                self.pub_goalpoint.publish(self.goal_msg)
            self.r.sleep()

    def cheat_odom_callback(self, msg):
        odom_quaternion = [msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(odom_quaternion)
        ang = euler[2]

        if self.flag == False:
            self.start_ang = ang
            self.flag = True
        
        dacor_ang = -(ang - self.start_ang)

        # rospy.loginfo("e1 = %f e2 = %f ang = %f", e1, e2, dacor_ang)
        
        ang_msg = Float64()
        
        ang_msg.data = dacor_ang
        self.pub_ang.publish(ang_msg)
        
        quaternion = quaternion_from_euler(0, 0, dacor_ang)

        marker_msg = Marker()
        marker_msg.header.stamp = msg.header.stamp
        marker_msg.header.frame_id = "/car_center"
        marker_msg.type = Marker.ARROW
        marker_msg.pose.orientation.x = quaternion[0]
        marker_msg.pose.orientation.y = quaternion[1]
        marker_msg.pose.orientation.z = quaternion[2]
        marker_msg.pose.orientation.w = quaternion[3]
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 0.02
        marker_msg.scale.z = 0.02
        marker_msg.color.r = 1.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0
        self.pub_marker.publish(marker_msg)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.stamp = msg.header.stamp
        self.goal_msg.header.frame_id = "/map"
        self.goal_msg.pose.position.x = 20 * numpy.cos(dacor_ang)
        self.goal_msg.pose.position.y = 20 * numpy.sin(dacor_ang)
        self.goal_msg.pose.orientation.x = quaternion[0]
        self.goal_msg.pose.orientation.y = quaternion[1]
        self.goal_msg.pose.orientation.z = quaternion[2]
        self.goal_msg.pose.orientation.w = quaternion[3]


if __name__ == '__main__':
    try:
        node = GoalDirectionPublisher()
        node.execute()
    except rospy.ROSInterruptException: pass
