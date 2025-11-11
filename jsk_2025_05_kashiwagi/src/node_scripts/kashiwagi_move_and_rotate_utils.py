#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveAndRotate:
    def __init__(self):
        self.pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def move_forward_target_distance(self, distance):
        # distance [meter]
        twist = Twist()
        if distance >= 0:
            twist.linear.x = 0.001 # [meter / second] 実際は twist.linear.x * 50 [meter / second]
        else:
            twist.linear.x = - 0.001
        
        start_time = rospy.Time.now()
        seconds = abs(distance) / (abs(twist.linear.x) * 50) # 分母に * 50 してズレを修正している

        duration = rospy.Duration(seconds)

        while not rospy.is_shutdown() and rospy.Time.now() - start_time < duration:
            self.pub.publish(twist)
            self.rate.sleep()

    def move_forward_target_velocity(self, velocity):
        twist = Twist()
        twist.linear.x = velocity / 50# [meter / second] (引数で与えるのはあっている単位)
        self.pub.publish(twist)

    def rotate_target_radian(self, radian):
        twist = Twist()
        if radian >= 0:
            twist.angular.z = 0.01 # [rad / second] 実際は (rad * (13 / 360)) / second
        else:
            twist.angular.z = - 0.01 # [rad / second] 実際は (rad * (13 / 360)) / second

        start_time = rospy.Time.now()
        seconds = abs(radian) * 13 / (360 * abs(twist.angular.z)) # ここでズレ修正してる

        duration = rospy.Duration(seconds)

        while not rospy.is_shutdown() and rospy.Time.now() - start_time < duration:
            self.pub.publish(twist)
            self.rate.sleep()

    def rotate_target_velocity(self, velocity):
        twist = Twist()
        twist.angular.z = velocity * (13 / 360) # [rad / second] 実際は (rad * (13 / 360)) / second
        self.pub.publish(twist)
