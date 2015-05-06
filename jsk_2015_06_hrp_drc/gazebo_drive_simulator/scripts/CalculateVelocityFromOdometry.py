#! /usr/bin/env python
# license removed for brevity
import rospy, numpy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

class CalcVelocityFromOdometry:
    def __init__(self):
        rospy.init_node("CalculateVelocityFromOdometry", anonymous=True)
        rospy.Subscriber("/ground_truth_odom", Odometry, self.odom_callback)
        self.pub_vel = rospy.Publisher("/ground_truth_odom/linear_velocity", Float64, queue_size=10)
        self.pub_ang_vel = rospy.Publisher("/ground_truth_odom/angular_velocity", Float64, queue_size=10)
        self.r = rospy.Rate(30) # 30hz

    def execute(self):
        while not rospy.is_shutdown():
            self.r.sleep()


    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        vel = numpy.sqrt(vx*vx + vy*vy)
        ang_vel = wz

        vel_msg = Float64()
        vel_msg.data = vel
        self.pub_vel.publish(vel_msg)
        ang_vel_msg = Float64()
        ang_vel_msg.data = ang_vel
        self.pub_ang_vel.publish(ang_vel_msg)


if __name__ == '__main__':
    try:
        node = CalcVelocityFromOdometry()
        node.execute()
    except rospy.ROSInterruptException: pass
