#!/usr/bin/python
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from time import sleep
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import Vector3
import math

class Logger:
    def __init__(self):
        rospy.init_node("logger")
        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")

        self.tfl.wait_for_server()
        
        self.state_pitch = 0
        self.state_yaw = 0
        self.input_arm = 0
        self.input_base = 0
        self.vec_z = 0

        rospy.Subscriber("calc_idle_diabolo_state/diabolo_state", Float64MultiArray, self.callbackDiaboloState)
        rospy.Subscriber("/base_odometry/odom", Odometry, self.callbackOdom)

    @staticmethod
    def quaternion_to_euler(quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])
    
    def callbackDiaboloState(self, msg):
        self.state_pitch = msg.data[0]
        self.state_yaw = msg.data[1]
    
    def callbackOdom(self, msg):
        if self.quaternion_to_euler(msg.pose.pose.orientation).z - self.vec_z > 6:
            self.input_base = self.quaternion_to_euler(msg.pose.pose.orientation).z - self.vec_z - math.pi * 2
        elif self.quaternion_to_euler(msg.pose.pose.orientation).z - self.vec_z < -6:
            self.input_base = self.quaternion_to_euler(msg.pose.pose.orientation).z - self.vec_z + math.pi * 2
        else:
            self.input_base = self.quaternion_to_euler(msg.pose.pose.orientation).z - self.vec_z
        self.vec_z = self.quaternion_to_euler(msg.pose.pose.orientation).z
    
    def execute(self):
        while True:
            transform = self.tfl.lookup_transform("base_footprint", "r_gripper_tool_frame", rospy.Time(0))
            self.input_arm = transform.transform.translation.x
            print self.input_arm, self.input_base, self.state_pitch, self.state_yaw
            #sleep(0.05)

if __name__ == '__main__':
    logger = Logger()
    logger.execute()
