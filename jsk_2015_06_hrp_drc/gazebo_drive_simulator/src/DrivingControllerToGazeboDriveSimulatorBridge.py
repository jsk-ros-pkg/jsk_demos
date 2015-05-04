#! /usr/bin/env python

import roslib; roslib.load_manifest('gazebo_drive_simulator')
from std_msgs.msg import *
from sensor_msgs.msg import *
import rospy
import numpy

class DrivingControllerToGazeboDriveSimulatorBridge:
    def __init__ (self, node_name = "driving_controller_to_gazebo_drive_simulator_bridge", rate = 10):
        rospy.init_node(node_name)
        self.rate = rospy.Rate(rate)
        rospy.Subscriber("~driving_controller_handle", Float32, self.handle_callback)
        self.gazebo_handle_publisher = rospy.Publisher("~handle_cmd", Float64)
        rospy.Subscriber("~driving_controller_accel", Float32, self.accel_callback)
        self.gazebo_accel_publisher = rospy.Publisher("~accel_cmd", Float64)
        rospy.Subscriber("~driving_controller_min_step", Float32, self.min_step_callback)
        rospy.Subscriber("~driving_controller_max_step", Float32, self.max_step_callback)
        self.min_step = None
        self.max_step = None        

    def handle_callback(self, msg):
        pub_msg = Float64()
        pub_msg.data = msg.data * numpy.pi / 180.0 # [deg] -> [rad]
        self.gazebo_handle_publisher.publish(pub_msg)

    def accel_callback(self, msg):
        if self.min_step == None or self.max_step == None:
            return
        pub_msg = Float64()
        pub_msg.data = (msg.data - self.min_step) / (self.max_step - self.min_step)
        self.gazebo_accel_publisher.publish(pub_msg)

    def min_step_callback(self, msg):
        self.min_step = msg.data

    def max_step_callback(self, msg):
        self.max_step = msg.data
        
    def execute(self):
        self.rate.sleep()

if __name__ == "__main__":
    node = DrivingControllerToGazeboDriveSimulatorBridge()
    while not rospy.is_shutdown():
        node.execute()
