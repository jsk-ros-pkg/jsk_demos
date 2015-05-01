#! /usr/bin/env python

import roslib; roslib.load_manifest('gazebo_drive_simulator')
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import rospy
import numpy
import HandleControllerPublisher
from dynamic_reconfigure.server import Server
from gazebo_drive_simulator.cfg import HandleAngleWeightConfig

class DRCDriveControllerNode:
    def __init__ (self, node_name = "drc_drive_controller", handle_controller_namespace = "/handle_controller", rate = 10):
        rospy.init_node(node_name)
        self.rate = rospy.Rate(rate)
        rospy.Subscriber(handle_controller_namespace + "/handle", Float64, self.handle_callback)
        self.drc_handle = rospy.Publisher("~handle_cmd", Float64)
        rospy.Subscriber(handle_controller_namespace + "/accel", Float64, self.accel_callback)
        self.drc_accel = rospy.Publisher("~accel_cmd", Float64)
        rospy.Subscriber(handle_controller_namespace + "/brake", Float64, self.brake_callback)
        self.drc_brake = rospy.Publisher("~brake_cmd", Float64)
        rospy.Subscriber(handle_controller_namespace + "/gear", Int8, self.gear_callback)
        self.drc_gear = rospy.Publisher("~gear_cmd", Int8)
        srv = Server(HandleAngleWeightConfig, self.dynamic_reconfigure_callback)
        self.handle_angle_weight = 1.0

    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("\n\n\nhandle_angle_weight = %f\n\n", config["handle_angle_weight"])
        self.handle_angle_weight = config["handle_angle_weight"]
        return config

    def rad2analog(self, rad_min, rad_max, analog_min, analog_max, data):
        step = (analog_max - analog_min) / (rad_max - rad_min)
        return step * (data - rad_min) + analog_min

    def handle_callback(self, msg):
        pub_msg = Float64()
        pub_msg.data = msg.data * self.handle_angle_weight # offset for real handling
        self.drc_handle.publish(pub_msg)

    def accel_callback(self, msg):
        pub_msg = Float64()
        pub_msg.data = self.rad2analog(0, numpy.pi * (1.0 / 3.0), 0, 1, msg.data)
        self.drc_accel.publish(pub_msg)

    def brake_callback(self, msg):
        pub_msg = Float64()
        pub_msg.data = self.rad2analog(0, numpy.pi * (1.0 / 3.0), 0, 1, msg.data)
        self.drc_brake.publish(pub_msg)

    def gear_callback(self, msg):
        if msg.data != 0: # 0 means direction is not to be changed
            pub_msg = Int8()
            pub_msg.data = msg.data
            self.drc_gear.publish(pub_msg)

    def execute(self):
        self.rate.sleep()

if __name__ == "__main__":
    node = DRCDriveControllerNode()
    while not rospy.is_shutdown():
        node.execute()
