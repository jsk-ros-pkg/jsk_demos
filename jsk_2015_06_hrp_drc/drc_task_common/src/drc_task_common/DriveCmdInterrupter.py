#! /usr/bin/env python

import roslib; roslib.load_manifest('drc_task_common')
from std_msgs.msg import *
import rospy
import numpy

class DriveCmdInterrupter:
    def __init__(self, node_name = "command_selector", rate = 10.0):
        rospy.init_node(node_name)
        self.rate = rospy.Rate(rate)
        # self.handle_cmd_buf = []
        self.old_handle_cmd_length = 0
        self.max_accel_cmd = 1.0
        self.max_brake_cmd = 1.0
        
        # publisher
        self.handle_publisher = rospy.Publisher("drive/operation/handle_cmd_raw", Float64)
        self.accel_publisher = rospy.Publisher("drive/operation/accel_cmd_raw", Float64)
        self.brake_publisher = rospy.Publisher("drive/operation/brake_cmd_raw", Float64)
        self.grasp_publisher = rospy.Publisher("drive/operation/grasp_cmd", String)
        self.operation_flag_handle_publisher = rospy.Publisher("drive/operation/flag/handle", Bool)
        self.operation_flag_pedal_publisher = rospy.Publisher("drive/operation/flag/pedal", Bool)

        # subscribe operation topics
        rospy.Subscriber("/handle_controller/handle", Float64, self.handle_cmd_callback)
        rospy.Subscriber("/handle_controller/accel", Float64, self.accel_cmd_callback)
        rospy.Subscriber("/handle_controller/brake", Float64, self.brake_cmd_callback)
        rospy.Subscriber("/handle_controller/gear", Int8, self.recognition_flag_callback)
        rospy.Subscriber("/drive/grasp_cmd", String, self.grasp_cmd_callback)
        
    def set_operation_flag(self, pub, flag):
        operation_flag = Bool()
        operation_flag.data = flag
        pub.publish(operation_flag)

        
    def handle_cmd_callback(self, msg):
        # self.set_operation_flag(self.operation_flag_handle_publisher, True)
        # self.handle_cmd_buf.append(msg.data)
        turn_cmd = Float64()
        turn_cmd.data = msg.data
        self.handle_publisher.publish(turn_cmd)

    def grasp_cmd_callback(self, msg):
        # self.set_operation_flag(self.operation_flag_handle_publisher, True)
        grasp_cmd = String()
        grasp_cmd.data = msg.data
        self.grasp_publisher.publish(grasp_cmd)
        # self.handle_cmd_buf = [] # init handle_cmd_buf to prevent turn topic

    def accel_cmd_callback(self, msg):
        # self.set_operation_flag(self.operation_flag_pedal_publisher, True)
        accel_cmd = Float64()
        accel_cmd.data = self.max_accel_cmd * msg.data # 0.0 <= accel_cmd <= max_accel_cmd
        self.accel_publisher.publish(accel_cmd)

    def brake_cmd_callback(self, msg):
        # self.set_operation_flag(self.operation_flag_pedal_publisher, True)
        brake_cmd = Float64()
        brake_cmd.data = self.max_brake_cmd * msg.data # 0.0 <= brake_cmd <= max_brake_cmd
        self.brake_publisher.publish(brake_cmd)
            
    def recognition_flag_callback(self, msg):
        if msg.data > 0:
            self.set_operation_flag(self.operation_flag_handle_publisher, True)
            # self.set_operation_flag(self.operation_flag_pedal_publisher, True)
        elif msg.data < 0:
            self.set_operation_flag(self.operation_flag_handle_publisher, False)
            # self.set_operation_flag(self.operation_flag_pedal_publisher, False)
        else:
            pass # do nothing when msg.data == 0
        
    def execute(self):
        # handle command trimming, this is temporary implementation
        # if len(self.handle_cmd_buf) != 0:
        #     if len(self.handle_cmd_buf) == self.old_handle_cmd_length:
        #         turn_cmd = Float64()
        #         turn_cmd.data = self.handle_cmd_buf.pop()
        #         self.handle_publisher.publish(turn_cmd)
        #         self.handle_cmd_buf = []
        #     self.old_handle_cmd_length = len(self.handle_cmd_buf)
        self.rate.sleep()

if __name__ == "__main__":
    selector = DriveCmdInterrupter()
    while not rospy.is_shutdown():
        selector.execute()
        
