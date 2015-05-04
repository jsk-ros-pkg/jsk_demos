#! /usr/bin/env python

import roslib; roslib.load_manifest('drc_task_common')
from std_msgs.msg import *
from sensor_msgs.msg import *
import rospy
import numpy

class HandleControllerPublisher:
    # init value of /joy is 0 but neutral value of brake/accel pedal is -1
    is_accel_inited = False
    is_brake_inited = False
    # remember old values and only publish when current != old
    old_handle_joy = None
    old_accel_joy = None
    old_brake_joy = None
    old_gear_joy = None

    def __init__ (self, node_name = "handle_controller", rate = 100):
        rospy.init_node(node_name)        
        self.rate = rospy.Rate(rate)
        self.handle_publisher = rospy.Publisher("~handle", Float64)
        self.accel_publisher = rospy.Publisher("~accel", Float64)
        self.brake_publisher = rospy.Publisher("~brake", Float64)
        self.gear_publisher = rospy.Publisher("~gear", Int8)
        rospy.Subscriber("driving_force_gt/joy", Joy, self.callback)

    def callback(self, msg):
        current_handle_joy = msg.axes[0]
        current_accel_joy = msg.axes[1]
        current_brake_joy = msg.axes[2]
        current_gear_joy = msg.buttons[12] - msg.buttons[13]
        
        self.publish_handle_angle(current_handle_joy)
        self.publish_accel(current_accel_joy)
        self.publish_brake(current_brake_joy)
        # self.publish_gear(current_gear_joy)
        
        # if self.old_handle_joy == None or self.old_handle_joy != current_handle_joy:
        #     self.publish_handle_angle(current_handle_joy)
        # if self.old_accel_joy == None or self.old_accel_joy != current_accel_joy:
        #     self.publish_accel(current_accel_joy)
        # if self.old_brake_joy == None or self.old_brake_joy != current_brake_joy:
        #     self.publish_brake(current_brake_joy)
        if self.old_gear_joy == None or self.old_gear_joy != current_gear_joy:
            self.publish_gear(current_gear_joy)

    def publish_handle_angle(self, handle_joy):
        handle_angle = Float64()
        handle_angle.data = self.analog2rad(-1.0, 1.0, -numpy.pi * (5.0 / 2.0), numpy.pi * (5.0 / 2.0), handle_joy)
        self.handle_publisher.publish(handle_angle)
        self.old_handle_joy = handle_joy

    def publish_accel(self, accel_joy):
        if accel_joy != 0.0:
            self.is_accel_inited = True # initialize check
        accel_angle = Float64()
        if self.is_accel_inited:
            tmp_accel_analog = accel_joy
        else:
            tmp_accel_analog = -1.0 # -1 means neautral (not pedaled)
        accel_angle.data = self.analog2rad(-1.0, 1.0, 0, numpy.pi * (1.0 / 3.0), tmp_accel_analog)
        self.accel_publisher.publish(accel_angle)
        self.old_accel_joy = accel_joy
        
    def publish_brake(self, brake_joy):
        if brake_joy != 0.0:
            self.is_brake_inited = True
        brake_angle = Float64()        
        if self.is_brake_inited:
            tmp_brake_analog = brake_joy
        else:
            tmp_brake_analog = -1.0 # -1 means neautral (not pedaled)
        brake_angle.data = self.analog2rad(-1.0, 1.0, 0, numpy.pi * (1.0 / 3.0), tmp_brake_analog)
        self.brake_publisher.publish(brake_angle)
        self.old_brake_joy = brake_joy
        
    def publish_gear(self, gear_joy):
        gear_direction = Int8()
        gear_direction.data = gear_joy
        self.gear_publisher.publish(gear_direction)
        self.old_gear_joy = gear_joy
        
    def analog2rad(self, analog_min, analog_max, rad_min, rad_max, data):
        step = (rad_max - rad_min) / (analog_max - analog_min)
        return step * (data - analog_min) + rad_min        

    def execute(self):
        self.rate.sleep()

if __name__ == "__main__":
    ctrl = HandleControllerPublisher()
    while not rospy.is_shutdown():
        ctrl.execute()
