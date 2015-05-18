#!/usr/bin/env python

import rospy
import numpy
import sys
from collections import deque
from std_msgs.msg import Float64
from multisense_ros.msg import RawImuData
from geometry_msgs.msg import *


from math import *

class MagnetometerToDirection:
    def __init__(self):
        self.argv = sys.argv
        self.argc = len(self.argv)
        if self.argc >= 1:
            try:
                self.offset_ang = float(self.argv[1]) * pi / 180
            except:
                print "DATA(1st Argument) Should Be Float, So Default (0.0) Is Used."
                self.offset_ang = 0.0
        else:
            self.offset_ang = 0.0

        rospy.init_node("magnetometer_to_direction", anonymous=True)
        ns = '/multisense/imu'
        rospy.Subscriber( "/staro_look_around/neck_p_angle", Float64, self.look_around_callback)
        rospy.Subscriber( ns + '/magnetometer', RawImuData, self.magnetometer_callback)
        self.ang_msg = None
        self.pub_ps = rospy.Publisher(ns + '/magnetometer_point', PointStamped)
        # self.pub_ang = rospy.Publisher( ns + "/magnetometer_direction_angle", Float64, queue_size=10)
        self.pub_ang = rospy.Publisher("/cheat_goal_dir/ang", Float64, queue_size=10)
        # self.pub_marker = rospy.Publisher( ns + "magnetometer_marker", Marker, queue_size=10)
        self.r = rospy.Rate(1) # 1hz
        self.flag = False
        self.start_ang = 0.0
        self.neck_y_ang = 0.0

        self.q = deque([])
        for i in range(5):
            self.q.append(self.offset_ang)
        
    def execute(self):
        while not rospy.is_shutdown():
            if self.ang_msg != None:
                self.pub_ang.publish(self.ang_msg)
            self.r.sleep()

    def look_around_callback(self, msg):
        self.neck_y_ang = msg.data


    def magnetometer_callback(self, msg):
        x_ref = 0.15
        y_ref = -0.11
        z_ref = -0.36
        gain = 5

        X = msg.x - x_ref
        Y = msg.y - y_ref
        Z = msg.z - z_ref
        ps_msg = PointStamped()
        ps_msg.header.stamp = msg.time_stamp
        ps_msg.header.frame_id = "multisense/mag"
        ps_msg.point.x = X * gain
        ps_msg.point.y = Y * gain
        ps_msg.point.z = Z * gain
        self.pub_ps.publish(ps_msg)
        
        # mag_raw_vector = numpy.array([msg.x, msg.y, msg.z])
        # mag_norm_vector = mag_raw_vector / numpy.linalg.norm(mag_raw_vector)
        # mag_quaternion = numpy.append(mag_norm_vector, 0)

        # marker_msg = Marker()
        # marker_msg.header.stamp = msg.time_stamp
        # marker_msg.header.frame_id = "/car_center"
        # marker_msg.type = Marker.ARROW
        # marker_msg.pose.orientation.x = mag_quaternion[0]
        # marker_msg.pose.orientation.y = mag_quaternion[1]
        # marker_msg.pose.orientation.z = mag_quaternion[2]
        # marker_msg.pose.orientation.w = mag_quaternion[3]
        # marker_msg.scale.x = 1.0
        # marker_msg.scale.y = 0.02
        # marker_msg.scale.z = 0.02
        # marker_msg.color.r = 1.0
        # marker_msg.color.b = 1.0
        # marker_msg.color.a = 1.0
        # self.pub_marker.publish(marker_msg)

        self.ang_msg = Float64()
        ang = atan2(Z, -Y)

        if self.flag == False:
            self.start_ang = ang
            self.flag = True
        

        dacor_ang = -(ang - self.start_ang - self.offset_ang -self.neck_y_ang)
        ave_ang = self.calc_average(dacor_ang)
       
        # rospy.loginfo("goal_direction_angle = %f [deg]", ave_ang * 180 / pi)
        
        self.ang_msg.data = ave_ang


    def calc_average(self, new_data):
        self.q.popleft()
        self.q.append(new_data)
        ave = sum(self.q) / len(self.q)
        return ave
        


if __name__ == '__main__':
    try:
        node = MagnetometerToDirection()
        node.execute()
    except rospy.ROSInterruptException: pass
