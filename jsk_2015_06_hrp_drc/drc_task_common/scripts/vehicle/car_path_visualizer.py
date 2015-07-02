#!/usr/bin/env python
import rospy
import numpy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf
from tf.transformations import quaternion_matrix
from math import pi

class CarPathVisualizer:
    def __init__(self):
        rospy.init_node("CarPathVisualizer", anonymous=True)
        rospy.Subscriber("car_steering_wheel", Float32, self.steering_callback)
        rospy.Subscriber("execute_flag", Bool, self.execute_flag_callback)
        self.marker_pub = rospy.Publisher("car_path_marker", MarkerArray, queue_size=10)
        self.r = rospy.Rate(5)
        self.a = 0.0258676
        self.play = 0.60952311
        self.b = -self.a * self.play
        self.A = self.a / (4 * self.play)
        self.P_c = 2 * self.play
        self.tread = 1.32
        self.line_length = 0.2
        self.ang = 0.0
        self.steering = 0.0
        self.execute_flag = True
        self.curve_length = 30
        self.polygon = 50
        self.tfl = tf.TransformListener()


    def execute(self):
        while not rospy.is_shutdown():
            if self.execute_flag == True:
                if self.tfl.frameExists("BODY") and self.tfl.frameExists("car_center"):
                    tm = self.tfl.getLatestCommonTime("BODY", "car_center")
                    try:
                        (self.pos, quat) = self.tfl.lookupTransform("BODY", "car_center", tm)
                        self.R = quaternion_matrix(quat)[0:3,0:3]
                        # print self.R
                        self.marker_publisher()
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                        print "tf error: %s" % e
                        pass
            self.r.sleep()

    def execute_flag_callback(self, msg):
        self.execute_flag = msg.data

    def steering_callback(self, msg):
        if self.execute_flag == True:
            self.steering = msg.data * pi / 180.0

    def marker_publisher(self):
        if self.execute_flag == False:
            return

        marker_array_msg = MarkerArray()

        curvature = self.alpha_to_kappa_table(self.steering)

        if (curvature == 0):
            self.ang = 0.0
        else:
            self.ang = 2* numpy.arcsin(self.line_length / 2.0 * numpy.fabs(curvature)) * curvature / numpy.fabs(curvature)

        temp_velocity = numpy.array([self.line_length, 0.0])
        temp_point = numpy.array([-self.line_length, 0.0])
        rotation = numpy.array([[numpy.cos(self.ang), -numpy.sin(self.ang)], [numpy.sin(self.ang), numpy.cos(self.ang)]])
        center_point_array = []
        center_velocity_array = []
        for count in range(self.curve_length):
            temp_velocity = numpy.dot(rotation, temp_velocity)
            temp_point = temp_velocity + temp_point
            center_point_array.append(temp_point)
            center_velocity_array.append(temp_velocity)
        l_point_array = []
        r_point_array = []
        for (p, vel) in zip(center_point_array, center_velocity_array):
            vel_norm = numpy.array([vel[1], -vel[0]])
            vel_norm/=numpy.linalg.norm(vel_norm)
            l_p = p + vel_norm * (self.tread/2.0)
            r_p = p - vel_norm * (self.tread/2.0)
            l_p_array = numpy.array([ (l_p[0]-0.75), l_p[1], 0 ])
            r_p_array = numpy.array([ (r_p[0]-0.75), r_p[1], 0 ])
            Rl_p = numpy.dot(self.R, l_p_array)
            Rr_p = numpy.dot(self.R, r_p_array)
            l_point_array.append(Point( (self.pos[0]+Rl_p[0]), (self.pos[1]+Rl_p[1]), (self.pos[2]+Rl_p[2])) )
            r_point_array.append(Point( (self.pos[0]+Rr_p[0]), (self.pos[1]+Rr_p[1]), (self.pos[2]+Rr_p[2])) )

        marker_left = Marker(header=std_msgs.msg.Header(frame_id="BODY"), type = Marker.LINE_STRIP, action = Marker.ADD, colors = [std_msgs.msg.ColorRGBA(1, 0.3, 0, 0.5)]*self.curve_length, scale = Vector3(0.2, 1, 1), points = l_point_array, id = 2, ns = "left_wheel")
        marker_array_msg.markers.append(marker_left)

        marker_right = Marker(header=std_msgs.msg.Header(frame_id="BODY"), type = Marker.LINE_STRIP, action = Marker.ADD, colors = [std_msgs.msg.ColorRGBA(1, 0.3, 0, 0.5)]*self.curve_length, scale = Vector3(0.2, 1, 1), points = r_point_array, id = 1, ns = "right_wheel")
        marker_array_msg.markers.append(marker_right)

        self.marker_pub.publish(marker_array_msg)


    def alpha_to_kappa_table(self, alpha):
        if (alpha >= 0):
            if (alpha <= self.P_c):
                kappa = self.A * alpha * alpha
            else:
                kappa = self.a * alpha + self.b
        else:
            if (alpha >= -self.P_c):
                kappa = -self.A * alpha * alpha
            else:
                kappa = self.a * alpha - self.b
        return kappa


if __name__ == "__main__":
    try:
        node = CarPathVisualizer()
        node.execute()
    except rospy.ROSInterruptException: pass
