#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
from jsk_recognition_msgs.msg import ModelCoefficientsArray, Torus, TorusArray
from jsk_topic_tools import ConnectionBasedTransport
from tf.transformations import *
from math import acos

import dynamic_reconfigure.server
from drc_task_common.cfg import ValveRejectorConfig

class ValveRejector(ConnectionBasedTransport):
    def __init__(self):
        super(ValveRejector, self).__init__()
        dynamic_reconfigure.server.Server(
            ValveRejectorConfig, self.config_callback)
        self.pub_torus = self.advertise('~output', Torus)
        self.pub_torus_array = self.advertise('output_array', TorusArray)
    def subscribe(self):
        self.sub_coef = message_filters.Subscriber('/valve_detector/extract_top_polygon_likelihood/output/coefficients', ModelCoefficientsArray)
        self.sub_torus = message_filters.Subscriber('/valve_detector/torus_finder/output', Torus)
        self.sync = message_filters.TimeSynchronizer([self.sub_torus, self.sub_coef])
        self.sync.registerCallback(self.callback)
    def unsubscribe(self):
        self.sub_coef.unregister()
        self.sub_torus.unregister()
    def config_callback(self, config, level):
        self.eps_angle = config.eps_angle
        return config
    def callback(self, coef_msg, torus_msg):
        if coef_msg.header.frame_id != torus_msg.header.frame_id:
            rospy.logerr('frame_id is not correct. coef: {0}, torus: {1}'.format(
                coef_msg.header.frame_id, torus_msg.header.frame_id))
        # convert torus_msg.pose to matrix
        torus_rot = quaternion_matrix([torus_msg.pose.orientation.x,
                                       torus_msg.pose.orientation.y,
                                       torus_msg.pose.orientation.z,
                                       torus_msg.pose.orientation.w])
        # 3x3 x 3x1
        torus_z_axis = np.dot(torus_rot, np.array([0, 0, -1]))
        coef_axis = np.array([coef_msg.coefficients[0].values[0],
                                 coef_msg.coefficients[0].values[1],
                                 coef_msg.coefficients[0].values[2]])
        torus_z_axis = torus_z_axis / np.linalg.norm(torus_z_axis)
        coef_axis = coef_axis / np.linalg.norm(coef_axis)
        theta = acos(np.dot(torus_z_axis, coef_axis))
        if theta > self.eps_angle:
            rospy.logwarn("torus detection is rejected")
        else:
            slef.pub_torus(torus_msg)
            arr = TorusArray()
            arr.header = torus_msg.header
            arr.toruses = [torus_msg]
            self.pub_torus_array.publish(arr)
            

if __name__ == "__main__":
    rospy.init_node("valve_rejector")
    r = ValveRejector()
    rospy.spin()
    
