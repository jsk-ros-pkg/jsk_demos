#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from gazebo_msgs.msg import ModelStates
import math
from geometry_msgs.msg import PoseStamped
from posedetection_msgs.msg import ObjectDetection, Object6DPose
import rospy
import tf2_geometry_msgs as tft
import tf2_ros


class GazeboGroundTruthPerception(object):
    def __init__(self):
        self.sensor_frame_id = rospy.get_param("~sensor_frame_id", "head_mount_kinect_rgb_optical_frame")
        self.gazebo_origin_frame_id = rospy.get_param("~gazebo_origin_frame_id", "odom_combined")
        self.near_threshold = rospy.get_param("~near_threshold", 1.0) # [m]
        self.update_rate = rospy.get_param("~update_rate", 1.0) # [1/Hz]
        self.publish_rate = rospy.get_param("~publish_rate", 0.3)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.object_detection_pub = rospy.Publisher("ObjectDetection", ObjectDetection, queue_size=1)

        self.model_state_sub = None
        self.pub_timer = None
        self.pub_msg = None

    def check_near_object(self, p):
        return math.sqrt(p.position.x * p.position.x + p.position.y * p.position.y) < self.near_threshold

    def model_state_cb(self, msg):
        try:
            now = rospy.Time.now()
            map_to_robot = self.tf_buffer.lookup_transform(self.sensor_frame_id,
                                                           self.gazebo_origin_frame_id,
                                                           now,
                                                           rospy.Duration(self.publish_rate))
            pub_msg = ObjectDetection()
            pub_msg.header.frame_id = self.sensor_frame_id
            pub_msg.header.stamp = now

            for i in range(len(msg.name)):
                if "_static" in msg.name[i]:
                    continue
                ps = PoseStamped()
                ps.header.stamp = now
                ps.header.frame_id = self.gazebo_origin_frame_id
                ps.pose = msg.pose[i]
                robot_to_object = tft.do_transform_pose(ps, map_to_robot)
                if self.check_near_object(robot_to_object.pose):
                    obj = Object6DPose()
                    obj.type = msg.name[i]
                    obj.pose = robot_to_object.pose
                    pub_msg.objects.append(obj)
            if len(pub_msg.objects) > 0:
                self.pub_msg = pub_msg
        except Exception as e:
            rospy.loginfo(str(e))

    def publish(self, event=None):
        if self.pub_msg is not None:
            self.object_detection_pub.publish(self.pub_msg)
            self.pub_msg = None

    def run(self):
        while not rospy.is_shutdown():
            if self.object_detection_pub.get_num_connections() > 0 and self.model_state_sub is None:
                self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_cb)
                self.pub_timer = rospy.Timer(rospy.Duration(self.publish_rate), self.publish)
            elif self.object_detection_pub.get_num_connections() == 0 and self.model_state_sub is not None:
                self.model_state_sub.unregister()
                self.model_state_sub = None
                self.pub_timer.shutdown()
                self.pub_timer = None
            rospy.sleep(self.update_rate)


if __name__ == '__main__':
    rospy.init_node("gazebo_groundtruth_perception")
    GazeboGroundTruthPerception().run()
