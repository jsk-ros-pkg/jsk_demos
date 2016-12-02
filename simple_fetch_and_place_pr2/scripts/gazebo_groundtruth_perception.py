#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
from gazebo_msgs.msg import ModelStates
import math
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from posedetection_msgs.msg import ObjectDetection, Object6DPose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (Marker, MarkerArray)
import rospy
import tf2_geometry_msgs as tft
import tf2_ros
from tf import transformations as T
import numpy as np


def mat_from_pos(pos):
    return T.translation_matrix((pos.x, pos.y, pos.z))
def mat_from_rot(rot):
    return T.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
def mat_from_pose(pose):
    return np.dot(mat_from_pos(pose.position), mat_from_rot(pose.orientation))
def pos_from_mat(m):
    return Point(*T.translation_from_matrix(m))
def rot_from_mat(m):
    return Quaternion(*T.quaternion_from_matrix(m))
def pose_from_mat(m):
    return Pose(position=pos_from_mat(m), orientation=rot_from_mat(m))


def pose_distance(obj1, obj2=None):
    if obj2 is None: obj2 = Pose()
    return math.sqrt((obj1.position.x - obj2.position.x) * (obj1.position.x - obj2.position.x) +
                     (obj1.position.y - obj2.position.y) * (obj1.position.y - obj2.position.y) +
                     (obj1.position.z - obj2.position.z) * (obj1.position.z - obj2.position.z))

class GazeboGroundTruthPerception(object):
    def __init__(self):
        self.sensor_frame_id = rospy.get_param("~sensor_frame_id", "head_mount_kinect_rgb_optical_frame")
        self.gazebo_robot_frame_id = rospy.get_param("~gazebo_robot_frame_id", "pr2")
        self.ros_robot_frame_id = rospy.get_param("~ros_robot_frame_id", "base_footprint")

        self.always_publish = rospy.get_param("~always_publish", False)
        self.visualize_detected_region = rospy.get_param("~visualize_detected_region", False)

        self.near_threshold = rospy.get_param("~near_threshold", 3.0) # [m]

        self.update_rate = rospy.get_param("~update_rate", 1.0) # [1/Hz]
        self.publish_rate = rospy.get_param("~publish_rate", 0.3)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.object_detection_pub = rospy.Publisher("ObjectDetection", ObjectDetection, queue_size=1)
        self.debug_marker_pub = rospy.Publisher("debug_marker_array", MarkerArray, queue_size=1)

        self.model_state_sub = None
        self.pub_timer = None
        self.pub_msg = None

    def model_state_cb(self, msg):
        try:
            now = rospy.Time.now()
            robot_to_sensor = self.tf_buffer.lookup_transform(self.sensor_frame_id,
                                                              self.ros_robot_frame_id,
                                                              now,
                                                              rospy.Duration(self.publish_rate))
            robot_pose_mat_inv = None
            try:
                idx = msg.name.index(self.gazebo_robot_frame_id)
                robot_pose_matrix = mat_from_pose(msg.pose[idx])
                robot_pose_mat_inv = np.linalg.inv(robot_pose_matrix)
            except ValueError as e:
                rospy.logerr("invalid gazebo robot frame id: %s not in %s" % (self.gazebo_robot_frame_id, msg.name))
                return
            except np.linalg.LinAlgError:
                rospy.logerr("inv mat not found")
                return
            except Exception as e:
                rospy.logerr(str(e))
                return

            # map_to_robot: TransformStamped
            pub_msg = ObjectDetection()
            pub_msg.header.frame_id = self.sensor_frame_id
            pub_msg.header.stamp = now

            for i in range(len(msg.name)):
                if "_static" in msg.name[i]:
                    continue
                # convert to pose from robot origin
                pose_matrix = mat_from_pose(msg.pose[i])
                robot_to_object_mat = np.dot(robot_pose_mat_inv, pose_matrix)
                ps = PoseStamped()
                ps.header.stamp = now
                ps.header.frame_id = self.ros_robot_frame_id
                ps.pose = pose_from_mat(robot_to_object_mat)
                sensor_to_object = tft.do_transform_pose(ps, robot_to_sensor)
                if pose_distance(sensor_to_object.pose) < self.near_threshold:
                    obj = Object6DPose()
                    obj.type = msg.name[i]
                    obj.pose = sensor_to_object.pose
                    pub_msg.objects.append(obj)
            if len(pub_msg.objects) > 0:
                if self.visualize_detected_region:
                    self.publish_marker(pub_msg)
                self.pub_msg = pub_msg
        except Exception as e:
            rospy.logwarn(str(e))

    def publish(self, event=None):
        if self.pub_msg is not None:
            self.object_detection_pub.publish(self.pub_msg)
            self.pub_msg = None

    def subscribe(self):
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
                                                ModelStates, self.model_state_cb,
                                                queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(self.publish_rate), self.publish)

    def unsubscribe(self):
        self.model_state_sub.unregister()
        self.model_state_sub = None
        self.pub_timer.shutdown()
        self.pub_timer = None

    def publish_marker(self, msg):
        # msg = ObjectDetection
        ma = MarkerArray()
        idx = 0

        m = Marker()
        m.header = copy.deepcopy(msg.header)
        m.header.frame_id = self.sensor_frame_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.id = idx
        m.scale.x = self.near_threshold * 2
        m.scale.y = self.near_threshold * 2
        m.scale.z = self.near_threshold * 2
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(0.0, 0.0, 1.0, 0.4)
        m.ns = "gazebo_groundtruth_area"
        m.lifetime = rospy.Time(300)
        ma.markers.append(m)
        idx += 1

        for obj in msg.objects:
            m = Marker()
            m.header = copy.deepcopy(msg.header)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.id = idx
            m.scale.x = 0.07
            m.scale.y = 0.07
            m.scale.z = 0.07
            m.pose = copy.deepcopy(obj.pose)
            m.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            m.ns = "gazebo_groundtruth"
            m.lifetime = rospy.Time(300)
            ma.markers.append(m)
            idx += 1

            m = Marker()
            m.header = copy.deepcopy(msg.header)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.id = idx
            m.scale.z = 0.1
            m.pose = copy.deepcopy(obj.pose)
            m.pose.position.z += 0.05
            m.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            m.text = obj.type
            m.frame_locked = True
            m.ns = "gazebo_groundtruth_text"
            m.lifetime = rospy.Time(300)
            ma.markers.append(m)
            idx += 1
        self.debug_marker_pub.publish(ma)

    def run(self):
        if self.always_publish:
            self.subscribe()
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                if self.object_detection_pub.get_num_connections() > 0 and self.model_state_sub is None:
                    self.subscribe()
                elif self.object_detection_pub.get_num_connections() == 0 and self.model_state_sub is not None:
                    self.unsubscribe()
                rospy.sleep(self.update_rate)


if __name__ == '__main__':
    rospy.init_node("gazebo_groundtruth_perception")
    GazeboGroundTruthPerception().run()
