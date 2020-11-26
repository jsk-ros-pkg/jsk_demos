#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import argparse
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Pose, Quaternion
import random
import rospy
import string
import tf.transformations as trans


def generate_camera_model(ns="rgb", update_rate=1.0, format="R8G8B8",
                          width=800, height=600,
                          near_clip=0.1, far_clip=100, hfov=1.047):
    return """
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="camera">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="camera_link">
      <visual name="visual">
        <pose>-0.055 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.04 0.025</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 0</emissive>
        </material>
      </visual>
      <visual name="visual">
        <pose>0 0 0 0 1.57 0</pose>
        <geometry>
          <cylinder>
            <radius>0.013</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 0</emissive>
        </material>
      </visual>
      <sensor name="{ns}" type="camera">
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>{format}</format>
          </image>
          <clip>
            <near>{near_clip}</near>
            <far>{far_clip}</far>
          </clip>
        </camera>
        <update_rate>{update_rate}</update_rate>
        <plugin filename="libgazebo_ros_camera.so" name="{ns}">
          <cameraName>{ns}</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>camera_link</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>""".format(width=width, height=height, format=format,
                 near_clip=near_clip, far_clip=far_clip, hfov=hfov,
                 update_rate=update_rate, ns=ns)


def random_string(str_len, suffix=""):
    return suffix + ''.join([random.choice(string.ascii_letters + string.digits) for i in range(str_len)])

def spawn_camera_model(name, model, ros_ns, gazebo_ns, pose_args, ref_frame):
    pose = Pose()
    pose.position.x = pose_args[0]
    pose.position.y = pose_args[1]
    pose.position.z = pose_args[2]
    if len(pose_args) == 7:
        # quaternion expression
        tmpq = pose_args[3:]
    else:
        # rpy expression
        tmpq = trans.quaternion_from_euler(pose_args[3],pose_args[4],pose_args[5])
    pose.orientation = Quaternion(tmpq[0], tmpq[1], tmpq[2], tmpq[3])

    return gazebo_interface.spawn_sdf_model_client(name, model, ros_ns,
                                                   pose, ref_frame, gazebo_ns)

def command_add(args):
    if 6 > len(args.pose) or len(args.pose) > 7:
        rospy.logerr("arguments of option 'pose' must be 6 or 7")
        exit(1)
    model = generate_camera_model(args.camera_ns, args.rate, args.format,
                                  args.width, args.height,
                                  args.near, args.far, args.hfov)
    result = spawn_camera_model(args.name, model,
                                args.ros_ns, args.gazebo_ns,
                                args.pose, args.ref_frame)
    if result is True:
        rospy.loginfo("camera: %s is spawned under ns: %s" % (args.name, args.gazebo_ns))
        rospy.loginfo("(w: %d, h: %d, fps: %f, near: %f, far: %f, hfov: %f)"
                      % (args.width, args.height, args.rate, args.near, args.far, args.hfov))
        rospy.loginfo("camera image is published at /%s/%s/image_raw" % (args.ros_ns, args.camera_ns))
        exit(0)
    else:
        rospy.logerr("failed to spawn camera")
        exit(1)

def command_remove(args):
    rospy.logerr("not implemented")
    exit(1)

if __name__ == '__main__':
    rospy.init_node("gazebo_spawn_scene_camera")
    parser = argparse.ArgumentParser(description="control scene camera on gazebo / ROS")
    parser.add_argument("cmd")
    parser.add_argument("--ros-ns", default=rospy.get_param("~ros_namespace", rospy.get_namespace()),
                        help="CAMERA_NS/image_raw is published under given namespace on ros")

    parser.add_argument("--gazebo-ns", default=rospy.get_param("~gazebo_namespace", "/gazebo"),
                        help="model is spawned under given namespace on gazebo")
    parser.add_argument("--name", default=rospy.get_param("~gazebo_name", random_string(5, "scene_camera_")),
                        help="model name id on gazebo")

    parser.add_argument("--ref-frame", default=rospy.get_param("~reference_frame", ""),
                        help="asssoc parent link")
    parser.add_argument("--camera-ns", metavar="CAMERA_NS",
                        default=rospy.get_param("~camera_namespace", "rgb"),
                        help="namespace of camera")
    parser.add_argument("--rate", type=float, default=rospy.get_param("~frame_rate", 1.0),
                        help="frame rate of camera image")
    parser.add_argument("--pose", type=float, nargs='*',
                        default=rospy.get_param("~pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                        help="pose of camera")
    parser.add_argument("--format", type=str, default=rospy.get_param("~format", "R8G8B8"),
                        help="format of camera image")
    parser.add_argument("--width", type=int, default=rospy.get_param("~width", 800),
                        help="width of camera image")
    parser.add_argument("--height", type=int, default=rospy.get_param("~height", 600),
                        help="height of camera image")
    parser.add_argument("--near", type=float, default=rospy.get_param("~near_clip", 0.1),
                        help="near clip of camera")
    parser.add_argument("--far", type=float, default=rospy.get_param("~far_clip", 100.0),
                        help="far clip of camera")
    parser.add_argument("--hfov", type=float, default=rospy.get_param("~horizontal_fov", 1.047),
                        help="horizontal fov of camera")
    args = parser.parse_args(rospy.myargv()[1:])

    if args.cmd == "add":
        command_add(args)
    elif args.cmd == "remove":
        command_remove(args)
    else:
        rospy.logerr("available commands: add, remove")
        exit(1)
