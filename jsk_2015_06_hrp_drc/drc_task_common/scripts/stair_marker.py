#!/usr/bin/env python
#####################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, JSK Lab
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/o2r other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import rospy
import os
from visualization_msgs.msg import Marker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PoseArray, Pose
from interactive_markers.menu_handler import *

import tf
from tf.transformations import *

max_x = 0.3
max_y = 0.25
max_z = 0.28
min_z = 0
def poseMsgToMatrix(pose):
    return concatenate_matrices(translation_matrix([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]),
                                quaternion_matrix([pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w]))

def poseMatrixToMsg(mat):
    translation = translation_from_matrix(mat)
    quaternion = quaternion_from_matrix(mat)
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def absmin(a, b):
    if abs(a) > abs(b):
        return a
    else:
        return b

def processFeedback(feedback):
    (frame_transform_pos, frame_transform_rot) = tf_listener.lookupTransform(
        lleg_end_coords, feedback.header.frame_id, rospy.Time(0.0))
    frame_pose = concatenate_matrices(translation_matrix(frame_transform_pos),
                                      quaternion_matrix(frame_transform_rot))
    center_local_pose = poseMsgToMatrix(feedback.pose)
    center_local_pos = translation_from_matrix(center_local_pose)
    left_local_offset = translation_matrix([0, foot_margin / 2.0, 0])
    left_local_pose = concatenate_matrices(center_local_pose, left_local_offset)
    right_local_offset = translation_matrix([0, - foot_margin / 2.0, 0])
    right_local_pose = concatenate_matrices(center_local_pose,
                                            right_local_offset)
    left_global_pose = concatenate_matrices(frame_pose, left_local_pose)
    right_global_pose = concatenate_matrices(frame_pose, right_local_pose)

    left_global_pos = translation_from_matrix(left_global_pose)
    right_global_pos = translation_from_matrix(right_global_pose)
    footsteps = PoseArray()
    footsteps.header.frame_id = lleg_end_coords
    footsteps.header.stamp = feedback.header.stamp
    footsteps.poses = [poseMatrixToMsg(right_global_pose),
                       poseMatrixToMsg(left_global_pose)]
    pub_debug_current_pose_array.publish(footsteps)
    # check distance
    need_to_fix = False
    if (feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP and 
        (abs(center_local_pos[0]) + 0.01> max_x or
         abs(center_local_pos[2]) + 0.01> max_z or
         center_local_pos[2] - 0.01 < 0)):
        if abs(center_local_pos[0]) > max_x:
            center_local_pos[0] = max_x * center_local_pos[0] / abs(center_local_pos[0])
        if center_local_pos[2] < 0:
            center_local_pos[2] = 0
        elif abs(center_local_pos[2]) > max_z:
            center_local_pos[2] = max_z
        rospy.logwarn("need to reset")
        new_center_pose = translation_matrix(center_local_pos)
        server.setPose(feedback.marker_name, poseMatrixToMsg(new_center_pose))
        server.applyChanges()
    elif feedback.menu_entry_id == 0:
        return                  # do nothing
    elif feedback.menu_entry_id == 1: # reset to origin
        server.setPose(feedback.marker_name, poseMatrixToMsg(identity_matrix()))
        server.applyChanges()
        return
    elif feedback.menu_entry_id == 2: # reset orientation to origin
        position = [feedback.pose.position.x,
                    feedback.pose.position.y,
                    feedback.pose.position.z]
        server.setPose(feedback.marker_name, poseMatrixToMsg(translation_matrix(position)))
        server.applyChanges()
        return
    elif feedback.menu_entry_id == 3: # execute
        pub_goal.publish(footsteps)

def make6DOFControls():
    translation_x_control = InteractiveMarkerControl()
    translation_x_control.name = "move_x"
    translation_x_control.orientation.w = 1
    translation_x_control.orientation.x = 1
    translation_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    translation_y_control = InteractiveMarkerControl()
    translation_y_control.name = "move_y"
    translation_y_control.orientation.w = 1
    translation_y_control.orientation.y = 1
    translation_y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    translation_z_control = InteractiveMarkerControl()
    translation_z_control.name = "move_z"
    translation_z_control.orientation.w = 1
    translation_z_control.orientation.z = 1
    translation_z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    rotation_x_control = InteractiveMarkerControl()
    rotation_x_control.name = "rotate_x"
    rotation_x_control.orientation.w = 1
    rotation_x_control.orientation.x = 1
    rotation_x_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotation_y_control = InteractiveMarkerControl()
    rotation_y_control.name = "rotate_y"
    rotation_y_control.orientation.w = 1
    rotation_y_control.orientation.y = 1
    rotation_y_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotation_z_control = InteractiveMarkerControl()
    rotation_z_control.name = "rotate_z"
    rotation_z_control.orientation.w = 1
    rotation_z_control.orientation.z = 1
    rotation_z_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    
    return [translation_x_control, translation_y_control,
            translation_z_control,
            rotation_x_control, rotation_y_control,
            rotation_z_control]

def makeFootCube(x, y, offset = identity_matrix()):
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = x
    box_marker.scale.y = y
    box_marker.scale.z = 0.001
    box_marker.color.a = 1.0
    box_marker.pose = poseMatrixToMsg(offset)
    return box_marker

if __name__ == "__main__":
    rospy.init_node("stair_marker")
    pub_debug_current_pose_array = rospy.Publisher("~debug/current_pose_array",
                                                   PoseArray)
    pub_goal = rospy.Publisher("~output", PoseArray)
    menu_handler = MenuHandler()
    menu_handler.insert("Reset to origin", callback=processFeedback)
    menu_handler.insert("Reset to orientation only", callback=processFeedback)
    menu_handler.insert("Execute", callback=processFeedback)
    server = InteractiveMarkerServer("stair_marker")
    tf_listener = tf.TransformListener()
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/ground"
    int_marker.name = "stair_marker"
    int_marker.description = "Stair footstep marker"
    if os.environ["ROBOT"] == "JAXON":
        foot_margin = 0.20
        left_offset = translation_matrix([0.015, 0.01 + foot_margin / 2.0, 0])
        right_offset = translation_matrix([0.015, -0.01 - foot_margin / 2.0, 0])
        foot_depth = 0.240
        foot_width = 0.140
    elif os.environ["ROBOT"] == "JAXON_RED":
        foot_margin = 0.20
        left_offset = translation_matrix([0.0, 0.01 + foot_margin / 2.0, 0])
        right_offset = translation_matrix([0.0, -0.01 - foot_margin / 2.0, 0])
        foot_depth = 0.225
        foot_width = 0.140
    else:
        foot_margin = 0.21
        left_offset = translation_matrix([0.01, 0.02 + foot_margin / 2.0, 0])
        right_offset = translation_matrix([0.01, -0.02 - foot_margin / 2.0, 0])
        foot_depth = 0.240
        foot_width = 0.140
    lleg_end_coords = "lleg_end_coords"
    rleg_end_coords = "rleg_end_coords"
    # create a grey box marker
    lleg_marker = makeFootCube(foot_depth, foot_width, left_offset)
    rleg_marker = makeFootCube(foot_depth, foot_width, right_offset)
    lleg_marker.color.g = 1.0
    rleg_marker.color.r = 1.0
    # rleg_marker.pose.position.y = - foot_margin / 2.0
    # lleg_marker.pose.position.y = foot_margin / 2.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.extend([lleg_marker, rleg_marker])

    # add the control to the interactive marker
    int_marker.controls.append( box_control )


    int_marker.controls.extend(make6DOFControls())

    

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)
    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()

