#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import numpy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from dynamic_tf_publisher.srv import SetDynamicTF
from tf import transformations

def get_4x4_mat(mat):
    return numpy.array((
        (mat[0][0], mat[0][1], mat[0][2], 0.0),
        (mat[1][0], mat[1][1], mat[1][2], 0.0),
        (mat[2][0], mat[2][1], mat[2][2], 0.0),
        (0.0, 0.0, 0.0, 1.0)
    ))
def pose_cb(pose_msg):
    trans = TransformStamped()
    trans.child_frame_id = "handle_orig"
    trans.header = pose_msg.header
    trans.transform.rotation = pose_msg.pose.orientation
    trans.transform.translation.x = pose_msg.pose.position.x
    trans.transform.translation.y = pose_msg.pose.position.y
    trans.transform.translation.z = pose_msg.pose.position.z
    set_tf(10, trans) # orig

    try:
        transed_pose = listener.transformPose("BODY", pose_msg)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print("tf error: %s" % e)
        return
    trans.header = transed_pose.header
    trans.child_frame_id = "handle"
    trans.transform.translation.x = transed_pose.pose.position.x
    trans.transform.translation.y = transed_pose.pose.position.y
    trans.transform.translation.z = transed_pose.pose.position.z
    trans_qua = numpy.array([transed_pose.pose.orientation.x, transed_pose.pose.orientation.y, transed_pose.pose.orientation.z,  transed_pose.pose.orientation.w])
    trans_matrix = transformations.quaternion_matrix(trans_qua)
    uz = numpy.array([trans_matrix[0][2], trans_matrix[1][2], trans_matrix[2][2]])
    ux = numpy.array([1.0, 0.0, 0.0])
    uy = numpy.cross(uz ,ux)/numpy.linalg.norm(numpy.cross(uz ,ux))
    ux = numpy.cross(uy, uz)
    trans_qua = transformations.quaternion_from_matrix(get_4x4_mat(numpy.array([ux, uy, uz]).T))
    trans.transform.rotation.x = trans_qua[0]
    trans.transform.rotation.y = trans_qua[1]
    trans.transform.rotation.z = trans_qua[2]
    trans.transform.rotation.w = trans_qua[3]
    set_tf(10, trans) # handle
    trans_handle_mat = get_4x4_mat(numpy.array([ux, uy, uz]).T)
    trans_handle_mat[0][3] = transed_pose.pose.position.x
    trans_handle_mat[1][3] = transed_pose.pose.position.y
    trans_handle_mat[2][3] = transed_pose.pose.position.z
    theta = numpy.deg2rad(-30)
    trans_mat = numpy.array((
        (numpy.cos(theta), 0, numpy.sin(theta), 0),
        (0, 1, 0, 0),
        (-numpy.sin(theta), 0 , numpy.cos(theta), -1.0),
        (0.0, 0.0, 0.0, 1.0)
        ))
    car_mat = numpy.dot(trans_handle_mat, trans_mat)

    car_qua = transformations.quaternion_from_matrix(car_mat)
    trans.child_frame_id = "car"
    trans.transform.translation.x = car_mat[0][3]
    trans.transform.translation.y = car_mat[1][3]
    trans.transform.translation.z = car_mat[2][3]
    trans.transform.rotation.x = car_qua[0]
    trans.transform.rotation.y = car_qua[1]
    trans.transform.rotation.z = car_qua[2]
    trans.transform.rotation.w = car_qua[3]
    set_tf(10, trans) # car
    
if __name__ == "__main__":
    rospy.init_node('calc_drive_tf', anonymous=True)
    # posepub = rospy.Publisher('plane_centroid_pose', PoseStamped)
    listener = tf.TransformListener()
    set_tf = rospy.ServiceProxy('/set_dynamic_tf', SetDynamicTF)
    rospy.Subscriber("pose", PoseStamped,  pose_cb)
    rospy.spin()
