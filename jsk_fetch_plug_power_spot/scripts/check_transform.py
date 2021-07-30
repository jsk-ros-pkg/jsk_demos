#!/usr/bin/env python

import rospy
import tf2_ros
import PyKDL

def main():

    rospy.init_node('hoge')

    socket_frame = 'socket_frame'
    plug_frame = 'plug_frame'
    holder_frame = 'holder_frame'
    eef_frame = 'gripper_link'

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans_plug_to_eef = \
                tf_buffer.lookup_transform(
                    plug_frame,
                    eef_frame,
                    rospy.Time(),
                    rospy.Duration(1)
                    )
        rot = PyKDL.Rotation.Quaternion(
                trans_plug_to_eef.transform.rotation.x,
                trans_plug_to_eef.transform.rotation.y,
                trans_plug_to_eef.transform.rotation.z,
                trans_plug_to_eef.transform.rotation.w
                )
        trans_x = trans_plug_to_eef.transform.translation.x * 1000
        trans_y = trans_plug_to_eef.transform.translation.y * 1000
        trans_z = trans_plug_to_eef.transform.translation.z * 1000
        rot_r = rot.GetRPY()[0]
        rot_p = rot.GetRPY()[1]
        rot_y = rot.GetRPY()[2]
        rospy.loginfo('trans_plug_to_eef')
        rospy.loginfo('  translation[m] (x,y,z) = ({} {} {})'.format(trans_x,trans_y,trans_z))
        rospy.loginfo('  rotation: (r,p,y) = ({} {} {})'.format(rot_r,rot_p,rot_y))
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr('{}'.format(e))

    try:
        trans_socket_to_eef = \
                tf_buffer.lookup_transform(
                    socket_frame,
                    eef_frame,
                    rospy.Time(),
                    rospy.Duration(1)
                    )
        rot = PyKDL.Rotation.Quaternion(
                trans_socket_to_eef.transform.rotation.x,
                trans_socket_to_eef.transform.rotation.y,
                trans_socket_to_eef.transform.rotation.z,
                trans_socket_to_eef.transform.rotation.w
                )
        trans_x = trans_socket_to_eef.transform.translation.x * 1000
        trans_y = trans_socket_to_eef.transform.translation.y * 1000
        trans_z = trans_socket_to_eef.transform.translation.z * 1000
        rot_r = rot.GetRPY()[0]
        rot_p = rot.GetRPY()[1]
        rot_y = rot.GetRPY()[2]
        rospy.loginfo('trans_socket_to_eef')
        rospy.loginfo('  translation[m] (x,y,z) = ({} {} {})'.format(trans_x,trans_y,trans_z))
        rospy.loginfo('  rotation: (r,p,y) = ({} {} {})'.format(rot_r,rot_p,rot_y))
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr('{}'.format(e))

    try:
        trans_holder_to_eef = \
                tf_buffer.lookup_transform(
                    holder_frame,
                    eef_frame,
                    rospy.Time(),
                    rospy.Duration(1)
                    )
        rot = PyKDL.Rotation.Quaternion(
                trans_holder_to_eef.transform.rotation.x,
                trans_holder_to_eef.transform.rotation.y,
                trans_holder_to_eef.transform.rotation.z,
                trans_holder_to_eef.transform.rotation.w
                )
        trans_x = trans_holder_to_eef.transform.translation.x * 1000
        trans_y = trans_holder_to_eef.transform.translation.y * 1000
        trans_z = trans_holder_to_eef.transform.translation.z * 1000
        rot_r = rot.GetRPY()[0]
        rot_p = rot.GetRPY()[1]
        rot_y = rot.GetRPY()[2]
        rospy.loginfo('trans_holder_to_eef')
        rospy.loginfo('  translation[m] (x,y,z) = ({} {} {})'.format(trans_x,trans_y,trans_z))
        rospy.loginfo('  rotation: (r,p,y) = ({} {} {})'.format(rot_r,rot_p,rot_y))
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr('{}'.format(e))


if __name__ == '__main__':
    main()
