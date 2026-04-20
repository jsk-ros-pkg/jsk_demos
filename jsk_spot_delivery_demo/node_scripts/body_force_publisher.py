#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse
import tf2_ros
import PyKDL


def calc_pykdl_wrench_from_ros_wrench(ros_wrench):
    return PyKDL.Wrench(
        PyKDL.Vector(
            ros_wrench.force.x,
            ros_wrench.force.y,
            ros_wrench.force.z
        ),
        PyKDL.Vector(
            ros_wrench.torque.x,
            ros_wrench.torque.y,
            ros_wrench.torque.z
        )
    )


def calc_pykdl_frame_from_ros_transform(transform):
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ),
        PyKDL.Vector(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        )
    )


class BodyForcePublisher(object):

    def callback_wrench_front_right(self, msg):
        self.wrench_front_right = calc_pykdl_wrench_from_ros_wrench(msg.wrench)

    def callback_wrench_front_left(self, msg):
        self.wrench_front_left = calc_pykdl_wrench_from_ros_wrench(msg.wrench)

    def callback_wrench_rear_right(self, msg):
        self.wrench_rear_right = calc_pykdl_wrench_from_ros_wrench(msg.wrench)

    def callback_wrench_rear_left(self, msg):
        self.wrench_rear_left = calc_pykdl_wrench_from_ros_wrench(msg.wrench)

    def handler_zero_offset(self, req):
        self.offset_wrench_body = self.current_wrench_body
        return EmptyResponse()

    def __init__(self):

        self.wrench_front_right = PyKDL.Wrench()
        self.wrench_front_left = PyKDL.Wrench()
        self.wrench_rear_right = PyKDL.Wrench()
        self.wrench_rear_left = PyKDL.Wrench()

        self.current_wrench_body = PyKDL.Wrench()
        self.offset_wrench_body = PyKDL.Wrench()

        self.pub_wrench = rospy.Publisher(
            '~wrench_output', WrenchStamped, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_front_right = rospy.Subscriber(
            '~input_wrench_front_right', WrenchStamped, self.callback_wrench_front_right)
        self.sub_front_left = rospy.Subscriber(
            '~input_wrench_front_left', WrenchStamped, self.callback_wrench_front_left)
        self.sub_rear_right = rospy.Subscriber(
            '~input_wrench_rear_right', WrenchStamped, self.callback_wrench_rear_right)
        self.sub_rear_left = rospy.Subscriber(
            '~input_wrench_rear_left', WrenchStamped, self.callback_wrench_rear_left)

        self.srv_zero_offset = rospy.Service(
            '~zero_offset', Empty, self.handler_zero_offset)

    def spin(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            rate.sleep()

            try:
                frame_body_to_front_right = calc_pykdl_frame_from_ros_transform(
                    self.tf_buffer.lookup_transform(
                        'body', 'front_right_lower_leg', rospy.Time()).transform
                )
                frame_body_to_front_left = calc_pykdl_frame_from_ros_transform(
                    self.tf_buffer.lookup_transform(
                        'body', 'front_left_lower_leg', rospy.Time()).transform
                )
                frame_body_to_rear_right = calc_pykdl_frame_from_ros_transform(
                    self.tf_buffer.lookup_transform(
                        'body', 'rear_right_lower_leg', rospy.Time()).transform
                )
                frame_body_to_rear_left = calc_pykdl_frame_from_ros_transform(
                    self.tf_buffer.lookup_transform(
                        'body', 'rear_left_lower_leg', rospy.Time()).transform
                )
            except Exception as e:
                # TODO
                rospy.logerr('hogehoge: {}'.format(e))
                continue

            wrench_front_right_bodybased = frame_body_to_front_right * self.wrench_front_right
            wrench_front_left_bodybased = frame_body_to_front_left * self.wrench_front_left
            wrench_rear_right_bodybased = frame_body_to_rear_right * self.wrench_rear_right
            wrench_rear_left_bodybased = frame_body_to_rear_left * self.wrench_rear_left

            wrench_body = -1 * (wrench_front_right_bodybased + wrench_front_left_bodybased
                                + wrench_rear_right_bodybased + wrench_rear_left_bodybased)

            self.current_wrench_body = wrench_body

            msg_pub = WrenchStamped()
            msg_pub.header.frame_id = 'body'
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.wrench.force.x = (
                wrench_body - self.offset_wrench_body).force.x()
            msg_pub.wrench.force.y = (
                wrench_body - self.offset_wrench_body).force.y()
            msg_pub.wrench.force.z = (
                wrench_body - self.offset_wrench_body).force.z()
            msg_pub.wrench.torque.x = (
                wrench_body - self.offset_wrench_body).torque.x()
            msg_pub.wrench.torque.y = (
                wrench_body - self.offset_wrench_body).torque.y()
            msg_pub.wrench.torque.z = (
                wrench_body - self.offset_wrench_body).torque.z()

            self.pub_wrench.publish(msg_pub)


def main():

    rospy.init_node('body_force_publisher')
    node = BodyForcePublisher()
    node.spin()


if __name__ == '__main__':
    main()
