#!/usr/bin/env python3


import rospy
import sensor_msgs.msg

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
from skrobot.model import RobotModel


def get_namespace():
    full_namespace = rospy.get_namespace()
    last_slash_pos = full_namespace.rfind('/')
    clean_namespace = full_namespace[:last_slash_pos] \
        if last_slash_pos != 0 else ''
    return clean_namespace


class JoyMap(object):

    def __init__(self):
        namespace = get_namespace()
        download_urdf_mesh_files(namespace)
        robot_model = RobotModel()
        robot_model.load_urdf_from_robot_description(
            namespace + '/robot_description_viz')
        ri = KXRROSRobotInterface(  # NOQA
            robot_model, namespace=namespace, controller_timeout=60.0)
        self.ri = ri
        self.robot_model = robot_model

        self.sub = rospy.Subscriber(
            'joy',
            sensor_msgs.msg.Joy,
            queue_size=1,
            callback=self.callback)

    def callback(self, msg):
        if len(msg.buttons) >= 2 and msg.buttons[2] == 1:
            self.ri.servo_on()
        elif len(msg.buttons) >= 3 and msg.buttons[3] == 1:
            self.ri.servo_off()


if __name__ == '__main__':
    rospy.init_node('joy_map')
    act = JoyMap()
    rospy.spin()
