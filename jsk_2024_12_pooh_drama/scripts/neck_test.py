#!/usr/bin/env python

from __future__ import print_function

import time
import argparse
import numpy as np
import IPython
from kxr_controller.pooh_interface import PoohROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
import rospy
from skrobot.model import RobotModel

rospy.init_node('kxr_interface', anonymous=True)

namespace = ''
download_urdf_mesh_files(namespace)

robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    namespace + '/robot_description_viz')
ri = PoohROSRobotInterface(  # NOQA
    robot_model, namespace=namespace, controller_timeout=60.0)

#define nod, disagree, tilt and then define test
def nod(send_time = 1):
    controller_type = 'head_controller'
    robot_model.head_neck_p.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(1)
    robot_model.head_neck_p.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)

def disagree(send_time = 1):
    controller_type = 'head_controller'
    robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(1)
    robot_model.head_neck_y.joint_angle(np.deg2rad(-30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(1)
    robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(1)
    robot_model.head_neck_y.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)

def tilt(send_time = 1):
    controller_type = 'head_controller'
    robot_model.head_neck_r.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(2)                                                                                   
    robot_model.head_neck_r.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    
    
def test():
    ri.servo_on()
    ri.angle_vector(robot_model.init_pose())
    time.sleep(1)
    nod()
    time.sleep(2)
    disagree()
    time.sleep(2)
    tilt()
