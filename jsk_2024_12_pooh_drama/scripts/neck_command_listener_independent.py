#!/usr/bin/env python3

from __future__ import print_function

import time
import rospy
from std_msgs.msg import String
from kxr_controller.pooh_interface import PoohROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
from skrobot.model import RobotModel
import numpy as np

# ROSノードの初期化
rospy.init_node('neck_motion_control', anonymous=True)

# URDFのダウンロードとロボットモデルの初期化
namespace = ''
download_urdf_mesh_files(namespace)
robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    namespace + '/robot_description_viz')
rospy.loginfo("Init Real Robot Interface")
ri = PoohROSRobotInterface(robot_model, namespace=None, controller_timeout=60.0)
rospy.loginfo("Init Real Robot Interface Done")

# サーボをONにし、init_pose
ri.servo_on()
rospy.sleep(1.0)
ri.send_stretch(30)
rospy.sleep(1.0)
ri.angle_vector(robot_model.init_pose())
rospy.loginfo('init_pose')

# nod動作
def nod(send_time=1):
    controller_type = 'head_controller'
    ri.angle_vector(robot_model.init_pose(),
                    controller_type=controller_type)
    robot_model.head_neck_p.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_p.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

# disagree動作
def disagree(send_time=1):
    controller_type = 'head_controller'
    ri.angle_vector(robot_model.init_pose(),
                    controller_type=controller_type)
    robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_y.joint_angle(np.deg2rad(-30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    #robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    #ri.angle_vector(robot_model.angle_vector(), send_time)
    #ri.wait_interpolation()
    robot_model.head_neck_y.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

# tilt動作
def tilt(send_time=1):
    controller_type = 'head_controller'
    ri.angle_vector(robot_model.init_pose(),
                    controller_type=controller_type)
    robot_model.head_neck_r.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_r.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

# コールバック関数
def neck_motion_callback(msg):
    command = msg.data.lower()
    if command == 'nod':
        rospy.loginfo('Executing nod motion')
        nod()
    elif command == 'disagree':
        rospy.loginfo('Executing disagree motion')
        disagree()
    elif command == 'tilt':
        rospy.loginfo('Executing tilt motion')
        tilt()
    elif command == 'test':  # testメッセージに対応
        rospy.loginfo('Executing test motions')
        test()
    else:
        rospy.logwarn(f'Unknown command received: {command}')
        
# テスト用関数
def test():
    ri.angle_vector(robot_model.init_pose())
    ri.wait_interpolation()
    nod()
    ri.wait_interpolation()    
    disagree()
    ri.wait_interpolation()    
    tilt()

# メイン関数
def main():
    # /neck_motionトピックを購読し、neck_motion_callback関数をコールバックに指定
    rospy.Subscriber('/neck_motion', String, neck_motion_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
