#!/usr/bin/env python

from __future__ import print_function

import time
import numpy as np
import rospy
from kxr_controller.pooh_interface import PoohROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
from skrobot.model import RobotModel
from std_msgs.msg import String  # メッセージはString型と仮定

# URDFのダウンロードとモデルの初期化
namespace = ''
download_urdf_mesh_files(namespace)

robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    namespace + '/robot_description_viz')

ri = PoohROSRobotInterface(  # NOQA
    robot_model, namespace=namespace, controller_timeout=60.0)

# 動作の定義
def nod(send_time=1):
    controller_type = 'head_controller'
    robot_model.head_neck_p.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(1)
    robot_model.head_neck_p.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)

def disagree(send_time=1):
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

def tilt(send_time=1):
    controller_type = 'head_controller'
    robot_model.head_neck_r.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    time.sleep(2)                                                                                   
    robot_model.head_neck_r.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)

# コールバック関数: /neck_motionのメッセージを受け取り対応する動作を実行
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

# 動作のテスト関数（初期化後に動作確認する場合）
def test():
    ri.servo_on()
    ri.angle_vector(robot_model.init_pose())
    time.sleep(1)
    nod()
    time.sleep(2)
    disagree()
    time.sleep(2)
    tilt()

def main():
    #rospy.init_node('neck_motion_control', anonymous=True)
    
    # /neck_motionトピックを購読し、neck_motion_callback関数をコールバックに指定
    rospy.Subscriber('/neck_motion', String, neck_motion_callback)
    
    # サーボをオンにして初期ポーズを設定
    ri.servo_on()
    ri.angle_vector(robot_model.init_pose())
    
    # ROSスピン: ノードが終了するまでコールバックを待ち続ける
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
