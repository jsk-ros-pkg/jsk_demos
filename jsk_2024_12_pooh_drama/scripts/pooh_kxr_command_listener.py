#!/usr/bin/env python3

from __future__ import print_function

import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from kxr_controller.pooh_interface import PoohROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
from skrobot.model import RobotModel
import numpy as np
import random

speak_flag = False

def callback(msg):
    global speak_flag
    speak_flag = msg.data

# ROSノードの初期化
rospy.init_node('pooh_motion_control', anonymous=True)

sub = rospy.Subscriber("/is_speaking", Bool, queue_size=1,
                       callback=callback)

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
ri.send_stretch(10)
rospy.sleep(1.0)
ri.angle_vector(robot_model.init_pose())
rospy.loginfo('init_pose')

# nod動作
def nod(send_time=1):
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    robot_model.head_neck_p.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_p.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

def natural_nod():
    global speak_flag
    controller_type = 'head_controller'

    rate = rospy.Rate(10)
    rospy.sleep(1.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        send_time = 0.5*random.randint(1,3)
        rospy.sleep(0.5)
        robot_model.head_neck_p.joint_angle(np.deg2rad(20))
        ri.angle_vector(robot_model.angle_vector(), send_time,
                        controller_type=controller_type)
        ri.wait_interpolation()
        robot_model.head_neck_p.joint_angle(np.deg2rad(0))
        ri.angle_vector(robot_model.angle_vector(), send_time,
                        controller_type=controller_type)
        ri.wait_interpolation()
        rate.sleep()
    rospy.loginfo("while end")
# disagree動作
def disagree(send_time=0.5):
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    robot_model.head_neck_y.joint_angle(np.deg2rad(20))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_y.joint_angle(np.deg2rad(-20))
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
    global speak_flag
    rospy.loginfo("tilt")
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    ret = random.randint(0,1)
    if ret == 0:
        robot_model.head_neck_r.joint_angle(np.deg2rad(20))
    elif ret == 1:
        robot_model.head_neck_r.joint_angle(np.deg2rad(-20))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")        
    robot_model.head_neck_r.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

def right_hand_up(send_time=1):
    controller_type='rarm_controller'
    ri.angle_vector([-0.02238367,  0.19497527,  0.00824686, -1.7 ,  0.9024227 ,-1.1356856 ,  0.27862018,  0.33399075, -0.28627747], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    rospy.sleep(2.0)
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()

def left_hand_chin():
    global speak_flag
    controller_type='larm_controller'
    ri.angle_vector([-0.12370004,  0.2727297 , -0.01943843, -0.9283405 ,  0.0111921, 0.03063071,  0.2049891 ,  1.3224144 , -1.5380058 ], 2, controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 3, controller_type=controller_type)
    ri.wait_interpolation()

def right_hand_mouth():
    global speak_flag
    controller_type='rarm_controller'                                          
    ri.angle_vector([ 0.034165  ,  0.26742825,  0.00824686, -1 , -1.4095932 ,0.603775  ,  0.09248082, -0.00530126, -0.00530126], 2, controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 3, controller_type=controller_type)                
    ri.wait_interpolation()

def banzai(send_time=1):
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.4948007 ,  0.9960814 ], send_time, controller_type='rarm_controller')
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.4948007 ,  0.9960814 ], send_time, controller_type='larm_controller')
    ri.wait_interpolation()
    rospy.sleep(2.0)
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type='rarm_controller')
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type='larm_controller')
    ri.wait_interpolation()

def left_hand_up(send_time=1):
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.83 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.4948007 ,  0.9960814 ], send_time, controller_type='larm_controller')
    ri.wait_interpolation()
    rospy.sleep(2.0)
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type='larm_controller')
    ri.wait_interpolation()

def start_shake(send_time=1):
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  6.6680324e-01,  1.1892895e+00,-3.0335987e-01], send_time, controller_type='larm_controller') 
    ri.wait_interpolation()

def init_pose(send_time=1):
    ri.angle_vector(robot_model.init_pose(), send_time,                                                                                                                                                   
                    controller_type='larm_controller')                                                                                                                                                    
    ri.wait_interpolation()

def janken(send_time=0.4):
    controller_type='larm_controller'
    rospy.sleep(1.0)
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.937971,  0.4,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    rospy.sleep(1.0)
    for i in range(2):
        ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  0.8,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
        ri.wait_interpolation()
        ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.937971,  0.4,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
        ri.wait_interpolation()
    #ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  0.4,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)  
    #ri.wait_interpolation()

def aiko(send_time=0.5):
    controller_type='larm_controller'
    
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  0.8,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.937971,  0.4,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
    ri.wait_interpolation()                                                                                                                                                                           



def onegai():
    global speak_flag
    ri.angle_vector([-0.00530126,  0.23915392,  0.00235637, -1.1121236, -0.84587365, 0.8623674 ,  0.2049891 ,  1.0979868 , -1.0049168 ], 2, controller_type='rarm_controller')
    ri.angle_vector([-0.00530126,  0.23915392,  0.00235637, -1.1121236 , -0.84587365, 0.8623674 ,  0.2049891 ,  1.0979868 , -1.0049168 ], 2, controller_type='larm_controller')


    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 2, controller_type='rarm_controller')
    ri.angle_vector(robot_model.init_pose(), 2, controller_type='larm_controller')
    ri.wait_interpolation()

def left_hand_point(send_time=1):
    controller_type = 'larm_controller'
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  6.6680324e-01,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector(robot_model.init_pose(), send_time, controller_type='larm_controller')
    ri.wait_interpolation()

def right_hand_bye(send_time=0.5):
    controller_type = 'rarm_controller'
    ri.angle_vector([ 1.7555017e-07,  2.4504441e-01,  8.2468567e-03, -1.7,9.1302715e-02, -1.3177017e+00,  1.3783756e-01,  5.3016134e-03,5.3016134e-03], 2, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([ 1.7555017e-07,  2.5034586e-01,  8.2468567e-03, -1.7,5.4840446e-01, -5.4545885e-01,  1.3194707e-01,  5.3016134e-03,5.3016134e-03], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([ 1.7555017e-07,  2.4504441e-01,  8.2468567e-03, -1.7,9.1302715e-02, -1.3177017e+00,  1.3783756e-01,  5.3016134e-03,5.3016134e-03], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([ 1.7555017e-07,  2.5034586e-01,  8.2468567e-03, -1.7,5.4840446e-01, -5.4545885e-01,  1.3194707e-01,  5.3016134e-03,5.3016134e-03], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([ 1.7555017e-07,  2.4504441e-01,  8.2468567e-03, -1.7,9.1302715e-02, -1.3177017e+00,  1.3783756e-01,  5.3016134e-03,5.3016134e-03], send_time, controller_type=controller_type)
    ri.wait_interpolation()

    ri.angle_vector(robot_model.init_pose(), 2, controller_type=controller_type)
    ri.wait_interpolation()

def left_hand_bye(send_time=0.5):
    controller_type = 'larm_controller'
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.3 ,  0.6960814 ], 2, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.1 ,  0.9960814 ], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.34553 , -0.3 ,  0.6960814 ], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.1 ,  0.9960814 ], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector([-0.01119175,  0.07245316, -0.01943843, -1.7 ,  0.6114327 ,-1.1828095 ,  1.3094553 , -0.3 ,  0.6960814 ], send_time, controller_type=controller_type)
    ri.wait_interpolation()

    ri.angle_vector(robot_model.init_pose(), 2, controller_type=controller_type)
    ri.wait_interpolation()







    
# コールバック関数
def neck_motion_callback(msg):
    command = msg.data.lower()
    if command == 'nod':
        rospy.loginfo('Executing nod motion')
        nod()
    elif command == 'natural_nod':
        rospy.loginfo('Executing natural_nod motion')
        natural_nod()
    elif command == 'disagree':
        rospy.loginfo('Executing disagree motion')
        disagree()
    elif command == 'tilt':
        rospy.loginfo('Executing tilt motion')
        tilt()
    elif command == 'test':  # testメッセージに対応
        rospy.loginfo('Executing test motions')
        test()
    elif command == '':
        rospy.loginfo('no neck motion')
    else:
        rospy.logwarn(f'Unknown command received: {command}')

def arm_motion_callback(msg):
    command = msg.data.lower()
    if command == 'right_hand_up':
        rospy.loginfo('Executing right_hand_up motion')
        right_hand_up()
    elif command == 'left_hand_chin':
        rospy.loginfo('Executing left_hand_chin motion')
        left_hand_chin()
    elif command == 'right_hand_mouth':
        rospy.loginfo('Executing right_hand_mouth motion')
        right_hand_mouth()
    elif command == 'banzai':  
        rospy.loginfo('Executing banzai motions')
        banzai()
    elif command == 'onegai':
        rospy.loginfo('Executing onegai motion')
        onegai()
    elif command == 'left_hand_point':
        rospy.loginfo('Executing left_hand_point motion')
        left_hand_point()
    elif command == 'right_hand_bye':
        rospy.loginfo('Executing right_hand_bye motion')
        right_hand_bye()
    elif command == 'left_hand_up':
        rospy.loginfo('Executing left_hand_up motion')
        left_hand_up()
    elif command == 'left_hand_bye':
        rospy.loginfo('Executing left_hand_bye motion')
        left_hand_bye()
    elif command == 'janken':                                                                                                                                                                      
        rospy.loginfo('Executing janken motion')                                                                                                                                                    
        janken()
    elif command == 'aiko':                                                                                                                                                                      
        rospy.loginfo('Executing aiko motion')                                                                                                                                                   
        aiko()
    elif command == 'start_shake':                                                                                                                                                                      
        rospy.loginfo('Executing start_shake motion')                                                                                                                                                    
        start_shake()
    elif command == 'init_pose':                                                                                                                                                                      
        rospy.loginfo('Executing init_pose motion')                                                                                                                                                    
        init_pose() 
    elif command == '':
        rospy.loginfo('no arm motion')
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
    rospy.Subscriber('/arm_motion', String, arm_motion_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


