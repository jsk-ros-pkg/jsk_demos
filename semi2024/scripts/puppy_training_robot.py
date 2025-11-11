#!/usr/bin/env python3

from skrobot.viewers import PyrenderViewer
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np  # NOQA
import rospy
import os
from skrobot.model import RobotModel
import time
from jsk_recognition_msgs.msg import BoundingBoxArray
from skrobot.utils.urdf import no_mesh_load_mode

# グローバル変数
found_dog_flag = False
dog_found_start_time = None
action_executed = False


# 環境変数から取得する場合（任意）
aibo_access_token = os.getenv("AIBO_ACCESS_TOKEN")
aibo_device_id = os.getenv("AIBO_DEVICE_ID")

rospy.init_node("kxr_interface", anonymous=True)
namespace = ''
robot_description = namespace + "/robot_description"
robot_model = RobotModel()
with no_mesh_load_mode():
    robot_model.load_urdf_from_robot_description(robot_description)
ri = KXRROSRobotInterface(
    robot_model, namespace=namespace, controller_timeout=100.0
)
ri.servo_on()
viewer = PyrenderViewer(resolution=(640, 480))
viewer.add(robot_model)
viewer.show()

def wait(wait_time):
    rospy.sleep(wait_time)

# for i in range(1):
#     hajimeno_pose = np.array([ 1.6316666e-01, -1.4372769e-01, -3.4164645e-02, -1.3547943e-02,
#         7.6578078e-03,  1.4955947e+00, -5.3014204e-02, -5.3014204e-02,
#         5.3012623e-03,  1.6493538e-02, -1.7512414e+00,  2.7685462e-02,
#         2.7685462e-02,  1.1427561e-01,  8.4528464e-01,  2.6507192e+00,
#        -1.0455616e+00, -2.3560191e-03], dtype=np.float32)
#     robot_model.angle_vector(hajimeno_pose)
#     ri.angle_vector(robot_model.angle_vector(),0.2)
#     ri.wait_interpolation()
#     nageteru_dainino_pose = np.array([ 1.6316666e-01,  7.8638011e-01, -3.4164645e-02, -1.3547943e-02,
#         1.3548294e-02,  1.5014852e+00, -5.3014204e-02, -5.3014204e-02,
#         3.4753695e-02, -2.2383673e-02,  1.2546754e-01,  2.7685462e-02,
#         2.7685462e-02,  1.0838512e-01,  8.4528464e-01,  2.6507192e+00,
#        -1.0455616e+00, -2.3560191e-03], dtype=np.float32)
#     robot_model.angle_vector(nageteru_dainino_pose)
#     ri.angle_vector(robot_model.angle_vector(),0.2)
#     ri.wait_interpolation()

# for i in range(1):
#     hajimeno_dainino_pose = np.array([ 1.6316666e-01,  6.8329819e-02, -3.4164645e-02, -8.2465056e-03,
#         1.3548294e-02,  1.5014852e+00, -5.3014204e-02, -5.3014204e-02,
#        -2.2384023e-02,  1.3430327e-01,  1.5792396e+00,  1.6493538e-02,
#         1.6493538e-02,  1.1427561e-01,  8.4528464e-01,  2.6507192e+00,
#        -1.0514520e+00,  2.3563702e-03], dtype=np.float32)
#     robot_model.angle_vector(hajimeno_dainino_pose)
#     ri.angle_vector(robot_model.angle_vector(),0.3)
#     ri.wait_interpolation()
#     nageteru_daisannno_pose = np.array([ 1.6316666e-01, -2.3025911e+00, -2.8863208e-02, -8.2465056e-03,
#         1.3548294e-02,  1.5014852e+00, -5.3014204e-02, -5.3014204e-02,
#        -2.2384023e-02,  1.4549519e-01,  1.2287556e+00,  2.2384023e-02,
#         2.2384023e-02,  1.1427561e-01,  8.4528464e-01,  2.6507192e+00,
#        -1.0514520e+00, -2.3560191e-03], dtype=np.float32)
#     robot_model.angle_vector(nageteru_daisannno_pose)
#     ri.angle_vector(robot_model.angle_vector(),0.3)
#     ri.wait_interpolation()

def throw_motion1():
    serving_elbow = np.array([-0.7504478 ,  2.35      ,  0.034165  ,  0.087     ,  0.04594597,
       -1.6222398 , -0.03063035,  0.        , -1.5656915 ,  0.75162625,
        1.5338829 , -1.5138549 ,  0.17494762,  0.        ,  0.06832982,
        0.27037317,  2.6507192 , -1.4814575 , -0.29157892], dtype=np.float32)
    robot_model.angle_vector(serving_elbow)
    ri.angle_vector(robot_model.angle_vector(),1.5)
    ri.wait_interpolation()
    robot_model.larm_module3_joint1.joint_angle(np.deg2rad(-90))
    serving_wrist = np.array([-0.74514633,  2.3455918 ,  0.034165  ,  0.087     ,  0.04594597,
       -1.6222398 , -0.02473987,  0.        , -1.548609  ,  0.8022844 ,
        1.5733491 ,  1.6198839 ,  0.18024907,  0.        ,  0.07422031,
        0.27567458,  2.6507192 , -1.475567  , -0.29157892], dtype=np.float32)
    robot_model.angle_vector(serving_wrist)
    robot_model.larm_module3_joint1.joint_angle(np.deg2rad(90))
    ri.angle_vector(robot_model.angle_vector(),1)
    ri.wait_interpolation()
    serving_elbow = np.array([-0.7504478 ,  2.35      ,  0.034165  ,  0.087     ,  0.04594597,
       -1.6222398 , -0.03063035,  0.        , -1.5656915 ,  0.75162625,
        1.5338829 , -1.5138549 ,  0.17494762,  0.        ,  0.06832982,
        0.27037317,  2.6507192 , -1.4814575 , -0.29157892], dtype=np.float32)
    robot_model.angle_vector(serving_elbow)
    robot_model.left_hand_joint1.joint_angle(np.deg2rad(-30))
    ri.angle_vector(robot_model.angle_vector(),1)
    ri.wait_interpolation()
    before_serving = np.array([-0.7504478 ,  2.35      ,  0.034165  ,  0.087     ,  0.04594597,
       -1.6222398 , -0.02473987,  0.        , -1.57      ,  0.75162625,
       -1.5321153 , -1.5138549 ,  0.17494762,  0.        ,  0.06832982,
        0.27037317,  2.6507192 , -1.4814575 , -0.29157892], dtype=np.float32)
    robot_model.angle_vector(before_serving)
    robot_model.left_hand_joint1.joint_angle(np.deg2rad(-30))
    ri.angle_vector(robot_model.angle_vector(),2)
    ri.wait_interpolation()
    

def throw_motion2():
    hajimeno_daiyonnno_pose = np.array([ 1.6846809e-01,  2.3499999e+00, -2.2972722e-02, -8.2465056e-03,
        1.8849732e-02,  1.4955947e+00, -5.3014204e-02, -5.3014204e-02,
        4.0644180e-02,  5.0069310e-02, -1.4172509e+00,  1.4667329e-01,
        1.4667329e-01,  1.2016610e-01,  1.6198820e-01,  2.6507192e+00,
       -1.3530449e+00,  2.3563702e-03], dtype=np.float32) 
    robot_model.angle_vector(hajimeno_daiyonnno_pose)
    ri.angle_vector(robot_model.angle_vector(),0.2)
    ri.wait_interpolation()
    nageteru_daigono_pose = np.array([ 1.6846809e-01,  2.0481224e+00, -2.2972722e-02, -8.2465056e-03,
        1.8849732e-02,  1.4955947e+00, -5.8904689e-02, -5.8904689e-02,
        1.7082235e-02,  5.0069310e-02, -9.5720387e-01, -1.1191749e-02,
       -1.1191749e-02,  1.2016610e-01,  1.6198820e-01,  2.6507192e+00,
       -1.3589354e+00,  2.3563702e-03], dtype=np.float32)
    robot_model.angle_vector(nageteru_daigono_pose)
    ri.angle_vector(robot_model.angle_vector(),0.2)
    ri.wait_interpolation()

        
    nageteru_daiyonnno_pose = np.array([ 1.6316666e-01, -2.3131938e+00, -2.8863208e-02, -8.2465056e-03,
         1.3548294e-02,  1.4955947e+00, -5.3014204e-02, -5.3014204e-02,
        -5.3016134e-03,  1.7555017e-07,  8.2349020e-01,  2.7685462e-02,
         2.7685462e-02,  1.1427561e-01,  8.4528464e-01,  2.6507192e+00,
        -1.0455616e+00,  2.3563702e-03], dtype=np.float32)
    robot_model.angle_vector(nageteru_daiyonnno_pose)
    ri.angle_vector(robot_model.angle_vector(),0.3)
    ri.wait_interpolation()


class BoxFilter:
    def __init__(self, min_z=0.15, max_z=0.6, min_area=0.0015):
        self.min_z = min_z
        self.max_z = max_z
        self.min_area = min_area
        rospy.Subscriber('/boxes', BoundingBoxArray, self.boxes_callback)
        rospy.loginfo("BoxFilter 初期化完了")

    def boxes_callback(self, msg):
        global found_dog_flag, dog_found_start_time, action_executed

        dog_detected = False
        for box in msg.boxes:
            z = box.pose.position.z
            area = box.dimensions.x * box.dimensions.y
            if self.min_z <= z <= self.max_z and area >= self.min_area:
                dog_detected = True
                break

        if dog_detected:
            if not found_dog_flag:
                dog_found_start_time = time.time()
                rospy.loginfo("🐶 犬検出: タイマー開始")
            found_dog_flag = True
        else:
            found_dog_flag = False
            dog_found_start_time = None
            action_executed = False  # リセット

def check_dog_duration(event):
    global found_dog_flag, dog_found_start_time, action_executed
    required_duration = 3.0  # 秒
    if found_dog_flag and dog_found_start_time:
        elapsed = time.time() - dog_found_start_time
        rospy.loginfo(f"⏱ 犬検出中... 経過: {elapsed:.1f}s")
        if elapsed >= required_duration and not action_executed:
            # result = do_action("play_motion", '{\"Category\":\"marking\", \"Mode\":\"BOY\"}', aibo_access_token, aibo_device_id)
            throw_motion1()
            print("action has done")
            action_executed = True

if __name__ == '__main__':
    #rospy.init_node('box_filter_node_external_timer')
    robot_model.angle_vector(ri.angle_vector())
    for _ in range(3):
        robot_model.head_module1_joint1.joint_angle(0.0)
        ri.angle_vector(robot_model.angle_vector(),0.5)
        ri.wait_interpolation()
        robot_model.head_module1_joint1.joint_angle(np.deg2rad(-40))
        ri.angle_vector(robot_model.angle_vector(),0.5)
        ri.wait_interpolation()
    robot_model.head_module1_joint1.joint_angle(0.0)
    ri.angle_vector(robot_model.angle_vector(),0.5)
    ri.wait_interpolation()
    bf = BoxFilter()
    rospy.Timer(rospy.Duration(1.0), check_dog_duration)
    rospy.spin()
