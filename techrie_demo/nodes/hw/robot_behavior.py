#!/usr/bin/env python3
import os
import json
import numpy as np
import rospy
from skrobot.models.urdf import RobotModelFromURDF
from skrobot.coordinates import CascadedCoords
from skrobot.model import RobotModel
from skrobot.coordinates import Coordinates
from skrobot.utils.urdf import resolve_filepath
from skrobot.coordinates import Coordinates
import time
import random
from cached_property import cached_property
from jedy_play.jedy_interface import IJedyROSRobotInterface
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import UInt16, Bool, Float64MultiArray, String
from kxr_controller.msg import ServoOnOff
from sensor_msgs.msg import JointState
from skrobot.viewers import TrimeshSceneViewer
from skrobot.model import Axis


rospy.init_node("ichikura_jedy_model")


class IchikuraJedy(RobotModelFromURDF):

    def __init__(self, *args, **kwargs):
        super(IchikuraJedy, self).__init__(*args, **kwargs)
        self.rarm_end_coords = CascadedCoords(
            parent=self.rarm_link7,
            name='rarm_end_coords')
        self.rarm_end_coords.rotate(np.pi / 2, 'y')
        self.rarm_end_coords.translate([0.05, 0.0, 0.0])
        self.larm_end_coords = CascadedCoords(
            parent=self.larm_link7,
            name='larm_end_coords')
        self.larm_end_coords.rotate(np.pi / 2, 'y')
        self.larm_end_coords.translate([0.05, 0.0, 0.0])
        self.head_end_coords = CascadedCoords(
            parent=self.head_link1,
            name='head_end_coords')
        #self.head_end_coords.translate([0.0, 0.0, 0.06])
        #self.head_end_coords.rotate(2.0943951, [-0.57735, 0.57735, -0.57735])  # 対角方向に120度回転
        self.head_end_coords.rotate(np.pi / 2, 'y')
        #self.reset_pose()

    @cached_property
    def default_urdf_path(self):
        return resolve_filepath("", "package://kxr_humanoid_movebase_ichikura_version2/urdf/jedy.urdf")

    def reset_pose(self):
        self.rarm_joint0.joint_angle(np.deg2rad(-2))
        self.rarm_joint1.joint_angle(np.deg2rad(-10))
        self.rarm_joint2.joint_angle(np.deg2rad(-12))
        self.rarm_joint3.joint_angle(np.deg2rad(-43))
        self.rarm_joint4.joint_angle(np.deg2rad(0))
        self.rarm_joint5.joint_angle(np.deg2rad(33))
        self.rarm_joint6.joint_angle(np.deg2rad(6))
        self.larm_joint0.joint_angle(np.deg2rad(24))
        self.larm_joint1.joint_angle(np.deg2rad(8))
        self.larm_joint2.joint_angle(np.deg2rad(-11))
        self.larm_joint3.joint_angle(np.deg2rad(-26))
        self.larm_joint4.joint_angle(np.deg2rad(21))
        self.larm_joint5.joint_angle(np.deg2rad(40))
        self.larm_joint6.joint_angle(np.deg2rad(-8))
        self.head_joint0.joint_angle(np.deg2rad(90))

    @cached_property
    def rarm(self):
        link_names = ['rarm_link{}'.format(i) for i in range(0, 6)]
        links = [getattr(self, n) for n in link_names]
        joints = [l.joint for l in links]
        model = RobotModel(link_list=links, joint_list=joints)
        model.end_coords = self.rarm_end_coords
        return model

    @cached_property
    def larm(self):
        link_names = ['larm_link{}'.format(i) for i in range(0, 6)]
        links = [getattr(self, n) for n in link_names]
        joints = [l.joint for l in links]
        model = RobotModel(link_list=links, joint_list=joints)
        model.end_coords = self.larm_end_coords
        return model
    
    @cached_property
    def head(self):
        link_names = ['head_link{}'.format(i) for i in range(0, 2)]
        print(link_names)
        links = [getattr(self, n) for n in link_names]
        joints = [l.joint for l in links]
        model = RobotModel(link_list=links, joint_list=joints)
        model.end_coords = self.head_end_coords
        return model


robot_model = IchikuraJedy()
#robot_model.rotate(- np.pi / 2, 'z') #前を向くようにする


#viewer = TrimeshSceneViewer()
#viewer.add(robot_model)
#viewer.show()
axis = Axis.from_coords(robot_model.rarm_end_coords)
#viewer.add(axis)
#robot_model.rotate(np.pi / 2, 'x')


rospy.loginfo("Init Real Robot Interface")
ri = IJedyROSRobotInterface(robot_model, namespace=None,controller_timeout=10)
rospy.loginfo("Init Real Robot Interface Done")

led_pub = rospy.Publisher("/led_mode",UInt16,queue_size=1)
motion_done_pub = rospy.Publisher("/motion_done",Bool,queue_size=1)

def basic_led():
    mode_msg = UInt16()
    mode_msg.data = 3
    led_pub.publish(mode_msg)

def move_arm(send_time=1):
    rshoulder = random.uniform(-15, -5)
    relbow = random.uniform(-88,-78)
    rwrist = random.uniform(-5,5)
    lshoulder = random.uniform(10, 15)
    lelbow = random.uniform(-88,-78)
    lwrist = random.uniform(15,25)
    robot_model.rarm_joint1.joint_angle(np.deg2rad(rshoulder))
    robot_model.rarm_joint3.joint_angle(np.deg2rad(relbow))
    robot_model.rarm_joint4.joint_angle(np.deg2rad(rwrist))
    robot_model.larm_joint1.joint_angle(np.deg2rad(lshoulder))
    robot_model.larm_joint3.joint_angle(np.deg2rad(lelbow))
    robot_model.larm_joint4.joint_angle(np.deg2rad(lwrist))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type="rarm_controller")
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type="larm_controller")

def act(act_name, json_filepath, n_split=None):
    ri.servo_on()
    if os.path.exists(json_filepath):
        try:
            with open(json_filepath) as f:
                motion_dict = json.load(f)
        except json.JSONDecodeError as e:
            print(e)

        angles = [np.array(av) for av in motion_dict[act_name]["angles"]]
        time_stamps = motion_dict[act_name]["time_stamps"]
        speed = 1.0
        tms = []
        for prev_time, cur_time in zip(time_stamps[:-1], time_stamps[1:]):
            tms.append((cur_time - prev_time) / speed)
        if len(angles) > 0:
            ri.angle_vector(angles[0], 3)
            ri.wait_interpolation()
        ri.angle_vector_sequence(angles[1:], tms)
        ri.wait_interpolation()
    else:
        print("There is not such file.")

#IKを解くような関数
def solve_ik(coords):
    rospy.loginfo("start solving IK")
    robot_model.rarm.inverse_kinematics(
        target_coords=coords,
        rotation_axis=False,
        revert_if_fail=False
    )
    ri.angle_vector(robot_model.angle_vector(), 2.0)

def play_joint_frames_once(frame_buffer,frame_length=14,dt=0.01):
    ri.servo_on()
    
    frame_buffer = np.array(frame_buffer)
    nframes = len(frame_buffer) // frame_length
    print(f"Playing {nframes} frames with {int(dt * 1000)} msec interval")

    for i in range(nframes):
        start = i * frame_length
        end = (i + 1) * frame_length
        joint_angles = frame_buffer[start:end]
        print(joint_angles)
        # ロボットに角度を適用
        r_joint_angles = joint_angles[:7]
        l_joint_angles = joint_angles[7:15]
        #for k in range(len(l_joint_angles)):
        #    print(f"index {k}:{r_joint_angles[k]}")
        #    print(f"index {k}:{l_joint_angles[k]}")
        robot_model.rarm.angle_vector(r_joint_angles)
        ri.angle_vector(robot_model.angle_vector(), dt, controller_type="rarm_controller")
        robot_model.larm.angle_vector(l_joint_angles)
        ri.angle_vector(robot_model.angle_vector(), dt, controller_type="larm_controller")
        ri.wait_interpolation()

    motion_done_msg = Bool()
    motion_done_msg.data = True
    motion_done_pub.publish(motion_done_msg)
    time.sleep(3.0)
    ri.servo_off()
    
def start_grasp(arms):
    try:
        state = rospy.wait_for_message("/joint_states",JointState,timeout=20)
        position = list(state.position)
        joint_names = list(state.name)
        print(joint_names)
        for i in range(len(joint_names)):
            print(f"{i}:{joint_names[i]}")
        if arms == "rarm":
            rarm_joints = position[12:18] + [position[18]]
            print(f"radian to degree:{np.rad2deg(rarm_joints)}")
            robot_model.rarm.angle_vector(rarm_joints)
            robot_model.rarm_joint6.joint_angle(30.0)
            ri.angle_vector(robot_model.angle_vector(),0.5,controller_type="rarm_controller")
            ri.wait_interpolation()
            rospy.loginfo("done grasp with {arms}")
        elif arms == "larm":
            larm_joints = position[4:10] + [position[10]]
            print(f"radian to degree in GRASP:{np.rad2deg(larm_joints)}")
            robot_model.larm.angle_vector(larm_joints)
            robot_model.larm_joint6.joint_angle(-30.0)
            ri.angle_vector(robot_model.angle_vector(),0.5,controller_type="larm_controller")
            ri.wait_interpolation()
            rospy.loginfo("done grasp with {arms}")
    except Exception as e:
        rospy.loginfo(f"{e}")
        pass

def stop_grasp(arms):
    try:
        state = rospy.wait_for_message("/joint_states",JointState,timeout=20)
        position = list(state.position)
        joint_names = list(state.name)
        if arms == "rarm":
            rarm_joints = position[12:18] + [position[18]]
            print(f"radian to degree:{np.rad2deg(rarm_joints)}")
            robot_model.rarm.angle_vector(rarm_joints)
            robot_model.rarm_joint6.joint_angle(0.0)
            ri.angle_vector(robot_model.angle_vector(),0.5,controller_type="rarm_controller")
            ri.wait_interpolation()
            rospy.loginfo("done release with {arms}")
        elif arms == "larm":
            larm_joints = position[4:10] + [position[10]]
            print(f"radian to degree in RELEASE:{np.rad2deg(larm_joints)}")
            robot_model.larm.angle_vector(larm_joints)
            robot_model.larm_joint6.joint_angle(0.0)
            ri.angle_vector(robot_model.angle_vector(),0.5,controller_type="larm_controller")
            ri.wait_interpolation()
            rospy.loginfo("done release with {arms}")
    except Exception as e:
        rospy.loginfo(f"{e}")
        pass
    
def look_at_target(target):
    x = target.x
    y = target.y
    z = target.z
    robot_model.look_at(Coordinates(pos=(x, y, z)))
    # 変更をロボットに反映させる
    ri.angle_vector(robot_model.angle_vector(), 1.0, controller_type='head_controller')
    rospy.sleep(3.0)



def paint_motion(action):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    action_json_path = os.path.abspath(os.path.join(base_dir, "../../config/jedy_paint_action.json"))
    if action == "reach":
        act("reach",action_json_path)
        ri.servo_off()
        motion_done_pub.publish(Bool(data=True))
    elif action == "straight":
        act("straight",action_json_path)
        motion_done_pub.publish(Bool(data=True))


def paint_position(point):
    coordinates = Coordinates(pos=[point.x, point.y, point.z])
    solve_ik(coordinates)
    ri.wait_interpolation()
    motion_done_pub.publish(Bool(data=True))

def neck_motion_callback(msg):
    direction = msg.data
    # 方向ごとの注視点を定義（必要に応じてチューニング）
    direction_map = {
        "left_top": (-0.4,  -0.4, 0.3),
        "center_top": (0.0, -0.4, 0.3),
        "right_top": (0.4,  -0.4, 0.3),
        "left_center": (-0.4,  -0.4, 0.12),
        "center_center": (0.0,  -0.4, 0.12),
        "right_center": (0.4,  -0.4, 0.12),
        "left_bottom": (-0.4, -0.4, 0),
        "center_bottom": (0.0, -0.4, 0),
        "right_bottom": (0.4, -0.4, 0),
        "right_bottom2": (0.2, -0.3, -0.2),
    }

    pos = direction_map.get(direction, (0.0, 0.0, 1.2))  # fallbackは中央
    target = Coordinates(pos=pos)

    robot_model.look_at(target)
    ri.angle_vector(robot_model.angle_vector(), 1.0, controller_type='head_controller')
    rospy.sleep(2.0)


def search_idle_callback(msg):
    move_arm()
    basic_led()

def joint_angles_callback(msg):
    joint_angles = msg.data
    play_joint_frames_once(joint_angles)

def servo_on_off_callback(msg):
    # 追加: 空文字指定でALLトグル
    if msg.joint_names and msg.joint_names[0] == "":
        if msg.servo_on_states and msg.servo_on_states[0]:
            rospy.loginfo("[RobotBehavior] Turning ALL servos ON")
            ri.servo_on()  # 全サーボON
        else:
            rospy.loginfo("[RobotBehavior] Turning ALL servos OFF")
            ri.servo_off()
        return

    # 既存: 名前指定で個別ON
    rospy.loginfo(f"[RobotBehavior] Turning ON servos: {msg.joint_names}")
    ri.servo_on(joint_names=msg.joint_names)


def paint_motion_callback(msg):
    data = msg.data
    ri.servo_on()
    paint_motion(data)

def paint_position_callback(msg):
    ri.servo_on()
    paint_position(msg)
    
def grasp_callback(msg):
    print(msg.servo_on_states)
    if msg.servo_on_states[0]:
        if msg.joint_names[0] == "rarm":
            start_grasp("rarm")
        elif msg.joint_names[0] == "larm":
            start_grasp("larm")
    else:
        if msg.joint_names[0] == "rarm":
            stop_grasp("rarm")
        elif msg.joint_names[0] == "larm":
            stop_grasp("larm")



def main():
    #rospy.init_node('neck_motion_control', anonymous=True)
    
    # /neck_motionトピックを購読し、neck_motion_callback関数をコールバックに指定
    rospy.Subscriber('/detected_object_pose', String, neck_motion_callback)
    rospy.Subscriber('/search_idle', Bool, search_idle_callback)
    rospy.Subscriber('/joint_angles', Float64MultiArray, joint_angles_callback)
    rospy.Subscriber('/servo_on_off', ServoOnOff, servo_on_off_callback)
    rospy.Subscriber("/paint_motion",String,paint_motion_callback)
    rospy.Subscriber("/start_grasp",ServoOnOff,grasp_callback)
    #rospy.Subscriber("/paint_position",Point,paint_position_callback)

    # サーボをオンにして初期ポーズを設定
    ri.servo_on()
    #ri.angle_vector(robot_model.init_pose())

    # === ここを追加！ ===
    robot_model.reset_pose()  # モデル側の初期ポーズにセット
    ri.angle_vector(robot_model.angle_vector(), 2.0)  # 実ロボットに送信（2秒かけて動く）
    #rospy.sleep(2.0)  # 念のため待つ
        
    # ROSスピン: ノードが終了するまでコールバックを待ち続ける
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



