#!/usr/bin/env python3

import rospy
from skrobot.viewers import PyrenderViewer
from kxr_controller.kxr_interface import KXRROSRobotInterface
from ros_speak import speak_jp # 音声合成用のROSパッケージ
import numpy as np
import random
import time
from skrobot.model import RobotModel
from jsk_recognition_msgs.msg import ClassificationResult
from skrobot.utils.urdf import no_mesh_load_mode

sound_volume = 0.001


# --- ROSトピックから新しいラベルを待ち受ける関数 ---
def get_new_label_with_rate(topic_name, timeout=5.0, rate=10):
    """
    指定されたROSトピックから ClassificationResult メッセージを購読し、
    最初に見つかったラベル名を返します。タイムアウトを設定できます。
    """
    start_time = rospy.Time.now()
    result_container = []

    def callback(msg):
        # ラベル名があり、まだ結果が格納されていなければ、結果を格納
        if not result_container and msg.label_names and not rospy.is_shutdown():
            result_container.append(msg.label_names[0].lower())

    rospy.loginfo(f"'{topic_name}'トピックで新しいメッセージを最大{timeout}秒間待ちます...")
    sub = rospy.Subscriber(topic_name, ClassificationResult, callback)
    
    r = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        if result_container:
            rospy.loginfo(f"ラベルを受信しました: '{result_container[0]}'")
            break
            
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn(f"タイムアウト: {timeout}秒以内に'{topic_name}'から新しいメッセージを受信できませんでした。")
            break
        
        r.sleep()

    sub.unregister() # Subscriberを解除
    return result_container[0] if result_container else None

# --- ロボットの初期化 ---
rospy.init_node("kxr_interface", anonymous=True)
namespace = ''

robot_description = namespace + "/robot_description"
robot_model = RobotModel()
with no_mesh_load_mode():
    robot_model.load_urdf_from_robot_description(robot_description)
robot_model.left_finger_joint1.min_angle = -1.5

ri = KXRROSRobotInterface(
    robot_model, namespace=namespace, controller_timeout=60.0
)

viewer = PyrenderViewer(resolution=(640, 480))
viewer.add(robot_model)
viewer.show()

# --- ロボットのポーズ定義 ---
# ロボットモデルに合わせてこれらの関節角度を調整してください。
# これらはあくまで例であり、安全な値であることを確認してください。

# じゃんけんのポーズ
rock_pose = np.array([-2.0027479e-02,  1.3277159e+00,  5.3016134e-03, -6.7151368e-02,
        1.3548294e-02,  8.2468567e-03,  1.7555017e-07,  1.7555017e-07,
        2.3561770e-02, -1.9438431e-02, -1.7565429e+00, -1.5000000e+00,
       -1.9438431e-02,  7.7754594e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  8.2468567e-03], dtype=np.float32)
scissors_pose = np.array([-2.00274792e-02,  1.36188066e+00,  5.30161336e-03, -6.71513677e-02,
        1.88497324e-02,  8.24685674e-03,  1.75550170e-07,  1.75550170e-07,
        1.17807975e-02, -6.02596581e-01, -1.75654292e+00, -1.50000000e+00,
        1.05204105e+00,  7.77545944e-02, -8.86518359e-01, -8.71202767e-01,
        2.65071869e+00,  8.24685674e-03], dtype=np.float32)
paper_pose = np.array([-1.4136992e-02,  1.4302102e+00,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  2.3563702e-03,  1.7555017e-07,  1.7555017e-07,
        7.0685662e-02, -1.6257726e-01, -1.7618443e+00, -1.5963201e-01,
        1.3931003e+00,  7.1864113e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03], dtype=np.float32)
normal_pose = np.array([-2.0027479e-02, -2.4798931e-01,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  2.3563702e-03,  1.7555017e-07,  1.7555017e-07,
        3.5342742e-02,  2.1205769e-01, -1.7618443e+00, -4.3000001e-01,
        1.5700001e+00,  7.7754594e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03],  dtype=np.float32)

# あっちむいてホイ用の顔の向きポーズ (首を振る側)
migimuki_pose = np.array([-2.3560191e-03,  3.4164999e-02, -1.6375550e+00,  8.2468567e-03,
        7.6578078e-03,  8.2468567e-03, -5.3012623e-03, -5.3012623e-03,
       -2.8274510e-02,  8.2468567e-03,  8.2468567e-03, -4.3000001e-01,
        3.6403224e-01,  7.1864113e-02, -8.9240885e-01, -8.7120277e-01,
        2.6507187e+00, -2.3560191e-03], dtype=np.float32)
hidarimuki_pose = np.array([-2.3560191e-03,  3.4164999e-02,  1.5662805e+00,  8.2468567e-03,
        7.6578078e-03,  2.3563702e-03, -5.3012623e-03, -5.3012623e-03,
       -2.8274510e-02,  8.2468567e-03,  1.3548294e-02, -4.3000001e-01,
        3.6403224e-01,  7.1864113e-02, -8.9240885e-01, -8.7120277e-01,
        2.6507187e+00, -2.3560191e-03], dtype=np.float32)
uemuki_pose = np.array([-2.3560191e-03,  3.4164999e-02, -1.7082235e-02,  8.2468567e-03,
        7.6578078e-03,  2.3563702e-03, -5.3012623e-03, -5.3012623e-03,
       -2.8274510e-02,  8.2468567e-03,  1.3548294e-02, -4.3000001e-01,
        3.6403224e-01,  7.1864113e-02, -8.9240885e-01, -8.7120277e-01,
        2.6507187e+00,  1.0979868e+00], dtype=np.float32)
shitamuki_pose = np.array([-2.3560191e-03,  3.4164999e-02, -1.1191749e-02,  8.2468567e-03,
        7.6578078e-03,  2.3563702e-03, -5.3012623e-03, -5.3012623e-03,
       -2.8274510e-02,  8.2468567e-03,  1.3548294e-02, -4.3000001e-01,
        3.6403224e-01,  7.1864113e-02, -8.9240885e-01, -8.7120277e-01,
        2.6507187e+00, -1.2046043e+00], dtype=np.float32)

# あっちむいてホイ用の指差しポーズ (指す側)
migisashi_pose = np.array([-2.0027479e-02,  2.0616721e-01,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  8.2468567e-03, -5.3012623e-03, -5.3012623e-03,
        3.5342742e-02,  1.4690875e+00, -1.5556773e+00, -1.5374152e-01,
       -1.4136992e-02,  7.7754594e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03], dtype=np.float32)
hidarisashi_pose = np.array([-2.0027479e-02, -2.2972722e-02,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  8.2468567e-03,  1.7555017e-07,  1.7555017e-07,
       -8.0110788e-02, -9.2245001e-01, -1.5556773e+00, -1.5374152e-01,
       -1.9438431e-02,  7.7754594e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03], dtype=np.float32)
uesashi_pose = np.array([-1.4136992e-02,  1.4472927e+00,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  2.3563702e-03,  1.7555017e-07,  1.7555017e-07,
        7.0685662e-02,  6.4795524e-02, -1.5892531e+00, -1.5963201e-01,
       -1.4136992e-02,  7.7754594e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03], dtype=np.float32)
shitasahi_pose = np.array([-2.0027479e-02,  4.2352614e-01,  1.7555017e-07, -6.7151368e-02,
        1.3548294e-02,  8.2468567e-03,  1.7555017e-07,  1.7555017e-07,
       -5.7137895e-02,  1.7259143e-01, -1.3017957e-01, -1.5963201e-01,
       -1.4136992e-02,  8.3056033e-02, -8.8651836e-01, -8.7120277e-01,
        2.6507187e+00,  2.3563702e-03], dtype=np.float32)


# じゃんけんの手の種類と対応するポーズ
janken_hand_map = {
    "rock": rock_pose,
    "scissors": scissors_pose,
    "paper": paper_pose
}
janken_choices = list(janken_hand_map.keys())

# あっちむいてホイの方向と対応するポーズ
atchimuki_robot_pointing_map = { # ロボットが指す側のポーズ
    "up": uesashi_pose,
    "down": shitasahi_pose,
    "left": hidarisashi_pose,
    "right": migisashi_pose
}
atchimuki_robot_head_turn_map = { # ロボットが首を振る側のポーズ
    "up": uemuki_pose,
    "down": shitamuki_pose,
    "left": hidarimuki_pose,
    "right": migimuki_pose
}
atchimuki_directions = ["up", "down", "left", "right"] # あっちむいてホイで使う方向

# --- じゃんけんの勝敗判定関数 ---
def judge_janken(robot_hand, human_hand):
    """じゃんけんの勝敗を判定します。"""
    if robot_hand == human_hand:
        return "draw"
    elif (robot_hand == "rock" and human_hand == "scissors") or \
         (robot_hand == "scissors" and human_hand == "paper") or \
         (robot_hand == "paper" and human_hand == "rock"):
        return "robot_win"
    else:
        return "human_win"

# --- ロボットの動作と発話ヘルパー関数 ---
def move_robot(pose, duration=1.0, no_wait=False):
    """ロボットを目標ポーズに動かします。"""
    robot_model.angle_vector(pose)
    robot_model.rarm_module2_joint1.joint_angle(1.57)
    ri.angle_vector(robot_model.angle_vector(), duration)
    if no_wait is False:
        ri.wait_interpolation()

def reset_robot_pose(duration=2.0):
    """ロボットをノーマルポーズに戻します。"""
    robot_model.angle_vector(normal_pose)
    robot_model.rarm_module2_joint1.joint_angle(1.57)
    robot_model.larm_module2_joint1.joint_angle(-1.57)
    move_robot(robot_model.angle_vector(), duration)

# --- メインゲームループ ---
if __name__ == '__main__':
    while not rospy.is_shutdown():
        ri.servo_off()
        speak_jp("いっしょに遊ぶ？遊ぶ場合は僕の関節を動かしてね", volume=sound_volume ,wait=True) # ゲーム開始の挨拶
        previous_angle = ri.angle_vector()
        while not rospy.is_shutdown():
            current_angle = ri.angle_vector()
            if np.any(np.abs(current_angle - previous_angle) > np.deg2rad(10)):
                break

        try:
            ri.servo_on()
            speak_jp("いいよー", volume=sound_volume ,wait=True) # ゲーム開始の挨拶

            while not rospy.is_shutdown(): # ROSがシャットダウンされていない間、ゲームを継続
                janken_round_result = "draw"

                # --- じゃんけんフェーズ ---
                while janken_round_result == "draw" and not rospy.is_shutdown():
                    reset_robot_pose()
                    time.sleep(1)

                    speak_jp("最初はグー。", volume=sound_volume, wait=False)
                    move_robot(rock_pose, 1.0)
                    reset_robot_pose(duration=1.0) # 手を引っ込める

                    speak_jp("じゃんけん。", volume=sound_volume, wait=True)

                    robot_hand_name = random.choice(janken_choices) # ロボットの手をランダムに選択
                    robot_hand_pose = janken_hand_map[robot_hand_name] # 対応するポーズを取得

                    move_robot(robot_hand_pose, 1.0, no_wait=True) # ロボットが手を出します
                    speak_jp("ポン。", volume=sound_volume, wait=False) # 明瞭に「ポン」
                    ri.wait_interpolation()

                    speak_jp("あなたの手を認識しています", volume=sound_volume, wait=True)
                    human_hand_name = get_new_label_with_rate(topic_name='/gesture_recognition/result', timeout=5.0)

                    if human_hand_name not in janken_choices: # 認識失敗や無効な手の場合
                        speak_jp("うまく認識できませんでした。もう一度じゃんけんをしましょう。", volume=sound_volume, wait=True) # 丁寧な謝罪
                        rospy.logwarn(f"人間の手の認識に失敗または無効な手: {human_hand_name}。じゃんけんをやり直します。")
                        continue # じゃんけんを最初からやり直し

                    janken_round_result = judge_janken(robot_hand_name, human_hand_name)
                    rospy.loginfo(f"ジャンケン結果: ロボット({robot_hand_name}) vs 人間({human_hand_name}) -> {janken_round_result}")

                    if janken_round_result == "draw":
                        speak_jp("あいこだね", volume=sound_volume, wait=True)
                        reset_robot_pose(duration=1.0)
                        speak_jp("もう一度", volume=sound_volume, wait=True)
                    else:
                        break # じゃんけんが決着したら、あいこループを抜ける

                # --- じゃんけん勝敗の通知 ---
                if janken_round_result == "robot_win":
                    speak_jp("ぼくのかち！", volume=sound_volume,wait=True) # ロボットが勝った場合
                elif janken_round_result == "human_win":
                    speak_jp("ぼくのまけ", volume=sound_volume, wait=True) # ロボットが負けた場合 (少し残念そうに)
                time.sleep(1)

                # --- あっちむいてホイ フェーズ ---
                speak_jp("あっちむいて", volume=sound_volume,wait=True)
                speak_jp("ホイ。", volume=sound_volume) # 明瞭に「ホイ」

                if janken_round_result == "robot_win":
                    # ロボットがじゃんけんに勝ったので、ロボットが指す側を演じる
                    robot_pointing_direction = random.choice(atchimuki_directions)
                    robot_pointing_pose = atchimuki_robot_pointing_map[robot_pointing_direction]
                    robot_model.angle_vector(robot_pointing_pose)
                    robot_model.rarm_module2_joint1.joint_angle(1.57)
                    move_robot(robot_pointing_pose, 1.0) # ロボットが指を差す

                elif janken_round_result == "human_win":
                    # 人間がじゃんけんに勝ったので、ロボットが顔を振る側を演じる
                    robot_head_turn_direction = random.choice(atchimuki_directions)
                    robot_head_turn_pose = atchimuki_robot_head_turn_map[robot_head_turn_direction]
                    robot_model.angle_vector(robot_head_turn_pose)
                    robot_model.larm_joint0.joint_angle(0.3)
                    robot_model.larm_module2_joint1.joint_angle(-1.5)
                    move_robot(robot_model.angle_vector(), 1.0) # ロボットが首を振る

                # あっちむいてホイのアクションが終わったら、ゲームを終了する
                reset_robot_pose() # 通常姿勢に戻す
                speak_jp("あそんでくれて、ありがとう", volume=sound_volume, wait=True) # 終わりの挨拶
                rospy.loginfo("じゃんけん＆あっちむいてホイゲーム終了。")
                break # メインループを抜けてゲームを終了

        except rospy.ROSInterruptException:
            rospy.loginfo("ROSノードがシャットダウンされました。ゲームを終了します。")
        except Exception as e:
            rospy.logerr(f"予期せぬエラーが発生しました: {e}")
        finally:
            rospy.loginfo("ゲームプログラムを終了します。")
