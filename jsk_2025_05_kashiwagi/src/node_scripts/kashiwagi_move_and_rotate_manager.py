#!/usr/bin/env python3
import rospy
import time
import random
from std_msgs.msg import Float32, String
from jsk_recognition_msgs.msg import ClassificationResult
from geometry_msgs.msg import Point
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from kashiwagi_move_and_rotate_utils import MoveAndRotate
import math  # ← 追加

class MoveAndRotateManager:
    def __init__(self):
        rospy.init_node("kashiwagi_move_and_rotate_manager")

        self.lost_person_counter = 0
        self.max_lost_person_counter = 4
        self.plus_or_minus = 1
        
        rospy.sleep(1)
        rospy.Subscriber('/gesture_recognition/result', ClassificationResult, self.hand_pose_update_callback, queue_size=1)
        rospy.Subscriber('/gesture_recognition/hand_position', Point, self.rotate_callback, queue_size=1)

        self.cur_state = "unknown"
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        self.latest_nearest_distance = float('nan')
        self.stop_distance = 0.150 #[m]
        self.under_stop_distance_counter = 0
        self.max_under_stop_distance_counter = 5
        rospy.Subscriber("/nearest_distance", Float32, self.nearest_distance_callback, queue_size=1)

        self.latest_hand_pose = "no_hand"
        self.move_and_rotate = MoveAndRotate()

        self.nice_position_counter = 0

        # --- 追加: 探索中の累積回転量 [rad] ---
        self.search_rotation_accum = 0.0
        self.full_turn_threshold = 2.0 * math.pi  # 1回転
        # -------------------------------------

        self.approach_time_limit = 40
        rospy.loginfo("Launching Move and Rotate node ....")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.cur_state:
            rospy.loginfo(f"State changed: {self.cur_state} → {new_state}")
            self.cur_state = new_state
            # 探索開始/終了で累積回転をリセット
            if new_state == "move:finding_person":
                self.search_rotation_accum = 0.0
                self.nice_position_counter = 0
                self.plus_or_minus = random.choice([-1, 1])
            else:
                # 探索以外に入ったら念のためリセット
                self.search_rotation_accum = 0.0

    def nearest_distance_callback(self, msg):
        self.latest_nearest_distance = msg.data
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$", self.under_stop_distance_counter)
        if self.cur_state == "move:approaching_person" and self.latest_nearest_distance < self.stop_distance:
            self.under_stop_distance_counter += 1
        else:
            self.under_stop_distance_counter = 0

    def hand_pose_update_callback(self, msg):
        if msg.label_names:
            self.latest_hand_pose = msg.label_names[0]
        else:
            self.latest_hand_pose = "no_hand"

    def _accumulate_rotation_and_check_lost(self, delta_rad, saw_person=False):
        """探索中の回転量を加算し、1回転超で getting_lost へ遷移。"""
        if self.cur_state != "move:finding_person":
            return
        # 人を見ていない時のみ「見失い」カウント（Paperを見ていたらリセットしても良い）
        if not saw_person:
            self.search_rotation_accum += abs(float(delta_rad))
            rospy.logdebug(f"accum_rot={self.search_rotation_accum:.2f} / {self.full_turn_threshold:.2f}")
            if self.search_rotation_accum >= self.full_turn_threshold:
                try:
                    req_state = "move:getting_lost"
                    resp = self.set_state_srv(req_state)
                    if resp.success:
                        rospy.logwarn("Did a full turn without finding a person → move:getting_lost")
                        self.cur_state = req_state
                        self.search_rotation_accum = 0.0
                    else:
                        rospy.logwarn(f"State change to getting_lost failed: {resp.message}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to call service: {e}")

    def rotate_callback(self, msg):
        print("###################", self.cur_state)
        if self.cur_state == "move:finding_person" or self.cur_state == "move:found_person":
            if self.latest_hand_pose == "Paper":
                print("11111")
                self.lost_person_counter = 0
                # 人を検出
                if self.cur_state == "move:finding_person":
                    print("22222")
                    req_state = "move:found_person"
                    try:
                        resp = self.set_state_srv(req_state)
                        if resp.success:
                            self.cur_state = req_state
                            rospy.loginfo(f"State reset successful: {resp.message}")
                            # 見つけたので探索累積はリセット
                            self.search_rotation_accum = 0.0
                        else:
                            rospy.logwarn(f"State reset failed: {resp.message}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Failed to call service: {e}")

                # 位置がちょうどよい
                if 300 <= msg.x <= 500:
                    print("OK")
                    self.nice_position_counter += 1
                    if self.nice_position_counter >= 5:
                        req_state = "move:approaching_person"
                        print("33333")
                        try:
                            resp = self.set_state_srv(req_state)
                            if resp.success:
                                rospy.loginfo(f"State reset successful: {resp.message}")
                                self.cur_state = req_state
                                self.move_forward()
                            else:
                                rospy.logwarn(f"State reset failed: {resp.message}")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Failed to call service: {e}")
                else:
                    # もう少し回転が必要
                    if msg.x < 300:
                        delta = -0.05
                        self.move_and_rotate.rotate_target_radian(delta)
                        print("AAAAAAAAAA")
                        self.nice_position_counter = 0
                        # found_person中はカウントしない。finding_person中でも「見えている」ので lost 判定に使わない
                        self._accumulate_rotation_and_check_lost(delta, saw_person=True)
                    elif msg.x > 500:
                        delta = 0.05
                        self.move_and_rotate.rotate_target_radian(delta)
                        print("BBBBBBBBBBBBBB")
                        self.nice_position_counter = 0
                        self._accumulate_rotation_and_check_lost(delta, saw_person=True)
            else:
                print(self.latest_hand_pose)
                delta = 0.1 * self.plus_or_minus
                self.move_and_rotate.rotate_target_radian(delta)
                self.nice_position_counter = 0
                print("CCCCCCCCCCCC")

                if self.cur_state == "move:found_person":
                    self.lost_person_counter += 1
                    rospy.loginfo(f"Lost person count: {self.lost_person_counter}")

                    if self.lost_person_counter >= self.max_lost_person_counter:
                        req_state = "move:finding_person"
                        try:
                            resp = self.set_state_srv(req_state)
                            if resp.success:
                                rospy.loginfo(f"Lost person 5 times → move:finding_person")
                                self.cur_state = req_state
                                self.lost_person_counter = 0
                            else:
                                rospy.logwarn(f"State reset failed: {resp.message}")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Failed to call service: {e}")
                else:
                    self.lost_person_counter = 0
                    
                self._accumulate_rotation_and_check_lost(delta, saw_person=False)

        else:
            pass

    def move_forward(self):
        print("under stop distance counter", self.under_stop_distance_counter)

        start_time = time.time() 
        print(self.cur_state, "%%%%%%%%%%%%&&&&&&&&&&&&")
            
        
        while self.cur_state == "move:approaching_person":

            # ★ 20秒経過したら停止して move:staying へ
            elapsed = time.time() - start_time
            if elapsed >= 20.0:
                rospy.loginfo(f"Approach time limit exceeded ({elapsed:.1f} sec) → move:staying")
                self.move_and_rotate.move_forward_target_velocity(0)
                req_state = "move:staying"
                break

            # ★ 近づきすぎ判定（元の動き）
            if self.under_stop_distance_counter < self.max_under_stop_distance_counter:
                self.move_and_rotate.move_forward_target_velocity(0.015)
            else:
                self.move_and_rotate.move_forward_target_velocity(0)
                self.under_stop_distance_counter = 0
                req_state = "daily:happy"
                break
        
        try:    
            # ★ while を抜けた後に状態を更新
            if req_state in ["daily:happy", "move:staying"]:
                try:
                    resp = self.set_state_srv(req_state)
                    if resp.success:
                        self.cur_state = req_state
                        rospy.loginfo(f"State reset successful: {resp.message}")
                    else:
                        rospy.logwarn(f"State reset failed: {resp.message}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to call service: {e}")
        except NameError:
            pass

if __name__ == "__main__":
    try:
        MoveAndRotateManager()
    except rospy.ROSInterruptException:
        pass
