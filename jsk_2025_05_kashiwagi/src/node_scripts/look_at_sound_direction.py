#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32, Float32
from collections import deque
import kashiwagi_utils

class VoiceTriggerWithMajority:
    def __init__(self):
        # パラメータ設定
        self.buffer_size = 60
        self.trigger_margin = 5  # 平均より+10でスパイクと判定
        self.vote_window = 10     # 直近何回で判定するか
        self.vote_threshold = 5   # 何回以上で確定とみなすか

        # 音量履歴（スパイク除去のため多めに保持）
        self.volume_history = deque(maxlen=200)
        self.spike_votes = deque(maxlen=self.vote_window)  # スパイクかどうかの履歴
        self.latest_direction = None  # 最新の音源方向

        self.audio_volume_sub = rospy.Subscriber("/audio_volume", Float32, self.volume_callback, queue_size=1)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32, self.direction_callback, queue_size=1)
        self.neck_pub = rospy.Publisher("/neck_yaw_angle", Float32, queue_size=10)
        self.rotate_rad = rospy.Publisher("/rotate_rad", Float32, queue_size=10)

        rospy.loginfo("VoiceTriggerWithMajority ノード起動")
        rospy.spin()

    def direction_callback(self, msg):
        self.latest_direction = msg.data

    def volume_callback(self, msg):
        vol = msg.data
        self.volume_history.append(vol)

        if len(self.volume_history) < self.buffer_size:
            rospy.loginfo(f"データ不足（{len(self.volume_history)}件） → スキップ")
            return

        # 直近60件から平均計算（スパイク除外あり）
        recent = list(self.volume_history)[-self.buffer_size:]
        rough_avg = sum(recent) / len(recent)
        valid = [v for v in recent if v <= rough_avg + self.trigger_margin]
        if len(valid) == 0:
            return
        avg = sum(valid) / len(valid)

        # 今回の音量がスパイクかどうかを判定
        is_spike = vol > avg + self.trigger_margin
        self.spike_votes.append(is_spike)

        # 過半数を超えたら確定判定
        spike_count = sum(self.spike_votes)
        if spike_count >= self.vote_threshold:
            self.spike_votes.clear()  # 判定後はリセット
            if self.latest_direction == None:
                rospy.loginfo(f"I heard voice but cannot detect its direction")
                
            else:
                rospy.loginfo(f"I heard voice. volume:({spike_count}/10)-> direction: {self.latest_direction}")

                ############ 首の角度の計算 #################
                cur_neck_yaw_angle = kashiwagi_utils.ri.angle_vector()[0]
                if self.latest_direction > 0:
                    new_neck_yaw_angle = (-1) * self.latest_direction / 180 + 1 + cur_neck_yaw_angle
                elif self.latest_direction <= 0:
                    new_neck_yaw_angle = (-1) * self.latest_direction / 180 - 1 + cur_neck_yaw_angle
                if new_neck_yaw_angle <= 0:
                    new_neck_yaw_angle = max(-0.5, new_neck_yaw_angle)
                else:
                    new_neck_yaw_angle = min(0.5, new_neck_yaw_angle)
                rospy.loginfo(f"new neck angle is {new_neck_yaw_angle}")
                pub_msg = Float32()
                pub_msg.data = new_neck_yaw_angle
                self.neck_pub.publish(pub_msg)
                rospy.loginfo(f"publishing at {new_neck_yaw_angle}")

                ############ 体の回転角度の計算 #############
                if self.latest_direction > 0:
                    direction_deg = self.latest_direction - 180
                elif self.latest_direction <= 0:
                    direction_deg = self.latest_direction + 180

                direction_rad = math.radians(direction_deg)
                print(direction_deg, "#############################################")
                
                
                # rospy.loginfo(
                #     f"I heard voice. volume:({spike_count}/{self.vote_threshold}) "
                #     f"-> direction_deg: {direction_deg}, direction_rad: {direction_rad}"
                # )

        else:
            rospy.logdebug(f"音量 {vol}（平均 {avg:.1f}） → スパイク: {is_spike}, スパイク履歴: {spike_count}/10")

if __name__ == "__main__":
    try:
        VoiceTriggerWithMajority()
    except rospy.ROSInterruptException:
        pass
