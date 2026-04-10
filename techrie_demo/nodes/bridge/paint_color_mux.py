#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time
from std_msgs.msg import String

class PaintColorMux:
    def __init__(self):
        rospy.init_node("paint_color_mux")

        # --- Params ---
        self.ttl_sec     = float(rospy.get_param("~human_ttl_sec", 20.0))  # 人の選択を優先する秒数
        self.cooldown    = float(rospy.get_param("~cooldown_sec", 0.5))    # 同一色の過剰発行防止
        self.default_col = rospy.get_param("~default_color", "orange")

        # 入力
        self.sub_h = rospy.Subscriber("/paint_color_human",  String, self._on_human,  queue_size=10)
        self.sub_p = rospy.Subscriber("/paint_color_policy", String, self._on_policy, queue_size=10)

        # 出力（ラッチ）
        self.pub    = rospy.Publisher("/paint_color", String, queue_size=1, latch=True)

        # 状態
        self.last_out = ""       # 直近に出した色
        self.last_out_t = 0.0    # 最終 publish 時刻
        self.human_until = 0.0   # 人の優先期限（時刻）
        self.human_color = ""    # 直近の人色
        self.policy_color = ""   # 直近の提案色

        # 初期色を出して安定化
        self._publish_once(self.default_col)
        rospy.loginfo("paint_color_mux: ready (ttl=%.1fs, cooldown=%.1fs)", self.ttl_sec, self.cooldown)

    # ---------- inputs ----------
    def _on_human(self, s: String):
        col = (s.data or "").strip().lower()
        if not col: return
        self.human_color  = col
        self.human_until  = time.time() + self.ttl_sec
        self._maybe_publish(col, reason="human")

    def _on_policy(self, s: String):
        col = (s.data or "").strip().lower()
        if not col: return
        self.policy_color = col
        # 人の優先時間が切れている場合のみ反映
        if time.time() >= self.human_until:
            self._maybe_publish(col, reason="policy")

    # ---------- core ----------
    def _maybe_publish(self, col: str, reason: str):
        now = time.time()
        if col == self.last_out and (now - self.last_out_t) < self.cooldown:
            return
        self._publish_once(col)
        rospy.loginfo("paint_color_mux: -> %s (by %s)", col, reason)

    def _publish_once(self, col: str):
        self.last_out = col
        self.last_out_t = time.time()
        self.pub.publish(String(col))

def main():
    PaintColorMux()
    rospy.spin()

if __name__ == "__main__":
    main()
