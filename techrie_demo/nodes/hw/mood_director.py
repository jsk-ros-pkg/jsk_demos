#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mood_director.py
- たまに入る2モードを演出:
  1) ROBOT_LISTEN : ぼんやりピンクLED（idleは止めない）
  2) ROBOT_PRESENT: オレンジ点滅 + 目線 + 左腕 + グリッパ開→閉
- /interaction_events へログ（diary_loggerで写真+JSON）
- ★ self-paint連携: PAINT_BEGIN 中は開始禁止。終了後は PAINT_COOLDOWN_SEC だけ開始禁止
- ★ speak_lock連携: 他ノードのロックだけを開始ブロック対象にする
  （自分がセットした "mood_director:*" ロックは無視）
"""

import json
import random
import threading
import time

import rospy
from std_msgs.msg import ColorRGBA, String, Bool
from kxr_controller.msg import ServoOnOff
from techrie_demo.msg import InteractionEvent

# ===== パラメータ =====
LISTEN_MIN_INTERVAL = 40.0
LISTEN_PROB         = 0.20
LISTEN_DURATION     = 12.0

PRESENT_MIN_INTERVAL = 60.0
PRESENT_PROB         = 0.15
PRESENT_DURATION     = 10.0

PAINT_COOLDOWN_SEC   = 8.0   # ペイント終了後の開始禁止時間

BLINK_PERIOD   = 0.3
ORANGE         = ColorRGBA(1.0, 0.5, 0.0, 0.6)
PINK_DIM       = ColorRGBA(1.0, 0.3, 0.6, 0.35)
LED_OFF        = ColorRGBA(0.0, 0.0, 0.0, 0.0)

POSE_TOPIC     = "/detected_object_pose"  # "left_top"/"center_top"/"right_top"
MOTION_TOPIC   = "/motion/play"           # "pose:point_hand"
GRASP_TOPIC    = "/start_grasp"           # kxr_controller/ServoOnOff

SELF_LOCK_PREFIX = "mood_director:"       # 自分が立てるロックのプレフィックス
PRESENT_RESET_DELAY = 5.0   # グリッパ閉じてから reset までの待ち秒数

class MoodDirector(object):
    def __init__(self):
        rospy.init_node("mood_director")

        # ---- Publishers
        self.pub_led    = rospy.Publisher("/status_led", ColorRGBA, queue_size=10)
        self.pub_pose   = rospy.Publisher(POSE_TOPIC, String, queue_size=10)
        self.pub_motion = rospy.Publisher(MOTION_TOPIC, String, queue_size=10)
        self.pub_grasp  = rospy.Publisher(GRASP_TOPIC, ServoOnOff, queue_size=10)
        self.pub_idle   = rospy.Publisher("/idle/enabled", Bool, queue_size=3, latch=True)
        self.pub_evt    = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
        self.pub_speak_lock = rospy.Publisher("/ui/speak_lock", String, queue_size=3, latch=True)
        self.pub_text   = rospy.Publisher("/robot_text", String, queue_size=10)
        self.pub_emote  = rospy.Publisher("/emotion/set", String, queue_size=10)

        # ---- State
        self._last_listen_ts  = 0.0
        self._last_present_ts = 0.0
        self._lock = threading.Lock()

        self._painting_active = False
        self._paint_guard_until = 0.0
        self._speak_lock_state = ""      # 現在のロック文字列（外部・内部とも入る）
        self._active_mode = ""           # "", "listen", "present"
        self._nori_active = False        # ← ノリノリ中フラグ（外部から制御）

        # ---- Subscribers
        rospy.Subscriber("/interaction_events", InteractionEvent, self._cb_event_guard, queue_size=50)
        rospy.Subscriber("/ui/speak_lock", String, self._cb_speak_lock, queue_size=10)
        rospy.Subscriber("/ui/nori_mode_active", Bool, self._cb_nori_flag, queue_size=10)

        # diary meta
        self.event_meta = {
            "event_name":   rospy.get_param("~event_name",   "Ambient Modes"),
            "event_phase":  rospy.get_param("~event_phase",  "live"),
            "event_day":    rospy.get_param("~event_day",    "0"),
            "event_cohort": rospy.get_param("~event_cohort", "default"),
            "event_goal":   rospy.get_param("~event_goal",   "be lively"),
        }

        rate_hz = float(rospy.get_param("~check_rate_hz", 0.5))
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("mood_director: ready (rate=%.2f Hz)", rate_hz)

    # ====== guards ======
    def _cb_event_guard(self, e: InteractionEvent):
        et = (e.event_type or "").strip().upper()
        if et == "PAINT_BEGIN":
            self._painting_active = True
        elif et in ("PAINT_DONE", "PAINT_TIMEOUT"):
            self._painting_active = False
            self._paint_guard_until = time.time() + PAINT_COOLDOWN_SEC

    def _cb_speak_lock(self, s: String):
        self._speak_lock_state = (s.data or "")

    def _external_speak_lock_active(self) -> bool:
        """
        外部のロックがかかっているか？
        自分がセットした "mood_director:*" は無視して判定する。
        """
        lock = (self._speak_lock_state or "").strip()
        if not lock:
            return False
        return not lock.startswith(SELF_LOCK_PREFIX)
    
    def _cb_nori_flag(self, b: Bool):
        self._nori_active = bool(getattr(b, "data", False))

    def _can_start_mode(self) -> bool:
        """listen/present を開始してよいか（開始前の一度だけ利用）"""
        if self._painting_active:
            return False
        if time.time() < self._paint_guard_until:
            return False
        if self._external_speak_lock_active():
            return False
        if self._nori_active:            # ← ノリノリ中は開始しない
            return False
        if self._active_mode:           # すでにどちらか実行中なら二重起動しない
            return False
        return True

    # ===== helpers =====
    def _say(self, text, color="white"):
        self.pub_text.publish(String(text))
        self.pub_emote.publish(String(color))

    def _log_evt(self, etype, meta=None):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = etype
        e.actor_id = "mood_director"
        e.target_id = "scene"
        payload = dict(self.event_meta)
        if meta:
            payload.update(meta)
        e.meta_json = json.dumps(payload, ensure_ascii=False)
        self.pub_evt.publish(e)

    # ===== LISTEN =====
    def _run_listen(self):
        # 開始前チェック（ここだけでOK）
        if not self._can_start_mode():
            return
        with self._lock:
            self._last_listen_ts = time.time()
            self._active_mode = "listen"

        # 自分のロックをセット（外部ロック判定の対象外）
        self.pub_speak_lock.publish(String(SELF_LOCK_PREFIX + "listen"))

        self._log_evt("ROBOT_LISTEN", {"emotion": "calm"})
        self._say("うんうん", "interest")

        t_end = time.time() + LISTEN_DURATION
        try:
            while not rospy.is_shutdown() and time.time() < t_end:
                # 実行中は“ハードなプリエンプト”のみで中断（= self-paint 開始）
                if (self._painting_active or self._nori_active):
                    break
                self.pub_led.publish(PINK_DIM)
                rospy.sleep(0.5)
        finally:
            self.pub_led.publish(LED_OFF)
            self.pub_speak_lock.publish(String(""))  # ロック解除
            self._active_mode = ""

    # ===== PRESENT =====
    def _run_present(self):
        # 開始前チェック
        if not self._can_start_mode():
            return
        with self._lock:
            self._last_present_ts = time.time()
            self._active_mode = "present"

        # idle 停止＆強ロック
        self.pub_idle.publish(Bool(data=False))
        rospy.sleep(0.1)
        self.pub_speak_lock.publish(String(SELF_LOCK_PREFIX + "present"))

        self._log_evt("ROBOT_PRESENT", {"emotion": "show"})

        # LED: オレンジ点滅（別スレッド）
        stop_flag = {"stop": False}
        def _blink():
            on = True
            while not rospy.is_shutdown() and not stop_flag["stop"]:
                self.pub_led.publish(ORANGE if on else LED_OFF)
                on = not on
                rospy.sleep(BLINK_PERIOD)
        th_blink = threading.Thread(target=_blink, daemon=True)
        th_blink.start()

        try:
            # 実行中は“ハードなプリエンプト”のみで中断
            # 目線：左上→上→右上
            if not (self._painting_active or self._nori_active):
                self.pub_pose.publish(String(data="center_top"))
                rospy.sleep(random.uniform(1.0, 2.0))

            # 左腕を上げる
            if not (self._painting_active or self._nori_active):
                self.pub_motion.publish(String(data="pose:point_hand"))
                rospy.sleep(0.5)
                self._say("はい！", "joy")

            # グリッパ：開→閉
            if not (self._painting_active or self._nori_active):
                open_msg = ServoOnOff(); open_msg.joint_names = ["larm"]; open_msg.servo_on_states = [False]
                self.pub_grasp.publish(open_msg)
                rospy.sleep(1.0)
            if not (self._painting_active or self._nori_active):
                close_msg = ServoOnOff(); close_msg.joint_names = ["larm"]; close_msg.servo_on_states = [True]
                self.pub_grasp.publish(close_msg)
                rospy.sleep(0.5)

            # ★ ここで少し“見せ”てから、姿勢をリセット
            if not (self._painting_active or self._nori_active):
                rospy.sleep(PRESENT_RESET_DELAY)              # ← 好みで調整
                self.pub_motion.publish(String(data="pose:reset"))

        finally:
            stop_flag["stop"] = True
            th_blink.join(timeout=0.1)
            self.pub_led.publish(LED_OFF)
            self.pub_idle.publish(Bool(data=True))
            self.pub_speak_lock.publish(String(""))  # ロック解除
            self._active_mode = ""


    # ===== メインループ =====
    def spin(self):
        while not rospy.is_shutdown():
            now = time.time()

            # LISTEN
            if (now - self._last_listen_ts) > LISTEN_MIN_INTERVAL and self._can_start_mode():
                if random.random() < LISTEN_PROB:
                    threading.Thread(target=self._run_listen, daemon=True).start()

            # PRESENT
            if (now - self._last_present_ts) > PRESENT_MIN_INTERVAL and self._can_start_mode():
                if random.random() < PRESENT_PROB:
                    threading.Thread(target=self._run_present, daemon=True).start()

            self.rate.sleep()

def main():
    node = MoodDirector()
    node.spin()

if __name__ == "__main__":
    main()
