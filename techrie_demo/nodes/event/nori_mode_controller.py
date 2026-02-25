#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading, json, random
import rospy
from std_msgs.msg import String, UInt16, Bool
from techrie_demo.msg import InteractionEvent


class NoriModeController(object):
    """
    ノリノリモード制御ノード（/human/* を購読）
      - /human/dance         で開始（LED=3, 以降ループ）
      - /human/dance_stop    で停止（LED=1に戻す）
      - /human/dance_fast    でテンポ速く
      - /human/dance_slow    でテンポ遅く
      - /human/dance_mode_clap    で「手拍子ループ」モードに切替
      - /human/dance_mode_random で「ランダムダンス」モードに切替

    ループ中は /motion/play(String JSON) へ
      {"id":"pose:<name>", "dur":<秒>}
    を一定間隔で送ります。
    """

    def __init__(self):
        rospy.init_node("nori_mode_controller")

        # --- テンポ設定 ---
        self.duration = rospy.get_param("~duration_sec", 0.35)
        self.min_dur  = rospy.get_param("~min_duration_sec", 0.10)
        self.max_dur  = rospy.get_param("~max_duration_sec", 2.00)
        self.up_rate  = rospy.get_param("~speed_up_factor", 0.85)
        self.down_rate= rospy.get_param("~slow_down_factor", 1.15)

        # --- モード/ポーズ設定 ---
        # 初期モード：clap or random
        self.mode = rospy.get_param("~mode", "clap").strip().lower()
        # 手拍子ループに使うパターン
        self.clap_pattern = rospy.get_param("~clap_pattern", ["clap_1", "clap_2"])
        # ランダムで使う候補
        self.random_poses = rospy.get_param("~random_poses", ["clap_1", "clap_2"])
        # 直前と同じポーズを避けるか
        self.random_no_immediate_repeat = bool(rospy.get_param("~random_no_immediate_repeat", True))
        # 乱数seed（再現性が必要なときに設定）
        seed = rospy.get_param("~random_seed", None)
        if seed is not None:
            try:
                random.seed(int(seed))
            except Exception:
                pass

        # --- トピック名 ---
        self.topic_start = rospy.get_param("~topic_start", "/human/dance")
        self.topic_stop  = rospy.get_param("~topic_stop",  "/human/dance_stop")
        self.topic_fast  = rospy.get_param("~topic_fast",  "/human/dance_fast")
        self.topic_slow  = rospy.get_param("~topic_slow",  "/human/dance_slow")
        self.topic_mode_clap   = rospy.get_param("~topic_mode_clap",   "/human/dance_mode_clap")
        self.topic_mode_random = rospy.get_param("~topic_mode_random", "/human/dance_mode_random")

        # --- 出力 ---
        self.motion_pub = rospy.Publisher("/motion/play", String, queue_size=10)
        self.led_pub    = rospy.Publisher("/led_mode",   UInt16, queue_size=1, latch=True)
        self.ie_pub     = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=20)
        self.lock_pub   = rospy.Publisher("/ui/speak_lock", String, queue_size=1, latch=True)
        self.nori_pub   = rospy.Publisher("/ui/nori_mode_active", Bool, queue_size=1, latch=True)
        self.voice_pub  = rospy.Publisher("/jedy_voice", String, queue_size=10)

        # --- 入力 ---
        rospy.Subscriber(self.topic_start, String, lambda _msg: self.start())
        rospy.Subscriber(self.topic_stop,  String, lambda _msg: self.stop())
        rospy.Subscriber(self.topic_fast,  String, lambda _msg: self.speed_up())
        rospy.Subscriber(self.topic_slow,  String, lambda _msg: self.slow_down())
        rospy.Subscriber(self.topic_mode_clap,   String, lambda _msg: self.set_mode("clap"))
        rospy.Subscriber(self.topic_mode_random, String, lambda _msg: self.set_mode("random"))

        # --- 内部状態 ---
        self._running = False
        self._thr = None
        self._lock = threading.Lock()
        self._idx = 0            # clapパターン用のインデックス
        self._last_random = None # ランダム時の直前ポーズ

        # shutdown後片付け
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("NoriMode ready: mode=%s start=%s stop=%s fast=%s slow=%s",
                      self.mode, self.topic_start, self.topic_stop, self.topic_fast, self.topic_slow)

    # ===== Utility =====
    def _emit_ie(self, etype, meta=None, intensity=1.0):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = etype
        e.actor_id = "nori_mode_controller"
        e.target_id = ""
        e.intensity = float(intensity)
        e.meta_json = json.dumps(meta or {}, ensure_ascii=False)
        self.ie_pub.publish(e)

    def _on_shutdown(self):
        # 取り残し防止
        try:
            self.led_pub.publish(UInt16(1))
            self.lock_pub.publish(String(""))
            self.nori_pub.publish(Bool(data=False))
        except Exception:
            pass

    # ===== API =====
    def start(self):
        with self._lock:
            if self._running:
                rospy.loginfo("NoriMode already running")
                return
            self._running = True
            self._idx = 0
            self._last_random = None
        # 先にロック/フラグを立てる → その後演出
        self.lock_pub.publish(String("nori_mode"))
        self.nori_pub.publish(Bool(data=True))
        self.led_pub.publish(UInt16(3))
        self.voice_pub.publish(String("♪"))
        self._emit_ie("ROBOT_DANCE_START", {"mode": self.mode, "dur": self.duration})
        self._thr = threading.Thread(target=self._loop, name="nori_loop")
        self._thr.daemon = True
        self._thr.start()
        rospy.loginfo("NoriMode START (mode=%s, dur=%.3f)", self.mode, self.duration)

    def stop(self):
        with self._lock:
            if not self._running:
                rospy.loginfo("NoriMode not running")
            self._running = False
        self.led_pub.publish(UInt16(1))
        self.voice_pub.publish(String(""))
        self._emit_ie("ROBOT_DANCE_STOP", {"mode": self.mode})
        self.lock_pub.publish(String(""))
        self.nori_pub.publish(Bool(data=False))
        # リセット姿勢（任意）
        payload = json.dumps({"id": "pose:reset", "dur": self.duration})
        self.motion_pub.publish(String(payload))
        rospy.loginfo("NoriMode STOP")

    def speed_up(self):
        self.duration = max(self.min_dur, self.duration * self.up_rate)
        self._emit_ie("ROBOT_DANCE_TEMPO", {"mode": self.mode, "dur": self.duration})
        rospy.loginfo("NoriMode speed UP: dur=%.3f", self.duration)

    def slow_down(self):
        self.duration = min(self.max_dur, self.duration * self.down_rate)
        self._emit_ie("ROBOT_DANCE_TEMPO", {"mode": self.mode, "dur": self.duration})
        rospy.loginfo("NoriMode slow DOWN: dur=%.3f", self.duration)

    def set_mode(self, mode_name: str):
        mode_name = (mode_name or "").strip().lower()
        if mode_name not in ("clap", "random"):
            rospy.logwarn("NoriMode: unknown mode '%s'", mode_name)
            return
        with self._lock:
            if self.mode == mode_name:
                return
            self.mode = mode_name
            self._idx = 0
            self._last_random = None
        self._emit_ie("ROBOT_DANCE_MODE", {"mode": self.mode})
        rospy.loginfo("NoriMode: mode -> %s", self.mode)

    # ===== メインループ =====
    def _next_pose(self):
        """現在モードに基づいて、次に出すポーズ名を返す"""
        if self.mode == "clap":
            if not self.clap_pattern:
                return "clap_1"
            pose = self.clap_pattern[self._idx % len(self.clap_pattern)]
            self._idx = (self._idx + 1) % len(self.clap_pattern)
            return pose
        else:  # random
            if not self.random_poses:
                return "clap_1"
            candidates = list(self.random_poses)
            if self.random_no_immediate_repeat and self._last_random in candidates and len(candidates) >= 2:
                candidates.remove(self._last_random)
            pose = random.choice(candidates)
            self._last_random = pose
            return pose

    def _loop(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            with self._lock:
                if not self._running:
                    break
                pose = self._next_pose()
                dur  = float(self.duration)

            payload = json.dumps({"id": f"pose:{pose}", "dur": dur})
            self.motion_pub.publish(String(payload))

            t0 = rospy.Time.now()
            while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
                with self._lock:
                    if not self._running:
                        break
                rate.sleep()

        # 終了後の復帰（保険）
        self.led_pub.publish(UInt16(1))


if __name__ == "__main__":
    NoriModeController()
    rospy.spin()
