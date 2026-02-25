#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wake_word_blinker.py (modified)
- /speech_to_text (speech_recognition_msgs/SpeechRecognitionCandidates) を購読
- 正規表現でウェイクワードを検出すると LED を点滅
- 併せて /interaction_events に InteractionEvent を publish（event_type="HUMAN_WAKE"）
  → e.stamp / e.event_type / e.actor_id / e.target_id / e.meta_json を設定
"""

import json
import re
import threading
import time

import rospy
from std_msgs.msg import ColorRGBA, String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from techrie_demo.msg import InteractionEvent  # 環境に合わせてパッケージを調整してください


class WakeWordBlinker:
    def __init__(self):
        # ===== Params =====
        # 反応キーワード（正規表現）。既定は Jedy が誤認されやすい形を網羅
        default_patterns = [
            r"(ジェ)(リ|ディ)[ー〜]?",     # ジェリ-/ジェディ-
            r"ジェリー",                   # ジェリー
            r"チェリー",                   # チェリー（誤認カバー）
            r"じぇ(り|でぃ)?[ー〜]?",      # ひらがな
            r"\bjedy\b",                   # ローマ字
        ]
        patt_list = rospy.get_param("~patterns", default_patterns)
        self.patterns = [re.compile(p, re.IGNORECASE) for p in patt_list]

        self.min_conf = float(rospy.get_param("~min_confidence", 0.0))
        self.debounce_sec = float(rospy.get_param("~debounce_sec", 2.0))
        self.blink_times = int(rospy.get_param("~blink_times", 2))
        self.blink_period = float(rospy.get_param("~blink_period", 0.2))
        self.respect_lock = bool(rospy.get_param("~respect_speak_lock", True))

        # 色（オレンジ）
        self.color_r = float(rospy.get_param("~color_r", 1.0))
        self.color_g = float(rospy.get_param("~color_g", 0.5))
        self.color_b = float(rospy.get_param("~color_b", 0.0))
        self.color_a = float(rospy.get_param("~color_a", 0.6))
        self.col_on  = ColorRGBA(self.color_r, self.color_g, self.color_b, self.color_a)
        self.col_off = ColorRGBA(0.0, 0.0, 0.0, 0.0)

        # /interaction_events に付与したい文脈（必要なら launch から渡す）
        self.diary_meta = {
            "event_name":   rospy.get_param("~event_name",   "Wakeword"),
            "event_phase":  rospy.get_param("~event_phase",  "live"),
            "event_day":    rospy.get_param("~event_day",    "0"),
            "event_cohort": rospy.get_param("~event_cohort", "default"),
            "event_goal":   rospy.get_param("~event_goal",   "wake Jedy"),
        }

        # ===== IO =====
        self.pub_led = rospy.Publisher("/status_led", ColorRGBA, queue_size=1)
        self.pub_evt = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self._cb_stt, queue_size=10)

        # 発話ロック（他の演出と衝突させたくない場合に使用）
        self.locked = False
        if self.respect_lock:
            rospy.Subscriber("/ui/speak_lock", String, self._cb_lock, queue_size=1)

        # 内部状態
        self._last_fire = 0.0
        self._blink_thread = None

        rospy.loginfo(
            "wake_word_blinker: ready (debounce=%.1fs, blink=%dx %.1fs)",
            self.debounce_sec, self.blink_times, self.blink_period
        )

    # =============================
    # Callbacks / Helpers
    # =============================
    def _cb_lock(self, s: String):
        self.locked = bool((s.data or "").strip())

    def _cb_stt(self, msg: SpeechRecognitionCandidates):
        if self.locked:
            return
        now = time.time()
        if now - self._last_fire < self.debounce_sec:
            return

        # transcripts と confidence は同長の想定（足りないときは 1.0 を入れる）
        texts = list(getattr(msg, "transcript", []))
        confs = list(getattr(msg, "confidence", [])) or [1.0] * len(texts)
        if len(confs) < len(texts):
            confs += [1.0] * (len(texts) - len(confs))

        # どれか一つでもパターン & 信頼度を満たせば発火
        for t, c in zip(texts, confs):
            t_norm = (t or "").strip()
            if not t_norm or float(c) < self.min_conf:
                continue
            if any(p.search(t_norm) for p in self.patterns):
                self._trigger(t_norm, float(c))
                break

    def _trigger(self, matched_text="jedy", confidence=1.0):
        self._last_fire = time.time()

        # 1) /interaction_events に HUMAN_WAKE を publish
        self._log_evt("HUMAN_WAKE", meta={
            "text": matched_text,
            "confidence": float(confidence),
            "source": "/speech_to_text"
        })

        # 2) LED 点滅（ノンブロッキング）
        if self._blink_thread and self._blink_thread.is_alive():
            return
        self._blink_thread = threading.Thread(target=self._blink_once)
        self._blink_thread.daemon = True
        self._blink_thread.start()

    def _blink_once(self):
        """LED を一定回数点滅させる"""
        try:
            for _ in range(max(1, self.blink_times)):
                self.pub_led.publish(self.col_on)
                time.sleep(max(0.05, self.blink_period))
                self.pub_led.publish(self.col_off)
                time.sleep(max(0.05, self.blink_period))
        except rospy.ROSInterruptException:
            pass

    # あなたの実装例と同じシグネチャ／挙動
    def _log_evt(self, etype, meta=None):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = str(etype)
        e.actor_id = "wake_word_blinker"   # このノードID（任意）
        e.target_id = "jedy"               # 対象ID（任意）
        full = dict(self.diary_meta)
        if meta:
            full.update(meta)
        e.meta_json = json.dumps(full, ensure_ascii=False)
        self.pub_evt.publish(e)
        rospy.loginfo(
            "wake_word_blinker: published %s (text=%s, conf=%s)",
            e.event_type, full.get("text", ""), str(full.get("confidence", ""))
        )


def main():
    rospy.init_node("wake_word_blinker")
    WakeWordBlinker()
    rospy.spin()


if __name__ == "__main__":
    main()

