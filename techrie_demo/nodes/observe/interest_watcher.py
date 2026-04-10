#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from techrie_demo.msg import InteractionEvent

class InterestWatcher:
    def __init__(self):
        rospy.init_node("interest_watcher")

        # ===== Params =====
        # 入出力トピック
        self.suggestion_topic   = rospy.get_param("~suggestion_topic", "/interest/suggestion")
        self.speak_lock_topic   = rospy.get_param("~speak_lock_topic", "/ui/speak_lock")
        self.painting_topic     = rospy.get_param("~painting_topic", "/painting_active")
        self.text_topic         = rospy.get_param("~text_topic", "/robot_text")
        self.emote_topic        = rospy.get_param("~emote_topic", "/emotion/set")
        self.motion_topic       = rospy.get_param("~motion_topic", "/motion/play")

        # 視線系（首ふり）
        # 既存の robot_behavior は /detected_object_pose(String) を購読してラベルから視線を向けます
        self.gaze_label_topic   = rospy.get_param("~gaze_label_topic", "/detected_object_pose")
        # Point(x,y,z) を直接送りたい場合はこちら（robot_behavior 側で /paint_position を購読する必要あり）
        self.gaze_point_topic   = rospy.get_param("~gaze_point_topic", "/paint_position")
        self.enable_point_target= bool(rospy.get_param("~enable_point_target", False))

        # 実行条件
        self.allow_during_paint = bool(rospy.get_param("~allow_during_paint", False))
        self.min_interval_sec   = float(rospy.get_param("~min_interval_sec", 6.0))
        self.min_priority       = float(rospy.get_param("~min_priority", 0.35))

        # 表示制御
        self.announce_text      = bool(rospy.get_param("~announce_via_text", True))
        self.max_utter_len      = int(rospy.get_param("~max_utter_len", 64))

        # 意図→既定演出
        self.intent_pose_map = rospy.get_param("~intent_pose_map", {
            "greet":  "pose:hi",
            "praise": "seq:nod",
            "ask":    "pose:reset",
            "invite": "pose:hi",
            "comment":"pose:point_hand"
        })
        self.intent_emote_map = rospy.get_param("~intent_emote_map", {
            "greet":  "joy",
            "praise": "joy",
            "ask":    "interest",
            "invite": "interest",
            "comment":"calm"
        })

        # ===== IO =====
        self.pub_text   = rospy.Publisher(self.text_topic,   String, queue_size=10)
        self.pub_emote  = rospy.Publisher(self.emote_topic,  String, queue_size=10)
        self.pub_motion = rospy.Publisher(self.motion_topic, String, queue_size=10)
        self.pub_evt    = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=20)
        self.pub_gaze_label = rospy.Publisher(self.gaze_label_topic, String, queue_size=10)
        self.pub_gaze_point = rospy.Publisher(self.gaze_point_topic, Point,  queue_size=10)

        rospy.Subscriber(self.suggestion_topic, String, self._cb_suggestion, queue_size=10)
        rospy.Subscriber(self.speak_lock_topic, String, self._cb_lock, queue_size=1)
        rospy.Subscriber(self.painting_topic,   Bool,   self._cb_paint, queue_size=1)

        # 内部状態
        self.locked    = False
        self.painting  = False
        self.last_exec = 0.0

        rospy.loginfo(
            "interest_watcher: ready (interval=%.1fs, min_priority=%.2f, allow_during_paint=%s, point_target=%s)",
            self.min_interval_sec, self.min_priority, self.allow_during_paint, self.enable_point_target
        )

    # ---------- callbacks ----------
    def _cb_lock(self, s: String):
        self.locked = bool((s.data or "").strip())

    def _cb_paint(self, b: Bool):
        self.painting = bool(b.data)

    def _cb_suggestion(self, s: String):
        raw = (s.data or "").strip()
        taken = False
        reason = ""

        # 1) 解析
        try:
            sug = json.loads(raw)
        except Exception:
            sug = {"utterance": raw, "intent": "comment", "priority": 0.3}

        utter   = (sug.get("utterance") or "").strip()
        intent  = (sug.get("intent") or "comment").strip()
        try:
            priority = float(sug.get("priority", 0.0))
        except Exception:
            priority = 0.0
        pose    = (sug.get("pose") or "").strip()
        emote   = (sug.get("emote") or "").strip()

        # targetは gpt_detection から "target" もしくは後方互換で "look_at" を想定
        target  = sug.get("target", None) or sug.get("look_at", None)

        # 2) ゲーティング
        now = time.time()
        if self.locked:
            reason = "locked"
            return self._log_evt("GPT_ACT", taken, sug, reason)
        if self.painting and not self.allow_during_paint:
            reason = "painting"
            return self._log_evt("GPT_ACT", taken, sug, reason)
        if (now - self.last_exec) < self.min_interval_sec:
            reason = "cooldown"
            return self._log_evt("GPT_ACT", taken, sug, reason)
        if priority < self.min_priority:
            reason = "low_priority"
            return self._log_evt("GPT_ACT", taken, sug, reason)

        # 3) 実行（不足要素のフォールバック）
        if not emote:
            emote = self.intent_emote_map.get(intent, "calm")
        if not pose:
            pose = self.intent_pose_map.get(intent, "")

        if utter and self.announce_text:
            if len(utter) > self.max_utter_len:
                utter = utter[:self.max_utter_len-1] + "…"
            self.pub_text.publish(String(utter))
        if emote:
            self.pub_emote.publish(String(emote))
        if pose:
            self.pub_motion.publish(String(pose))

        # 4) 視線ターゲットの publish
        #   - ラベル指定: {"type":"label","label":"left_top"} など => /detected_object_pose(String)
        #   - 座標指定:   {"type":"point3d","x":0.1,"y":-0.3,"z":0.2} => /paint_position(Point) ※enable_point_targetがTrueかつ購読が有効な場合
        if isinstance(target, dict):
            ttype = (target.get("type") or target.get("kind") or "").lower()
            if ttype in ("label", "dir", "direction"):
                label = (target.get("label") or target.get("dir") or "center_center").strip()
                # robot_behavior の neck_motion_callback が扱えるラベル群に合わせる
                # ["left_top","center_top","right_top","left_center","center_center","right_center","left_bottom","center_bottom","right_bottom","right_bottom2"]
                self.pub_gaze_label.publish(String(label))
            elif ttype in ("point3d", "point"):
                if self.enable_point_target:
                    try:
                        x = float(target.get("x", 0.0))
                        y = float(target.get("y", 0.0))
                        z = float(target.get("z", 0.2))
                        self.pub_gaze_point.publish(Point(x=x, y=y, z=z))
                    except Exception:
                        pass  # 値が不正なら無視

        self.last_exec = now
        taken = True
        self._log_evt("GPT_ACT", taken, sug, reason)

    # ---------- diary logging ----------
    def _log_evt(self, etype, taken, suggestion, reason=""):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = etype
        e.actor_id = "interest_watcher"
        e.target_id = "scene"
        meta = {
            "taken": bool(taken),
            "reason": reason,
            "suggestion": suggestion
        }
        e.meta_json = json.dumps(meta, ensure_ascii=False)
        self.pub_evt.publish(e)

def main():
    InterestWatcher()
    rospy.spin()

if __name__ == "__main__":
    main()
