#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, time
from std_msgs.msg import String, Bool

try:
    from techrie_demo.msg import InteractionEvent
except Exception:
    # フォールバック（型が無い環境でも動かせる）
    from std_msgs.msg import String as InteractionEvent  # ダミー
    pass

class InactivityWatchdog:
    def __init__(self):
        rospy.init_node("inactivity_watchdog")

        # ===== Params =====
        self.to_sulk_sec      = float(rospy.get_param("~to_sulk_sec", 45.0))   # 無反応で何秒後にSULK
        self.sulk_dur_sec     = float(rospy.get_param("~sulk_dur_sec", 12.0))  # いじけ継続秒
        self.sulk_cooldown_sec= float(rospy.get_param("~sulk_cooldown_sec", 60.0)) # 再発火までの猶予
        self.wait_started     = bool(rospy.get_param("~wait_system_started", True))
        self.require_lock_clear = bool(rospy.get_param("~require_lock_clear", True))
        self.lock_topic       = rospy.get_param("~lock_topic", "/ui/speak_lock")

        # “反応あり”とみなす監視トピック（std_msgs/String 想定）
        default_watch = [
            "/human/praise", "/human/pet", "/human/offer_food", "/human/show_art",
            "/human/ok", "/human/ng", "/human/invite"
        ]
        self.watch_topics = rospy.get_param("~watch_topics", default_watch)

        # ===== State =====
        self._started = (not self.wait_started)  # True なら即監視開始
        self._last_active = time.time()
        self._sulk_active = False
        self._next_allowed_at = 0.0
        self._speak_lock = ""  # 空文字ならロック無し

        # ===== IO =====
        self.pub_evt = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
        if self.wait_started:
            rospy.Subscriber("/system/started", Bool, self._started_cb, queue_size=1)
        rospy.Subscriber(self.lock_topic, String, self._lock_cb, queue_size=1)

        # 監視トピックを購読
        for t in self.watch_topics:
            rospy.Subscriber(t, String, self._mark_active, queue_size=20)

        rospy.loginfo("inactivity_watchdog: watch=%s to_sulk=%.1fs dur=%.1fs cd=%.1fs",
                      ",".join(self.watch_topics), self.to_sulk_sec, self.sulk_dur_sec, self.sulk_cooldown_sec)

        self._loop()

    # ---------- callbacks ----------
    def _started_cb(self, m: Bool):
        self._started = bool(m.data)
        if self._started:
            self._last_active = time.time()

    def _lock_cb(self, s: String):
        self._speak_lock = (s.data or "").strip()

    def _mark_active(self, _m: String):
        self._last_active = time.time()
        # 反応が来たら“いじけ終わり”とみなす
        self._sulk_active = False

    # ---------- helpers ----------
    def _emit_sulk_start(self):
        now = time.time()
        self._sulk_active = True
        self._next_allowed_at = now + self.sulk_cooldown_sec

        try:
            evt = InteractionEvent()
            evt.stamp = rospy.Time.now()
            evt.event_type = "SULK_START"
            evt.actor_id = "watchdog"
            evt.target_id = "robot"
            evt.intensity = 1.0
            evt.meta_json = json.dumps({"dur": self.sulk_dur_sec, "reason": "no_human_input"}, ensure_ascii=False)
            self.pub_evt.publish(evt)
        except Exception:
            # フォールバック（型が無い環境）
            self.pub_evt.publish(String(json.dumps({"event_type":"SULK_START","dur":self.sulk_dur_sec})))
        rospy.loginfo("inactivity_watchdog: -> SULK_START dur=%.1fs", self.sulk_dur_sec)

    # ---------- main loop ----------
    def _loop(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            now = time.time()
            if self._started:
                idle = now - self._last_active
                cond_idle = (idle >= self.to_sulk_sec)
                cond_cd   = (now >= self._next_allowed_at)
                cond_lock = (not self.require_lock_clear) or (self._speak_lock == "")
                if (not self._sulk_active) and cond_idle and cond_cd and cond_lock:
                    self._emit_sulk_start()
            rate.sleep()

def main():
    InactivityWatchdog()

if __name__ == "__main__":
    main()
