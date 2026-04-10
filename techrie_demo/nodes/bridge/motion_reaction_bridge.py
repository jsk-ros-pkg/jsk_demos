#!/usr/bin/env python3
# coding: utf-8
import os
import time
import yaml
import threading

import rospy
from std_msgs.msg import String, Bool
# 型自動検出に rostopic を使う
import rostopic

DEFAULT_INTENTS = ["praise", "pet", "offer_food", "show_art", "invite"]

class MotionReactionBridge:
    def __init__(self):
        rospy.init_node("motion_reaction_bridge")

        # 設定
        self.intents = rospy.get_param("~intents", DEFAULT_INTENTS)
        self.cooldown_sec = float(rospy.get_param("~cooldown_sec", 1.5))
        self.motion_topic = rospy.get_param("~motion_topic", "/motion/play")
        self.human_ns = rospy.get_param("~human_ns", "/human")
        self.started_topic = rospy.get_param("~started_topic", "/system/started")
        self.package_root = rospy.get_param("~package_root", "")  # 明示指定もOK

        # マップの読み込み
        cfg_path = rospy.get_param("~motion_map_path", "")
        if not cfg_path:
            try:
                import rospkg
                rp = rospkg.RosPack()
                pkg_path = self.package_root or rp.get_path("techrie_demo")
            except Exception:
                pkg_path = self.package_root or os.path.expanduser("~/catkin_ws/src/techrie_demo")
            cfg_path = os.path.join(pkg_path, "config", "motion_map.yaml")

        self._map_lock = threading.Lock()
        self.motion_map = self._load_map(cfg_path)
        self.map_path = cfg_path
        self.last_fire_at = {k: 0.0 for k in self.intents}
        self.started = False

        # 出力
        self.pub_motion = rospy.Publisher(self.motion_topic, String, queue_size=10, latch=False)
        rospy.Subscriber(self.started_topic, Bool, self._on_started, queue_size=1)

        # /human/* を実型で購読する（発行者が現れるまで待機→購読）
        self._subs = {}
        for intent in self.intents:
            topic = "{}/{}".format(self.human_ns.rstrip("/"), intent)
            th = threading.Thread(target=self._subscribe_when_available, args=(topic, intent), daemon=True)
            th.start()
            rospy.loginfo("motion_reaction_bridge: waiting publisher for %s ...", topic)

        # 設定ファイルのホットリロード
        self._watch_thread = threading.Thread(target=self._watch_map, daemon=True)
        self._watch_thread.start()

        rospy.loginfo("motion_reaction_bridge: ready (map=%s, motion_topic=%s)", self.map_path, self.motion_topic)

    def _on_started(self, msg):
        self.started = bool(msg.data)

    def _subscribe_when_available(self, topic, intent):
        """発行者の型を検出してから購読する。"""
        msg_class = None
        # publisher が立ち上がるまで待つ（1秒間隔でリトライ）
        while not rospy.is_shutdown() and msg_class is None:
            try:
                msg_class, _, _ = rostopic.get_topic_class(topic, blocking=False)
                if msg_class is None:
                    time.sleep(1.0)
            except Exception:
                time.sleep(1.0)
        if rospy.is_shutdown():
            return
        # 実型で購読開始
        self._subs[topic] = rospy.Subscriber(topic, msg_class, self._make_cb(intent), queue_size=10)
        rospy.loginfo("motion_reaction_bridge: subscribed %s as %s", topic, msg_class.__name__)

    def _make_cb(self, intent):
        def _cb(_msg):
            if not self.started:
                return
            now = time.time()
            if now - self.last_fire_at.get(intent, 0.0) < self.cooldown_sec:
                return
            cmd = self._resolve_cmd(intent)
            if cmd:
                self.pub_motion.publish(String(data=cmd))
                self.last_fire_at[intent] = now
                rospy.loginfo("motion_reaction_bridge: %s -> %s", intent, cmd)
        return _cb

    def _resolve_cmd(self, intent):
        with self._map_lock:
            return (self.motion_map.get(intent)
                    or self.motion_map.get("default")
                    or "pose:reset")

    def _load_map(self, path):
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                raise ValueError("motion_map.yaml must be a dict")
            return data
        except Exception as e:
            rospy.logwarn("motion_reaction_bridge: failed to load %s (%s). Using minimal defaults.", path, e)
            return {"default": "pose:reset"}

    def _watch_map(self):
        mtime_prev = 0
        while not rospy.is_shutdown():
            try:
                mtime = os.path.getmtime(self.map_path)
                if mtime != mtime_prev:
                    new_map = self._load_map(self.map_path)
                    with self._map_lock:
                        self.motion_map = new_map
                    mtime_prev = mtime
                    rospy.loginfo("motion_reaction_bridge: reloaded %s", self.map_path)
            except Exception:
                pass
            time.sleep(5.0)

if __name__ == "__main__":
    MotionReactionBridge()
    rospy.spin()
