#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, random, json
from std_msgs.msg import String, Bool

class IdleMaintainer:
    """
    アイドル挙動（微動・センター戻し）を管理するノード。

    改訂点（本版）:
    - これまでの /idle/paused (Bool) に加え、/idle/enabled (Bool) にも対応。
      * /idle/paused: True → 停止, False → 再開
      * /idle/enabled: True → 再開, False → 停止
    - 再開時に一度センターへ戻すオプション (~center_on_resume)
    - 既存の構造・振る舞い（micro_ids の重み付き選択 等）は維持
    """

    def __init__(self):
        rospy.init_node("idle_maintainer")

        # ========= Params =========
        self.motion_topic = rospy.get_param("~motion_topic", "/motion/play")
        # これだけ無操作なら「アイドル」
        self.idle_gap = float(rospy.get_param("~idle_gap_sec", 8.0))
        # 次の発火までのランダム範囲（秒）
        self.min_period = float(rospy.get_param("~min_period_sec", 2.5))
        self.max_period = float(rospy.get_param("~max_period_sec", 5.0))

        # 微動として扱うID（これらは「本動作」と見なさない）
        self.micro_ids = rospy.get_param("~micro_ids",
            ["seq:micro_pitch_s", "seq:micro_yaw_s", "seq:micro_yaw_m"]
        )
        # 重み（micro_idsと同じ長さ）。大きいほど選ばれやすい
        self.micro_weights = rospy.get_param("~micro_weights", [0.5, 0.4, 0.1])

        # たまに中立に戻す
        self.center_id = rospy.get_param("~center_id", "pose:head_center")
        self.center_every = int(rospy.get_param("~center_every", 5))  # N回に1回 center

        # 再開時のセンター戻し（オプション）
        self.center_on_resume = bool(rospy.get_param("~center_on_resume", True))

        # ========= State / IO =========
        self.pub = rospy.Publisher(self.motion_topic, String, queue_size=1)
        rospy.Subscriber(self.motion_topic, String, self._on_motion)

        # 停止/再開：互換のため両トピックを受け付ける
        self.pause_topic = rospy.get_param("~pause_topic", "/idle/paused")  # 既存互換
        self.paused = False
        rospy.Subscriber(self.pause_topic, Bool, self._on_pause, queue_size=1)
        # 新規：enabled でも切替可能に
        self.enabled_topic = rospy.get_param("~enabled_topic", "/idle/enabled")
        rospy.Subscriber(self.enabled_topic, Bool, self._on_enabled, queue_size=1)

        self.last_real_activity = time.time()  # 最後の「本動作」時刻
        self.next_emit_at = time.time() + self.idle_gap
        self.emit_count = 0

        rospy.Timer(rospy.Duration(0.2), self._tick)
        rospy.loginfo(
            "idle_maintainer: watching '%s' (idle_gap=%.1fs, period=[%.1f,%.1f]s), "
            "pause_topic=%s, enabled_topic=%s",
            self.motion_topic, self.idle_gap, self.min_period, self.max_period,
            self.pause_topic, self.enabled_topic
        )

    # 「本動作」（= micro_ids 以外）が来たら時刻を更新
    def _on_motion(self, msg: String):
        payload = (msg.data or "").strip()
        # JSONも考慮（{"id":"..."} 形式なら抽出）
        motion_id = payload
        if payload.startswith("{"):
            try:
                obj = json.loads(payload)
                motion_id = str(obj.get("id", payload))
            except Exception:
                motion_id = payload
        # 自分の“微動”やセンター戻しは無視（= アイドル継続）
        if motion_id in self.micro_ids or motion_id == self.center_id:
            return
        # それ以外は「本動作」
        self.last_real_activity = time.time()
        # 本動作の直後は次の発火時刻を先送り（連続発火を防ぐ）
        self.next_emit_at = self.last_real_activity + self.idle_gap
        self.emit_count = 0

    # 既存互換: /idle/paused (True→停止)
    def _on_pause(self, msg: Bool):
        want_pause = bool(msg.data)
        if want_pause == self.paused:
            return
        self.paused = want_pause
        rospy.loginfo("idle_maintainer: %s via /idle/paused", "PAUSED" if self.paused else "RESUMED")
        if not self.paused and self.center_on_resume:
            # 再開時に一発センターへ
            self.pub.publish(String(self.center_id))

    # 新規: /idle/enabled (True→再開 / False→停止)
    def _on_enabled(self, msg: Bool):
        enabled = bool(msg.data)
        want_pause = (not enabled)
        if want_pause == self.paused:
            return
        self.paused = want_pause
        rospy.loginfo("idle_maintainer: %s via /idle/enabled", "PAUSED" if self.paused else "RESUMED")
        if not self.paused and self.center_on_resume:
            self.pub.publish(String(self.center_id))

    def _choose_micro(self) -> str:
        # 重み付きランダム選択
        weights = self.micro_weights
        ids = self.micro_ids
        if not ids:
            return self.center_id
        if len(weights) != len(ids):
            weights = [1.0] * len(ids)
        total = float(sum(max(w, 0.0) for w in weights)) or 1.0
        r = random.uniform(0, total)
        acc = 0.0
        for i, w in enumerate(weights):
            acc += max(w, 0.0)
            if r <= acc:
                return ids[i]
        return ids[-1]

    def _schedule_next(self):
        self.next_emit_at = time.time() + random.uniform(self.min_period, self.max_period)

    def _tick(self, _evt):
        # 一時停止中は何もしない
        if self.paused:
            return

        now = time.time()
        # まだアイドルでない
        if (now - self.last_real_activity) < self.idle_gap:
            return
        # 時刻になったら発火
        if now >= self.next_emit_at:
            self.emit_count += 1
            # たまにセンターへ
            if self.center_every > 0 and (self.emit_count % self.center_every == 0):
                self.pub.publish(String(self.center_id))
            else:
                micro = self._choose_micro()
                self.pub.publish(String(micro))
            self._schedule_next()

if __name__ == "__main__":
    IdleMaintainer()
    rospy.spin()
