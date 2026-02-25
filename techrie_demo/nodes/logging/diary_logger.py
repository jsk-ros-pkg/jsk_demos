#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
InteractionEvent を受けて:
- 既存の Markdown 日記 (~/.techrie_demo/diary/YYYY-MM-DD.md) に1行追記
- 必要なイベントでは /diary/snapshot サービスを叩いて写真を保存
- "object_images/YYYY/MM/DD/YYYY-MM-DD.json" に「場面レコード」を追記

追加:
- extra_string_topics を購読し、文字列メッセージも日記へ統一的に追記
- イベント文脈（名前/フェーズ/目標/回/コホート）を Markdown ヘッダと scene JSON に自動付与

★変更点（この版）:
- scene.top-level の emotion を meta["emotion"] → /profile/emotion → "neutral" の優先で埋める
- robot_state に emotion を追加
"""

import os, json
import sqlite3
from datetime import datetime
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rospkg import RosPack
from techrie_demo.msg import InteractionEvent

# ---------- ユーティリティ ----------

def _ensure_dir(p):
    if not os.path.isdir(p):
        os.makedirs(p, exist_ok=True)

def _fmt_ts(t=None):
    return (t or datetime.now()).strftime("%H:%M:%S")

def _safe_json_load(s):
    try:
        return json.loads(s) if s else {}
    except Exception:
        return {}

def _get_param(name, default):
    try:
        return rospy.get_param(name, default)
    except Exception:
        return default

# ---------- ロガー本体 ----------

class DiaryLogger(object):
    def __init__(self):
        rospy.init_node("diary_logger")

        # パラメータ
        self.enable_snapshot = rospy.get_param("~enable_snapshot", True)
        self.snap_events = rospy.get_param(
            "~snap_events",
            ["PAINT_END", "HUMAN_SHOW_ART", "INVITE_OK", "INVITE_NG", "GREETING_HELLO"],
        )
        self.md_root = os.path.expanduser(rospy.get_param("~md_root", "~/.ros/techrie_demo/diary"))
        self.profile_prefix = rospy.get_param("~profile_params_prefix", "/profile")
        self.env_prefix = rospy.get_param("~env_params_prefix", "/env")
        self.snapshot_srv_name = rospy.get_param("~snapshot_service", "/diary/snapshot")
        self.append_robot_text = rospy.get_param("~append_robot_text", True)
        # ★ 追加: 直近のロボット発話を次の scene に紐づける許容時間（秒）
        self.utter_attach_sec = float(rospy.get_param("~utterance_attach_window_sec", 3.0))
        # ★ 追加: /robot_text の最近値を保持
        self._last_robot_text = ""
        self._last_robot_text_ts = rospy.Time(0)

        # 文字列トピックの追加取り込み
        self.extra_string_topics = rospy.get_param("~extra_string_topics", [])

        # ★ イベント文脈（launch から与える）
        self.event_meta = {
            "name":   rospy.get_param("~event_name",   ""),   # 例: "Moon Project"
            "phase":  rospy.get_param("~event_phase",  ""),   # 例: "kickoff" / "regular" / "finale"
            "goal":   rospy.get_param("~event_goal",   ""),   # 例: "みんなで月を完成させる"
            "day":    rospy.get_param("~event_day",    ""),   # 例: "1" (回数)
            "cohort": rospy.get_param("~event_cohort", ""),   # 例: "A組" など
        }

        # 出力先（object_images のルートをパッケージから推定）
        self.pkg_root = RosPack().get_path("techrie_demo")

        # 追加演出（任意）
        self.pub_text = rospy.Publisher("/robot_text", String, queue_size=3)
        # ★ 追加: /robot_text を購読（誰が喋っても拾える）
        rospy.Subscriber("/robot_text", String, self.on_robot_text, queue_size=200)

        # スナップショットサービス
        self.snap_cli = rospy.ServiceProxy(self.snapshot_srv_name, Trigger)

        # SQLite
        self.db_path = os.path.join(os.path.expanduser("~/.ros/techrie_demo"), "diary.db")
        _ensure_dir(os.path.dirname(self.db_path))
        self._ensure_db()

        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        os.makedirs(self.md_root, exist_ok=True)

        # 購読
        rospy.Subscriber("/interaction_events", InteractionEvent, self.on_event, queue_size=100)
        self._extra_subs = []
        for tname in self.extra_string_topics:
            self._extra_subs.append(
                rospy.Subscriber(tname, String, lambda m, tn=tname: self.on_string_topic(tn, m), queue_size=200)
            )

        rospy.loginfo("diary_logger ready: snapshots=%s events=%s extra=%s event_meta=%s",
                      self.enable_snapshot, self.snap_events, self.extra_string_topics, self.event_meta)

    # ----- DB 準備 -----
    def _ensure_db(self):
        try:
            con = sqlite3.connect(self.db_path)
            cur = con.cursor()
            cur.execute("""
                CREATE TABLE IF NOT EXISTS events (
                  ts TEXT, type TEXT, actor TEXT, target TEXT, intensity REAL, meta_json TEXT
                )
            """)
            con.commit(); con.close()
        except Exception as e:
            rospy.logwarn("diary_logger: sqlite init failed: %s", e)

    def _insert_db(self, e: InteractionEvent):
        try:
            con = sqlite3.connect(self.db_path)
            cur = con.cursor()
            cur.execute("INSERT INTO events VALUES (?,?,?,?,?,?)", (
                datetime.fromtimestamp(e.stamp.to_sec()).isoformat(),
                e.event_type, e.actor_id, e.target_id, float(getattr(e, "intensity", 0.0) or 0.0),
                e.meta_json or ""
            ))
            con.commit(); con.close()
        except Exception as ex:
            rospy.logwarn("diary_logger: sqlite insert failed: %s", ex)

    # ----- Markdown 1行追記 -----
    def _append_markdown(self, e: InteractionEvent, meta: dict):
        _ensure_dir(self.md_root)
        day = datetime.now().date().isoformat()
        path = os.path.join(self.md_root, f"{day}.md")

        # ファイル作成時にその日のイベント文脈をヘッダに焼き込む
        if not os.path.exists(path):
            with open(path, "w", encoding="utf-8") as f:
                f.write(f"# Jedy Diary {day}\n\n")
                # イベント文脈ヘッダ（空は出さない）
                lines = []
                if self.event_meta.get("name"):
                    lines.append(f"- **Event**: {self.event_meta['name']}")
                if self.event_meta.get("phase"):
                    lines.append(f"- **Phase**: {self.event_meta['phase']}")
                if self.event_meta.get("day"):
                    lines.append(f"- **Day**: {self.event_meta['day']}")
                if self.event_meta.get("cohort"):
                    lines.append(f"- **Cohort**: {self.event_meta['cohort']}")
                if self.event_meta.get("goal"):
                    lines.append(f"- **Goal**: {self.event_meta['goal']}")
                if lines:
                    f.write("\n".join(lines) + "\n\n")

        line = f"- {_fmt_ts()} **{e.event_type}** (actor={e.actor_id}, target={e.target_id}, intensity={getattr(e, 'intensity', 0.0)})"
        if meta:
            hints = []
            for k in ("color","phrase","object","stroke_id","mode","emotion","human_action","text","topic"):
                if k in meta:
                    hints.append(f"{k}={meta[k]}")
            if hints:
                line += "  " + ", ".join(hints)
        with open(path, "a", encoding="utf-8") as f:
            f.write(line + "\n")

    # ----- 場面 JSON 追記 -----
    def _append_scene_json(self, entry: dict):
        # ここでイベント文脈を毎回付加
        if self.event_meta:
            entry["event"] = dict(self.event_meta)

        now = datetime.now()
        day_dir = os.path.join(self.pkg_root, "object_images",
                               now.strftime("%Y"), now.strftime("%m"), now.strftime("%d"))
        _ensure_dir(day_dir)
        json_path = os.path.join(day_dir, now.strftime("%Y-%m-%d") + ".json")

        data = []
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                pass
        data.append(entry)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def _take_snapshot(self) -> str:
        try:
            self.snap_cli.wait_for_service(rospy.Duration(2.0))
            res = self.snap_cli()
            if res.success:
                return res.message  # 例: "153055.jpg"
        except Exception as ex:
            rospy.logwarn("diary_logger: snapshot failed: %s", ex)
        return None

    # ---- 文字列トピック用の統一コールバック ----
    def on_string_topic(self, topic_name: str, s: String):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = "TOPIC_STRING"
        e.actor_id = "topic"
        e.target_id = topic_name
        e.intensity = 0.0
        meta = {"text": (s.data or ""), "topic": topic_name}
        e.meta_json = json.dumps(meta, ensure_ascii=False)

        self._insert_db(e)
        self._append_markdown(e, meta)

        # ★ 追加（任意）：文字列トピックでもスナップを撮るなら scene も残せる
        if self.enable_snapshot and ("TOPIC_STRING" in self.snap_events):
            filename = self._take_snapshot()
            if filename:
                scene = {
                    "image_filename": filename,
                    "selected_object": meta.get("color") or "unknown",
                    "selected_action": e.event_type.lower(),
                    "emotion": (_get_param(f"{self.profile_prefix}/emotion", "neutral")),
                    "robot_state": self._collect_robot_state(e.event_type, meta, e.stamp),
                    "human_state": self._collect_human_state(meta),
                    "extra_info": self._collect_env(),
                    "notes": meta.get("text",""),
                }
                self._append_scene_json(scene)
    # ---- InteractionEvent 本流 ----
    def _collect_robot_state(self, last_event_type: str, meta: dict = None, event_time: rospy.Time = None):
        p = self.profile_prefix
        def g(name, default):
            try:
                return float(_get_param(f"{p}/{name}", default))
            except Exception:
                return default
        mode = "with_people" if "HUMAN_" in last_event_type else "solo"
        state = {
            "interest": g("interest", 0.5),
            "fatigue":  g("fatigue", 0.3),
            "last_action": last_event_type,
            "mode": mode,
            "sociality_score": g("sociality", 0.5),
            # ★ 追加：現在の感情を robot_state にも保存
            "emotion": _get_param(f"{p}/emotion", "neutral"),
        }
        # ★ 追加: utterance を meta or /robot_text から補完
        try:
            if event_time is None:
                event_time = rospy.Time.now()
            utter = ""
            if isinstance(meta, dict):
                # 1) GPT提案が採用された発話
                if meta.get("taken") and isinstance(meta.get("suggestion"), dict):
                    utter = (meta["suggestion"].get("utterance") or "").strip()
                # 2) 文字列トピック等のテキスト
                if not utter and isinstance(meta.get("text"), str):
                    utter = meta["text"].strip()
                # 2b) 上流で 'utterance' や 'phrase' を直入れしてくる場合にも対応
                if not utter and isinstance(meta.get("utterance"), str):
                    utter = meta["utterance"].strip()
                if not utter and isinstance(meta.get("phrase"), str):
                    utter = meta["phrase"].strip()
            # 3) 直近の /robot_text を時間窓内なら採用
            if (not utter) and self._last_robot_text:
                if (event_time - self._last_robot_text_ts).to_sec() <= self.utter_attach_sec:
                    utter = self._last_robot_text
            if utter:
                state["utterance"] = utter
        except Exception:
            pass
        return state

    def _collect_human_state(self, meta: dict):
        return {
            "emotion": meta.get("human_emotion","unknown"),
            "action":  meta.get("human_action","unknown"),
            "needs":   meta.get("needs","unknown"),
            "interaction": meta.get("interaction","unknown"),
            "gaze_target": meta.get("gaze","unknown"),
        }

    def _collect_env(self):
        p = self.env_prefix
        return {
            "weather": _get_param(f"{p}/weather","unknown"),
            "temperature": _get_param(f"{p}/temperature","unknown"),
        }

    def on_event(self, e: InteractionEvent):
        meta = _safe_json_load(e.meta_json)
        self._insert_db(e)
        self._append_markdown(e, meta)

        filename = None
        if self.enable_snapshot and (e.event_type in self.snap_events):
            filename = self._take_snapshot()

        if filename:
            scene = {
                "image_filename": filename,
                "selected_object": meta.get("object") or meta.get("color") or "unknown",
                "selected_action": e.event_type.lower(),
                # ★ 変更：meta無ければ /profile/emotion → "neutral"
                "emotion": (meta.get("emotion") or _get_param(f"{self.profile_prefix}/emotion", "neutral")),
                "robot_state": self._collect_robot_state(e.event_type, meta, e.stamp),
                "human_state": self._collect_human_state(meta),
                "extra_info": self._collect_env(),
                "notes": meta.get("note",""),
            }
            self._append_scene_json(scene)
            rospy.loginfo("diary_logger: scene appended -> %s (%s)", filename, e.event_type)

        if self.append_robot_text and e.event_type in ("PAINT_END","HUMAN_SHOW_ART"):
            self.pub_text.publish(String("メモしたよ！"))

    # ★ 追加: /robot_text の最近値を保持
    def on_robot_text(self, msg: String):
        self._last_robot_text = (msg.data or "").strip()
        self._last_robot_text_ts = rospy.Time.now()

def main():
    DiaryLogger()
    rospy.spin()

if __name__ == "__main__":
    main()

