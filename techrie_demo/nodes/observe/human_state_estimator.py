#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, time
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import actionlib
from jsk_recognition_msgs.msg import VQATaskAction, VQATaskActionGoal
from techrie_demo.msg import InteractionEvent

class HumanStateEstimator:
    def __init__(self):
        rospy.init_node("human_state_estimator")

        # ===== Params =====
        self.period_sec = float(rospy.get_param("~period_sec", 4.0))  # 3〜5s推奨
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/color/image_raw")
        self.vqa_action   = rospy.get_param("~vqa_action",   "/vqa/inference_server")
        self.speak_lock_topic = rospy.get_param("~speak_lock_topic", "/ui/speak_lock")
        self.painting_topic   = rospy.get_param("~painting_active",  "/painting_active")

        # 質問（最初だけ存在確認→詳細）
        self.q_detect = rospy.get_param("~q_detect",
                                        "Are there any person in this image? (yes or no)")
        # この配列の文字列が VQA の result にそのまま question として返る前提
        self.questions = rospy.get_param("~questions", [
            "What is the person's general emotion? (one word)",
            "What is the person doing? (one short phrase)",
            "Does the person want help from the robot? (yes or no)",
            "Where is the person looking at? (left/center/right/up/down)"
        ])

        # ===== IO =====
        self.pub_state = rospy.Publisher("/human_state", String, queue_size=10)
        self.pub_evt   = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=20)
        rospy.Subscriber(self.speak_lock_topic, String, self._cb_lock, queue_size=1)
        rospy.Subscriber(self.painting_topic,   Bool,   self._cb_paint, queue_size=1)
        rospy.Subscriber(self.camera_topic,     Image,  self._cb_img,   queue_size=1)

        # VQA Action client
        self.cli = actionlib.SimpleActionClient(self.vqa_action, VQATaskAction)
        ok = self.cli.wait_for_server(rospy.Duration(3.0))
        if not ok:
            rospy.logwarn("human_state_estimator: VQA server '%s' not available.", self.vqa_action)

        # 内部状態
        self.last_img_msg = None
        self.locked = False
        self.painting = False
        self.last_json = None

        rospy.Timer(rospy.Duration(self.period_sec), self._tick, oneshot=False)
        rospy.loginfo("human_state_estimator: ready (period=%.1fs)", self.period_sec)

    # --------- Callbacks ---------
    def _cb_lock(self, s: String):
        self.locked = bool((s.data or "").strip())

    def _cb_paint(self, b: Bool):
        self.painting = bool(b.data)

    def _cb_img(self, m: Image):
        self.last_img_msg = m

    # --------- VQA helpers ---------
    def _vqa_send(self, questions, timeout=2.0):
        """questions: str or list[str] -> return result object or None"""
        if self.last_img_msg is None:
            return None
        if isinstance(questions, str):
            questions = [questions]
        goal = VQATaskActionGoal()
        goal.goal.image = self.last_img_msg
        goal.goal.questions = list(questions)
        try:
            self.cli.send_goal(goal.goal)
            ok = self.cli.wait_for_result(rospy.Duration(timeout))
            if not ok:
                try: self.cli.cancel_all_goals()
                except Exception: pass
                return None
            return self.cli.get_result()
        except Exception as e:
            rospy.logwarn("VQA call failed: %s", e)
            return None

    def _vqa_answer_list(self, res):
        """res -> list[(question, answer)]  / 空なら []"""
        out = []
        try:
            # jsk_recognition_msgs/VQATaskResult: result.result は配列
            for r in getattr(getattr(res, "result", None), "result", []) or []:
                q = getattr(r, "question", "")
                a = getattr(r, "answer", "")
                out.append((q, a))
        except Exception:
            pass
        return out

    def _ask_vqa_single(self, question, timeout=2.0):
        res = self._vqa_send(question, timeout=timeout)
        pairs = self._vqa_answer_list(res)
        return (pairs[0][1] if pairs else None)

    def _ask_vqa_batch(self, questions, timeout=2.5):
        res = self._vqa_send(questions, timeout=timeout)
        pairs = self._vqa_answer_list(res)
        # question -> answer の辞書へ
        return {q: a for q, a in pairs}

    # --------- Periodic tick ---------
    def _tick(self, _evt):
        # ゲート：発話ロック/描画中は抑制（監視は続ける）
        if self.locked or self.painting:
            return
        # VQA 未接続なら何もしない
        # （ここで wait_for_server を再試行しても良い）
        # present チェック（単一質問）
        ans = (self._ask_vqa_single(self.q_detect, timeout=1.5) or "").strip().lower()
        if not ans or ans.startswith("n"):  # no / none / …
            out = {"present": False}
        else:
            # 複数質問を一括で
            qa = self._ask_vqa_batch(self.questions, timeout=2.5)
            emotion = (qa.get(self.questions[0], "") if len(self.questions) > 0 else "")
            action  = (qa.get(self.questions[1], "") if len(self.questions) > 1 else "")
            needs   = (qa.get(self.questions[2], "") if len(self.questions) > 2 else "")
            gaze    = (qa.get(self.questions[3], "") if len(self.questions) > 3 else "")
            out = {
                "present": True,
                "emotion": emotion,
                "action":  action,
                "needs_help": needs.strip().lower().startswith("y"),
                "gaze":    gaze,
            }

        j = json.dumps(out, ensure_ascii=False)
        # 同一なら抑制
        if j == self.last_json:
            return
        self.last_json = j
        self.pub_state.publish(String(j))

        # 日記イベントも発行
        ev = InteractionEvent()
        ev.stamp = rospy.Time.now()
        ev.event_type = "HUMAN_STATE"
        ev.actor_id = "estimator"
        ev.target_id = "human"
        ev.meta_json = j
        self.pub_evt.publish(ev)

def main():
    HumanStateEstimator()
    rospy.spin()

if __name__ == "__main__":
    main()
