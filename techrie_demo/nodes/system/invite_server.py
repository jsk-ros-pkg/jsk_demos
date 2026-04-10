#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, json
from std_msgs.msg import String as S
from techrie_demo.msg import InviteAction, InviteResult, InteractionEvent
from techrie_demo.srv import AskForItem, AskForItemRequest

class InviteServer:
    def __init__(self):
        rospy.init_node("invite_server")

        # ===== Params =====
        self.ask_service = rospy.get_param("~ask_service_name", "ask_for_item")
        self.item_name   = rospy.get_param("~ask_item_name", "invite_confirm")  # 0=OK,1=NG の判定に使うラベル
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 10.0))
        self.use_lock    = bool(rospy.get_param("~use_speak_lock", True))
        self.ascii_text  = bool(rospy.get_param("~ascii_text", False))  # 環境によって ascii セーフに

        # ===== Pub/Sub =====
        self.pub_text  = rospy.Publisher("/robot_text", S, queue_size=10)
        self.pub_emote = rospy.Publisher("/emotion/set", S, queue_size=10)
        self.pub_motion= rospy.Publisher("/motion/play", S, queue_size=10)
        self.pub_ie    = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
        self.pub_lock  = rospy.Publisher("/ui/speak_lock", S, queue_size=1)
        self.policy_pub= rospy.Publisher("/policy/next_action", S, queue_size=1)  # 使わないなら未使用でOK

        # ask サービス
        rospy.wait_for_service(self.ask_service)
        self.ask = rospy.ServiceProxy(self.ask_service, AskForItem)

        # Action server
        self.server = actionlib.SimpleActionServer("invite", InviteAction,
                                                   execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo("invite_server ready")

    def _say(self, text, color="white"):
        if self.ascii_text:
            text = text.encode("ascii", "ignore").decode("ascii")
        self.pub_text.publish(S(text))
        self.pub_emote.publish(S(color))
        rospy.loginfo("[invite] %s", text)

    def _lock(self):   self.pub_lock.publish(S("invite_server")) if self.use_lock else None
    def _unlock(self): self.pub_lock.publish(S(""))               if self.use_lock else None

    def _emit_evt(self, kind: str, ok: bool):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = "INVITE_OK" if ok else "INVITE_NG"
        e.actor_id = "robot"; e.target_id = "human"; e.intensity = 1.0
        e.meta_json = json.dumps({"kind": kind}, ensure_ascii=False)
        self.pub_ie.publish(e)

    def execute(self, goal):
        self._lock()
        try:
            phrase = getattr(goal, "phrase", "") or "いっしょに絵を描こう？ (0/1)"
            self._say(phrase, "yellow")

            # 0/1 を ask サービスで判定（OK: True, NG: False）
            try:
                resp = self.ask(AskForItemRequest(item_name=self.item_name))
                ok = bool(resp.ok)
            except Exception as e:
                rospy.logwarn("invite ask error: %s", e)
                ok = False

            if ok:
                self.pub_motion.publish(S("seq:nod"))
                self._emit_evt("invite", True)
                res = InviteResult(accepted=True)
                self.server.set_succeeded(res, "accepted")
            else:
                self.pub_motion.publish(S("seq:shake"))
                self._emit_evt("invite", False)
                res = InviteResult(accepted=False)
                self.server.set_succeeded(res, "rejected")

        finally:
            self._unlock()

def main():
    InviteServer()
    rospy.spin()

if __name__ == "__main__":
    main()
