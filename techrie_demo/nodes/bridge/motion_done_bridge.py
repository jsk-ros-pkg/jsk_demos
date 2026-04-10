#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json
from std_msgs.msg import Bool, String
try:
    from techrie_demo.msg import InteractionEvent
    HAS_IE = True
except Exception:
    HAS_IE = False

class MotionDoneBridge:
    def __init__(self):
        rospy.init_node("motion_done_bridge")
        self.pub = rospy.Publisher("/motion_done", Bool, queue_size=10)

        if HAS_IE:
            rospy.Subscriber("/interaction/event", InteractionEvent, self._ie_cb, queue_size=50)
            rospy.loginfo("motion_done_bridge: listen /interaction/event")
        else:
            rospy.Subscriber("/interaction/event_fallback", String, self._fallback_cb, queue_size=50)
            rospy.loginfo("motion_done_bridge: listen /interaction/event_fallback")

    def _emit(self, ok: bool):
        self.pub.publish(Bool(data=bool(ok)))

    def _ie_cb(self, e: InteractionEvent):
        if (e.event_type or "").upper() == "MOTION_END":
            # success は meta_json 内に入っている設計／なければ True 扱い
            ok = True
            try:
                meta = json.loads(e.meta_json or "{}")
                ok = bool(meta.get("success", True))
            except Exception:
                pass
            self._emit(ok)

    def _fallback_cb(self, s: String):
        txt = s.data or ""
        # 例: "[InteractionEvent] {...}"
        try:
            j = txt[txt.find("{"):]
            meta = json.loads(j)
            if (meta.get("event_type","").upper()=="MOTION_END"):
                ok = bool((meta.get("meta") or {}).get("success", True))
                self._emit(ok)
        except Exception:
            pass

if __name__ == "__main__":
    MotionDoneBridge(); rospy.spin()
