#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os, sys, traceback
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from rospkg import RosPack

# 成果物の有無で成否判定
def _check_outputs(paths: dict):
    ok = True
    missing = []
    for k in ("diary_json", "diary_image", "diary_combined"):
        p = paths.get(k)
        if not p:
            continue
        if not os.path.exists(p):
            ok = False
            missing.append((k, p))
    return ok, missing

def handle(_req):
    try:
        pkg = RosPack().get_path("techrie_demo")
        os.chdir(pkg)
        sys.path.insert(0, pkg)

        from config import resolve_paths_full
        paths = resolve_paths_full(None)

        rospy.loginfo("diary: pipeline start. targets="
                      "scenes_json=%s, captioned=%s, diary_json=%s, image=%s, combined=%s",
                      paths["scenes_json"], paths["captioned_json"],
                      paths["diary_json"], paths["diary_image"], paths["diary_combined"])

        from main_pipeline import main as run_pipeline
        run_pipeline(None)  # 今日

        ok, missing = _check_outputs(paths)
        if not ok:
            for k, p in missing:
                rospy.logwarn("diary: missing output %s -> %s", k, p)
            return TriggerResponse(False, "pipeline finished but outputs missing")

        # 成功！通知
        txt = rospy.Publisher("/robot_text", String, queue_size=1)
        emo = rospy.Publisher("/emotion/set", String, queue_size=1)
        rospy.sleep(0.05)
        txt.publish(String("今日の絵日記ができたよ！"))
        emo.publish(String("joy"))

        rospy.loginfo("diary: pipeline done. diary=%s image=%s combined=%s",
                      paths["diary_json"], paths["diary_image"], paths["diary_combined"])
        return TriggerResponse(True, "ok")
    except Exception as e:
        rospy.logerr("diary pipeline exception:\n%s", traceback.format_exc())
        return TriggerResponse(False, "pipeline failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("diary_pipeline_service")
    rospy.Service("/diary/make_today", Trigger, handle)
    rospy.loginfo("diary_pipeline_service ready: /diary/make_today")
    rospy.spin()

