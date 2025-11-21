#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

class StateAutoResetter:
    def __init__(self):
        rospy.init_node("kashiwagi_state_resetter")
        rospy.sleep(1)

        self.target_states = ["daily:happy", "daily:waking_up", "daily:goodbye", "talking_game:starting", "talking_game:happy", "move:surprised", "katakanashi:starting", "katakanashi:happy"]
        self.timeout_duration = rospy.Duration(5.0)  # 5秒

        self.cur_state = "unknown"
        self.state_enter_time = rospy.Time.now()

        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_state_timeout)

        rospy.loginfo("Launching State AutoResetter node ....")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.cur_state:
            rospy.loginfo(f"State changed: {self.cur_state} → {new_state}")
            self.cur_state = new_state
            self.state_enter_time = rospy.Time.now()

    def check_state_timeout(self, event):
        if self.cur_state in self.target_states:
            if self.cur_state in ["daily:happy", "daily:waking_up", "daily:goodbye"]:
                req_state = "daily:normal"
            elif self.cur_state in ["talking_game:starting", "talking_game:happy"]:
                req_state = "talking_game:listening_turn"
            elif self.cur_state in ["move:surprised"]:
                req_state = "move:staying"
            elif self.cur_state in ["katakanashi:starting"]:
                req_state = "katakanashi:playing"
            elif self.cur_state in ["katakanashi:happy"]:
                req_state = "katakanashi:playing"

            elapsed = rospy.Time.now() - self.state_enter_time
            if elapsed >= self.timeout_duration:
                rospy.loginfo(f"State '{self.cur_state}' timed out after {elapsed.to_sec()}s. Resetting to 'daily:normal'.")
                try:
                    resp = self.set_state_srv(req_state)
                    if resp.success:
                        rospy.loginfo(f"State reset successful: {resp.message}")
                    else:
                        rospy.logwarn(f"State reset failed: {resp.message}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to call service: {e}")

if __name__ == "__main__":
    try:
        StateAutoResetter()
    except rospy.ROSInterruptException:
        pass
