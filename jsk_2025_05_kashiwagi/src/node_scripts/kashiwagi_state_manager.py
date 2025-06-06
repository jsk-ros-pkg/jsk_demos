#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState, SetKashiwagiStateResponse

class StateManagerNode:
    def __init__(self):
        rospy.init_node("state_manager_node")

        self.state = "idle"
        self.pub = rospy.Publisher("/kashiwagi_state", String, queue_size=10)

        # 状態変更サービスの登録
        self.srv = rospy.Service("/set_kashiwagi_state", SetKashiwagiState, self.handle_set_state)

        # 周期的に状態をbroadcast（例：1Hz）
        rospy.Timer(rospy.Duration(1.0), self.broadcast_state)

        rospy.loginfo("State Manager Node launched.")
        rospy.spin()

    def handle_set_state(self, req):
        if req.new_state == self.state:
            msg = f"State already '{self.state}', no change."
            return SetKashiwagiStateResponse(success=False, message=msg)

        rospy.loginfo(f"State changing: {self.state} → {req.new_state}")
        self.state = req.new_state
        self.pub.publish(self.state)  # 状態が変わったときはすぐに publish
        return SetKashiwagiStateResponse(success=True, message=f"State changed to {self.state}")

    def broadcast_state(self, event):
        self.pub.publish(self.state)

if __name__ == "__main__":
    try:
        StateManagerNode()
    except rospy.ROSInterruptException:
        pass
