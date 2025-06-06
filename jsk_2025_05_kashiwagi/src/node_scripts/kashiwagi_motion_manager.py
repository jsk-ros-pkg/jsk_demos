#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import kashiwagi_utils

class MotionManager:
    def __init__(self):
        self.current_kashiwagi_state = "unknown"
        self.prev_kashiwagi_state = "unknown"
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)

        kashiwagi_utils.servo_on()
        rospy.loginfo("Launching Motion Manager node ....")

        self.rate = rospy.Rate(5)  # 5Hz で動作チェック
        self.main_loop()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_kashiwagi_state == "talking_game:speaking_turn":
                kashiwagi_utils.speaking_mode()  # 繰り返し実行される
            elif self.current_kashiwagi_state == "talking_game:listening_turn":
                kashiwagi_utils.breath_mode()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MotionManager()
    except rospy.ROSInterruptException:
        pass
