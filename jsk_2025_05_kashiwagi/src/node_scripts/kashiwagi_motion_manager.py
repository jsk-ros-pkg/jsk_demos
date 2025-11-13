#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
import kashiwagi_utils

class MotionManager:
    def __init__(self):
        self.current_kashiwagi_state = "unknown"
        self.prev_kashiwagi_state = "unknown"
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        rospy.Subscriber('/neck_yaw_angle', Float32, self.look_at_direction_callback)

        self.neck_yaw_angle = 0
        kashiwagi_utils.servo_on()
        rospy.loginfo("Launching Motion Manager node ....")

        self.rate = rospy.Rate(5)  # 5Hz で動作チェック
        self.main_loop()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data

    def look_at_direction_callback(self, msg):
        self.neck_yaw_angle = msg.data
        
    def main_loop(self):
        while not rospy.is_shutdown():
            print(self.neck_yaw_angle)
            if self.current_kashiwagi_state == "talking_game:speaking_turn":
                kashiwagi_utils.speaking_mode()  # 繰り返し実行される
            elif self.current_kashiwagi_state == "daily:introduction":
                kashiwagi_utils.speaking_mode()
            elif self.current_kashiwagi_state == "daily:singing":
                kashiwagi_utils.speaking_mode()
            elif self.current_kashiwagi_state == "talking_game:listening_turn":
                kashiwagi_utils.breath_mode_and_look_at_direction(self.neck_yaw_angle)
            elif self.current_kashiwagi_state == "talking_game:thinking_turn":
                kashiwagi_utils.thinking_mode()
            elif self.current_kashiwagi_state == "daily:normal":
                kashiwagi_utils.breath_mode_and_look_at_direction(self.neck_yaw_angle)
            elif self.current_kashiwagi_state == "daily:waking_up":
                kashiwagi_utils.init_pose()
            elif self.current_kashiwagi_state == "daily:happy":
                kashiwagi_utils.breath_mode_and_look_at_direction(self.neck_yaw_angle)
            elif self.current_kashiwagi_state == "daily:goodbye":
                kashiwagi_utils.goodbye()
            elif self.current_kashiwagi_state == "idle":
                kashiwagi_utils.breath_mode()
            elif self.current_kashiwagi_state == "move:staying":
                kashiwagi_utils.breath_mode_and_look_at_direction(self.neck_yaw_angle)
            elif self.current_kashiwagi_state == "move:surprised":
                kashiwagi_utils.init_pose()
            elif self.current_kashiwagi_state == "move:approaching_person":
                kashiwagi_utils.moving_mode()
            elif self.current_kashiwagi_state == "move:finding_person":
                kashiwagi_utils.look_around_mode()
            elif self.current_kashiwagi_state == "move:found_person":
                kashiwagi_utils.init_pose()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MotionManager()
    except rospy.ROSInterruptException:
        pass
