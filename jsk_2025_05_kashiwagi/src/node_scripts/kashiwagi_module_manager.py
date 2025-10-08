#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from kashiwagi_module_utils import Modules

class ModuleManager:
    def __init__(self):
        rospy.init_node("kashiwagi_module_manager")
        rospy.sleep(1.0)
        self.current_state = "unknown"
        self.prev_state = "unknown"
        self.modules = Modules()

        self.ume_led_color_map = {
            "idle": (60, 60, 255),
            "talking_game:listening_turn": (255, 255, 40),
            "talking_game:speaking_turn": (255, 135, 135),
            "daily:waking_up":  (255, 130, 255),
            "daily:normal":  (255, 130, 255),
            "daily:happy":  (255, 130, 255),
        }

        self.cheek_led_color_map = {
            "daily:waking_up":  (255, 130, 255),
            "daily:happy":  (255, 130, 255),
        }

        self.eye_map = {
            "idle": "sleepy",
            "talking_game:listening_turn": "blink",
            "talking_game:speaking_turn": "normal",
            "daily:waking_up": "surprised",
            "daily:normal": "normal",
            "daily:happy": "happy",
        }

        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        rospy.loginfo("Launching Module Manager node ....")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.current_state:
            rospy.loginfo(f"Kashiwagi State Changed: {self.current_state} → {new_state}")
            self.prev_state = self.current_state
            self.current_state = new_state

        # Ume LED control
        ume_color = self.ume_led_color_map.get(self.current_state)
        if ume_color:
            ume_r, ume_g, ume_b = ume_color
            self.modules.ume_led(ume_r, ume_g, ume_b)
        else:
            self.modules.ume_led(0, 0, 0, 0)

        # Cheek LED control
        cheek_color = self.cheek_led_color_map.get(self.current_state)
        if cheek_color:
            cheek_r, cheek_g, cheek_b = cheek_color
            self.modules.cheek_led(cheek_r, cheek_g, cheek_b)
        else:
            self.modules.cheek_led(0, 0, 0, 0)

        # Eye expression control
        eye_status = self.eye_map.get(self.current_state)
        if eye_status:
            self.modules.eye(eye_status)
        else:
            self.modules.eye("normal")

if __name__ == '__main__':
    try:
        ModuleManager()
    except rospy.ROSInterruptException:
        pass
