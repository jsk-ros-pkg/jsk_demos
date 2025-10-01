#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from kashiwagi_module_utils import Modules

class LedManager:
    def __init__(self):
        rospy.init_node("kashiwagi_ume_led_manager")
        rospy.sleep(1.0)
        self.current_state = "unknown"
        #self.prev_state = "unknown"
        self.modules = Modules()

        self.led_color_map = {
            "idle": (40, 255, 40),
            "talking_game:listening_turn": (255, 255, 40),
            "talking_game:speaking_turn": (255, 135, 135),
        }

        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        rospy.loginfo("Launching LED Manager node ....")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data
        #if new_state == self.current_state:
        #    return

        #rospy.loginfo(f"LED state changed: {self.current_state} → {new_state}")
        #self.prev_state = self.current_state
        self.current_state = new_state

        color = self.led_color_map.get(self.current_state)
        if color:
            r, g, b = color
            self.modules.ume_led(r, g, b)

if __name__ == '__main__':
    try:
        LedManager()
    except rospy.ROSInterruptException:
        pass
