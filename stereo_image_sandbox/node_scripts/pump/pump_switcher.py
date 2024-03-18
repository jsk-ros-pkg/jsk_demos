import board
import digitalio

import rospy
from std_msgs.msg import Empty


class PumpSwitcher(object):
    def __init__(self):
        rospy.loginfo("Start PumpSwitcher. Default is pump on.")
        # For pinout, see https://wiki.radxa.com/Zero/hardware/gpio
        # D19: GPIOH_4
        self.mosfet = digitalio.DigitalInOut(board.D19)
        self.mosfet.direction = digitalio.Direction.OUTPUT
        self.mosfet.value = True  # default: D19 is high
        # ROS subscribers
        rospy.sleep(1.0)
        rospy.Subscriber('pump_on', Empty, self.on_cb)
        rospy.Subscriber('pump_off', Empty, self.off_cb)

    def on_cb(self, msg):
        rospy.loginfo("Pump on")
        self.mosfet.value = True

    def off_cb(self, msg):
        rospy.loginfo("Pump off")
        self.mosfet.value = False

if __name__ == '__main__':
    rospy.init_node('pump_switcher')
    PumpSwitcher()
    rospy.spin()
