#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16


class VacuumPressure(object):
    def __init__(self):
        # To read Radxa's ADC, see https://forum.radxa.com/t/how-use-uart-ee-uart-ao/10112/2
        self.adc1_path = '/sys/bus/platform/drivers/meson-saradc/ff809000.adc/iio:device0/in_voltage1_mean_raw'
        # ROS publisher
        rospy.sleep(1.0)
        self.pub = rospy.Publisher('vacuum_pressure', Int16, queue_size=10)

    def read_pressure(self):
        with open(self.adc1_path) as f:
            return f.read()

    def publish_pressure(self):
        pressure = int(self.read_pressure())
        pressure_msg = Int16(data=pressure)
        self.pub.publish(pressure_msg)


if __name__ == '__main__':
    rospy.init_node('vacuum_pressure')
    rospy.loginfo('Start publishing vacuum pressure data')
    vp = VacuumPressure()
    r = rospy.Rate(3)  # 3hz
    while not rospy.is_shutdown():
        vp.publish_pressure()
        r.sleep()
