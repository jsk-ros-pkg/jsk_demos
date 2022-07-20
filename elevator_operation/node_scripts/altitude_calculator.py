#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
import message_filters


class AltitudeCalculator(object):

    def __init__(self):

        self.pub = rospy.Publisher('~output', Float32, queue_size=1)

        sub_pressure = message_filters.Subscriber('~input_pressure', Float32)
        sub_temparature = message_filters.Subscriber(
            '~input_temperature', Float32)

        # Pa
        self.p_b = rospy.get_param('~sea_level_pressure', 101325)
        # J K^-1 mol^-1
        self.gas_constant = rospy.get_param('~gas_const', 8.31446261815324)
        # m sec^-2
        self.grav_accel = rospy.get_param('~gravitational_accel', 9.8)

        slop = rospy.get_param('~slop', 0.1)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_pressure, sub_temparature],
            10,
            slop=slop,
            allow_headerless=True
        )
        self.ts.registerCallback(self.callback)

        rospy.loginfo('initialized')

    def callback(self, msg_pressure, msg_temp):

        p = msg_pressure.data
        T = msg_temp.data
        altitude = (math.pow(self.p_b / p, 1 / 5.257) - 1) * \
            (T + 273.15) / 0.0065
        msg = Float32()
        msg.data = altitude
        self.pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('altitude_calculator')
    node = AltitudeCalculator()
    rospy.spin()
