#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
import scipy.fftpack
import numpy as np

## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import numpy
from geometry_msgs.msg import WrenchStamped
from drc_task_common.msg import Float32ArrayStamped
stored_force_x = []
stored_force_y = []
stored_force_z = []
N = 64

def wrench_cb(wrench_msg):
    global stored_data_x
    stored_force_x.append(wrench_msg.wrench.force.x)
    if (len(stored_force_x) > N):
        stored_force_x.pop(0)
        F_X = np.fft.fft(stored_force_x)
        amplitudeSpectrum = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in F_X]
        phaseSpectrum = [np.arctan2(int(c.imag), int(c.real)) for c in F_X]
        fft_amplitude_force_x_pub.publish(Float32ArrayStamped(data=amplitudeSpectrum, header=wrench_msg.header))
        fft_phase_force_x_pub.publish(Float32ArrayStamped(data=phaseSpectrum, header=wrench_msg.header))
    global stored_data_y
    stored_force_y.append(wrench_msg.wrench.force.y)
    if (len(stored_force_y) > N):
        stored_force_y.pop(0)
        F_Y = np.fft.fft(stored_force_y)
        amplitudeSpectrum = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in F_Y]
        phaseSpectrum = [np.arctan2(int(c.imag), int(c.real)) for c in F_Y]
        float_array = Float32ArrayStamped(data=amplitudeSpectrum)
        float_array.header = wrench_msg.header
        fft_amplitude_force_y_pub.publish(Float32ArrayStamped(data=amplitudeSpectrum, header=wrench_msg.header))
        fft_phase_force_y_pub.publish(Float32ArrayStamped(data=phaseSpectrum, header=wrench_msg.header))
    global stored_data_z
    stored_force_z.append(wrench_msg.wrench.force.z)
    if (len(stored_force_z) > N):
        stored_force_z.pop(0)
        F_Z = np.fft.fft(stored_force_z)
        amplitudeSpectrum = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in F_Z]
        phaseSpectrum = [np.arctan2(int(c.imag), int(c.real)) for c in F_Z]
        float_array = Float32ArrayStamped(data=amplitudeSpectrum)
        float_array.header = wrench_msg.header
        fft_amplitude_force_z_pub.publish(Float32ArrayStamped(data=amplitudeSpectrum, header=wrench_msg.header))
        fft_phase_force_z_pub.publish(Float32ArrayStamped(data=phaseSpectrum, header=wrench_msg.header))

if __name__ == "__main__":
    rospy.init_node('wrench_fft', anonymous=True)
    wrench_name = rospy.resolve_name("wrench")
    fft_amplitude_force_x_pub = rospy.Publisher(wrench_name+"/fft/amplitude/force/x", Float32ArrayStamped)
    fft_amplitude_force_y_pub = rospy.Publisher(wrench_name+"/fft/amplitude/force/y", Float32ArrayStamped)
    fft_amplitude_force_z_pub = rospy.Publisher(wrench_name+"/fft/amplitude/force/z", Float32ArrayStamped)
    fft_phase_force_x_pub = rospy.Publisher(wrench_name+"/fft/phase/force/x", Float32ArrayStamped)
    fft_phase_force_y_pub = rospy.Publisher(wrench_name+"/fft/phase/force/y", Float32ArrayStamped)
    fft_phase_force_z_pub = rospy.Publisher(wrench_name+"/fft/phase/force/z", Float32ArrayStamped)
    rospy.Subscriber(wrench_name, WrenchStamped,  wrench_cb)
    rospy.spin()
