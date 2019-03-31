#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

import numpy as np
from matplotlib import pyplot as plt

import sys, signal
import math

class PitchYawPlotter:
    def __init__(self):
        self.now_pitch = 0
        self.now_yaw = 0

        self.initNode()
        self.initGraph()

    def initNode(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("calc_idle_diabolo_state/diabolo_state", Float64MultiArray, self.callbackForDiaboloState)

    def initGraph(self):
        self.t = np.zeros(20)
        self.pitch = np.zeros(20)
        self.yaw = np.zeros(20)
        self.zero = np.zeros(20)

        plt.ion()
        plt.figure()
    
        plt.subplot(2,1,1)
        plt.title("Pitch")
        self.li_pitch, = plt.plot(self.t, self.pitch)
        self.li_pitch_zero, = plt.plot(self.t, self.zero)
        plt.xlabel("time[s]")
        plt.ylabel("pitch[degree]")
    
        plt.subplot(2,1,2)
        plt.title("Yaw")
        self.li_yaw, = plt.plot(self.t, self.yaw)
        self.li_yaw_zero, = plt.plot(self.t, self.zero)
        plt.xlabel("time[s]")
        plt.ylabel("yaw[degree]")
    
        self.now_time = 0;

    def callbackForDiaboloState(self, data):
        self.now_pitch = data.data[0]
        self.now_yaw = data.data[1]
    
    def plot(self):
        while True:
            self.t = np.append(self.t, self.now_time) 
            self.t = np.delete(self.t, 0)
            self.pitch = np.append(self.pitch, self.now_pitch)
            self.pitch = np.delete(self.pitch, 0)
            self.yaw = np.append(self.yaw, self.now_yaw)
            self.yaw = np.delete(self.yaw, 0)
    
            self.li_pitch.set_xdata(self.t)
            self.li_pitch.set_ydata(self.pitch)           
            self.li_pitch_zero.set_xdata(self.t)
            self.li_pitch_zero.set_ydata(self.zero)
    
            self.li_yaw.set_xdata(self.t)
            self.li_yaw.set_ydata(self.yaw)           
            self.li_yaw_zero.set_xdata(self.t)
            self.li_yaw_zero.set_ydata(self.zero)
    
            plt.subplot(2,1,1)
            plt.xlim(min(self.t), max(self.t))
            pitch_min = min(min(self.zero), min(self.pitch))
            pitch_max = max(max(self.zero), max(self.pitch))
            #plt.ylim((pitch_max * (-1) + pitch_min * 6) / 5, (pitch_max * 6 + pitch_min * (-1)) / 5)
            plt.ylim(-80, 80)
    
            plt.subplot(2,1,2)
            plt.xlim(min(self.t), max(self.t))
            yaw_min = min(min(self.zero), min(self.yaw))
            yaw_max = max(max(self.zero), max(self.yaw))
            #plt.ylim((yaw_max * (-1) + yaw_min * 6) / 5, (yaw_max * 6 + yaw_min * (-1)) / 5)
            plt.ylim(-80, 80)
    
            plt.draw()
    
            self.now_time += 1
            plt.pause(0.01)
            
if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))
    
    plotter = PitchYawPlotter()
    plotter.plot()
