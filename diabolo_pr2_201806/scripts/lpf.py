# average LPF
# average_num=10 is good!

import numpy as np
from matplotlib import pyplot as plt
import sys

if len(sys.argv) == 2:
    log_file = sys.argv[-1]
else:
    log_file = "../log/stable_pitch_plus.log"
    
pitch = []
yaw = []    
with open(log_file, 'r') as f:
    for l in f.readlines():
        w = l.split(' ')
        pitch.append(float(w[2]))
        yaw.append(float(w[3]))

lpf_average_nums = [1, 3, 5, 10]
lpf_pitch = [[] for i in range(len(lpf_average_nums))]
lpf_yaw = [[] for i in range(len(lpf_average_nums))]
for i in range(len(lpf_average_nums)):
    for j in range(len(pitch) - lpf_average_nums[i]):
        p = 0
        y = 0
        for k in range(lpf_average_nums[i]):
            p += pitch[j + k]
            y += yaw[j + k]
        lpf_pitch[i].append(p / lpf_average_nums[i])
        lpf_yaw[i].append(y / lpf_average_nums[i])

plt.figure()

plt.subplot(2,1,1)
plt.title("Pitch")
plt.plot(np.array([i for i in range(len(lpf_pitch[0]))]), np.array(lpf_pitch[0]))
plt.plot(np.array([i for i in range(len(lpf_pitch[1]))]), np.array(lpf_pitch[1]))
plt.plot(np.array([i for i in range(len(lpf_pitch[2]))]), np.array(lpf_pitch[2]))

plt.subplot(2,1,2)
plt.title("Yaw")
plt.plot(np.array([i for i in range(len(lpf_yaw[0]))]), np.array(lpf_yaw[0]))
plt.plot(np.array([i for i in range(len(lpf_yaw[1]))]), np.array(lpf_yaw[1]))
plt.plot(np.array([i for i in range(len(lpf_yaw[2]))]), np.array(lpf_yaw[2]))

plt.show()
