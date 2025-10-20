import os
import sys
import subprocess

pkg_path = subprocess.check_output(['rospack', 'find', 'jsk_2023_12_codesign']).decode().strip()
sys.path.append(os.path.join(pkg_path, 'scripts'))

from robotmodel import *

def init_pose():
    av = [0] * 10
    ri.angle_vector(av, 0.2)
    ri.wait_interpolation()

def breath_mode():
    larm_roll_angles = [-1.0, -1.0, -1.0, -1.0, -1.0, -0.9, -0.8, -0.7, -0.7, -0.7, -0.8, -0.9]
    rarm_roll_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 0.9, 0.8, 0.7, 0.7, 0.7, 0.8, 0.9]
    neck_pitch_angles = [-0.04, -0.04, -0.04, -0.04, -0.04, -0.08, -0.12, -0.15, -0.15, -0.15, -0.12, -0.08]
    
    for i in range(12):
        av = [ri.angle_vector()[0], neck_pitch_angles[i], 0, larm_roll_angles[i], 0, 0, 0, rarm_roll_angles[i], 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

def look_downside_mode():
    print("look_downside_mode")

def thinking_mode():
    neck_pitch_angles = [0, 0, 0, -0.1, -0.15, -0.2, -0.3, -0.3, 0.3, -0.2, -0.15, -0.1]
    neck_yaw_angles = [-0.1, -0.05, 0, 0, 0.05, 0.1, 0.1, 0.05, 0, 0, -0.05, -0.1]
    larm_pitch_angles = [0.7, 0.7, 0.7, 0.8, 0.9, 1.0, 1.2, 1.2, 1.2, 1.0, 0.9, 0.8]
    rarm_pitch_angles = [-0.7, -0.7, -0.7, -0.8, -0.9, -1.0, -1.2, -1.2, -1.2, -1.0, -0.9, -0.8]

    for i in range(12):
        av = [neck_pitch_angles[i], neck_yaw_angles[i], larm_pitch_angles[i], -1.2, 0, 0, rarm_pitch_angles[i], 1.2, 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

def look_at_direction(neck_yaw_angle):
    av = ri.angle_vector()
    av[0] = neck_yaw_angle
    ri.angle_vector(av, 0.4)
    ri.wait_interpolation()

def breath_mode_and_look_at_direction(neck_yaw_angle):
    larm_roll_angles = [-1.0, -1.0, -1.0, -1.0, -1.0, -0.9, -0.8, -0.7, -0.7, -0.7, -0.8, -0.9]
    rarm_roll_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 0.9, 0.8, 0.7, 0.7, 0.7, 0.8, 0.9]
    neck_pitch_angles = [-0.04, -0.04, -0.04, -0.04, -0.04, -0.08, -0.12, -0.15, -0.15, -0.15, -0.12, -0.08]
    
    for i in range(12):
        av = [neck_yaw_angle, neck_pitch_angles[i], 0, larm_roll_angles[i], 0, 0, 0, rarm_roll_angles[i], 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

def goodbye():
    rarm_roll_angles = [-1.0, -0.3, 0.4, 0]
    for i in range(len(rarm_roll_angles)):
        av = [0, 0.04, 0, -0.7, 0, 0, 0, rarm_roll_angles[i], 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

def speaking_mode():
    larm_roll_angles = [-0.8, -0.4]
    rarm_roll_angles = [0.8, 0.4]
    neck_pitch_angles = [0, -0.3]

    for i in range(2):
        av = [0, neck_pitch_angles[i], 0, larm_roll_angles[i], 0, 0, 0, rarm_roll_angles[i], 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

    
    
    
