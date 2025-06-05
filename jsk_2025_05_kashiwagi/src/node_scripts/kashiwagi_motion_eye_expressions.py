def init_pose():
    av = [0] * 10
    ri.angle_vector(av, 0.2)
    ri.wait_interpolation()

def breath_mode():
    larm_roll_angles = [-1.0, -1.0, -1.0, -0.9, -0.8, -0.7, -0.7, -0.7, -0.8, -0.9]
    rarm_roll_angles = [1.0, 1.0, 1.0, 0.9, 0.8, 0.7, 0.7, 0.7, 0.8, 0.9]
    neck_pitch_angles = [-0.04, -0.04, -0.04, -0.08, -0.12, -0.15, -0.15, -0.15, -0.12, -0.08]
    
    for i in range(10):
        av = [0, neck_pitch_angles[i], 0, larm_roll_angles[i], 0, 0, 0, rarm_roll_angles[i], 0, 0]
        ri.angle_vector(av, 0.2)
        ri.wait_interpolation()

def look_downside_mode():
    print("look_downside_mode")

def thinking_mode():
    print("thinking_mode")

    
    
