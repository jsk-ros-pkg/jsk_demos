import threading
import os
# import readline
import json

import rospy
from enum import Enum


class CommandTypes(Enum):
    none = 0
    save = 1
    play = 2
    name = 3
    
    
current_state = CommandTypes.none.value
play_motion_name = None

PROMPT = "{talker:<6}: "


def command():
    global saving
    global playing
    global current_state
    global play_motion_name
    print(rospy.is_shutdown())
    while not rospy.is_shutdown():
        try:
            if current_state == CommandTypes.save.value:
                text = input(PROMPT.format(talker="saving angles you"))
            else:
                text = input(PROMPT.format(talker="you"))
        except EOFError:
            text = None
        if not text:
            print("End saving")
            current_state = CommandTypes.none.value
            break
        if text == "save":
            current_state = CommandTypes.save.value
        elif text == "play":
            current_state = CommandTypes.play.value
        elif text == "end":
            current_state = CommandTypes.play.value
    print("command receive end")


def save_angle_vector_mode(ri, json_filepath=None):
    global saving
    global playing
    global current_state
    motion_dict = {}
    if json_filepath is not None:
        json_filepath = os.path.expanduser(json_filepath)
        if os.path.exists(json_filepath):
            try:
                with open(json_filepath) as f:
                    motion_dict = json.load(f)
            except json.JSONDecodeError as e:
                print(e)
            if len(motion_dict) > 0:
                print("current motions")
                print(list(motion_dict))
    saving = True    
    ri.servo_off()
    thread = threading.Thread(target=command)
    thread.daemon = True  # terminate when main thread exit    
    thread.start()
    angles = []

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state == CommandTypes.save.value:
            angles.append(ri.angle_vector())
        elif current_state == CommandTypes.play.value:
            break
        rate.sleep()
    ri.servo_on()

    print("playing motions")
    ###
    print(len(angles))
    cnt = 0
    ####
    if len(angles) > 0:
        ri.angle_vector(angles[0], 3)
        ri.wait_interpolation()
    for av in angles[1:]:
        ri.angle_vector(av, 0.1)
        # ri.wait_interpolation()
        rate.sleep()
        cnt += 1
        print(cnt)
    thread.join()

    print("thread.join end")
    if len(angles) > 0:
        while True:
            name = input("please input motion name: ")
            if len(name) == 0:
                continue
            break
        new_angles = []
        for av in angles:
            new_angles.append([float(angle) for angle in av])
        motion_dict[name] = new_angles
        print("saved {} motion".format(name))
        if json_filepath is not None:
            with open(json_filepath, "w") as f:
                f.write(
                    json.dumps(motion_dict,
                               indent=4, separators=(",", ": ")))
                
