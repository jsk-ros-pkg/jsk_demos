#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rospy.loginfo(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    rospy.logerr(f"Failed to open serial port: {e}")
    exit(1)


def send_angles_to_arduino(angles):
    try:
        data = bytes([0x01] + angles)
        ser.write(data)
        rospy.loginfo(f"Sent to Arduino: {data}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to send data to Arduino: {e}")


def joint_state_callback(msg):
    rospy.loginfo("JointState message received")
    rospy.logdebug(f"Positions: {msg.position}")
    if len(msg.position) >= 3:
        angle1 = min(255, max(0, int(msg.position[0])))
        angle2 = min(255, max(0, int(msg.position[1])))
        angle3 = min(255, max(0, int(msg.position[2])))
        rospy.loginfo(f"Angles: {angle1}, {angle2}, {angle3}")
        send_angles_to_arduino([angle1, angle2, angle3])
    else:
        rospy.logwarn("Insufficient data in JointState position array.")


def main():
    rospy.init_node('joint_state_to_serial', anonymous=True)
    rospy.Subscriber("eye_brow/joint_states", JointState, joint_state_callback)
    rospy.loginfo("Started JointState to Serial bridge")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser.is_open:
            ser.close()
