#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from riberry.i2c_base import I2C


I2C_SLAVE_ADDRESS = 0x62
i2c = I2C(device=I2C_SLAVE_ADDRESS, bus=3)


def send_angles_to_arduino(angles):
    try:
        i2c.write([0x01] + angles)
    except IOError as e:
        rospy.logerr(f"Failed to send data to Arduino: {e}")


def joint_state_callback(msg):
    rospy.loginfo("JointState message received")
    print(msg.position)
    if len(msg.position) >= 3:
        angle1 = min(255, max(0, int(msg.position[0])))
        angle2 = min(255, max(0, int(msg.position[1])))
        angle3 = min(255, max(0, int(msg.position[2])))
        rospy.loginfo(f"Angles: {angle1}, {angle2}, {angle3}")
        send_angles_to_arduino([angle1, angle2, angle3])
    else:
        rospy.logwarn("Insufficient data in JointState position array.")


def main():
    rospy.init_node('joint_state_to_i2c', anonymous=True)
    rospy.Subscriber("eye_brow/joint_states", JointState, joint_state_callback)
    rospy.loginfo("Started JointState to I2C bridge")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
