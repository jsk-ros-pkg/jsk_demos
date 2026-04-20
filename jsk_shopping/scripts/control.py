import rospy
from arm_controller_switcher import ArmControllerReset

rospy.init_node('hoge')
c = ArmControllerReset()
c.servo_on()
