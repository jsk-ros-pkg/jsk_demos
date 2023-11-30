#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo_drive_simulator')
import rospy, math, time, threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from osrf_msgs.msg import JointCommands
from numpy import zeros, array, linspace, ceil

lock = threading.Lock()
currentJointState = None

def jointStatesCallback(msg):
  with lock:
    global currentJointState
    currentJointState = msg

def jointTrajectoryCommandCallback(msg):
  while rospy.get_rostime().to_sec() == 0.0:
    time.sleep(0.1)
    
  with lock:
    if currentJointState == None:
      print("/atlas/joint_states is not published")
      return

    atlasJointNames = [
      'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx',
      'atlas::l_arm_elx', 'atlas::l_arm_ely', 'atlas::l_arm_mwx', 'atlas::l_arm_shx', 'atlas::l_arm_usy', 'atlas::l_arm_uwy',
      'atlas::r_arm_elx', 'atlas::r_arm_ely', 'atlas::r_arm_mwx', 'atlas::r_arm_shx', 'atlas::r_arm_usy', 'atlas::r_arm_uwy', 
      'atlas::l_leg_kny', 'atlas::l_leg_lax', 'atlas::l_leg_lhy', 'atlas::l_leg_mhx', 'atlas::l_leg_uay', 'atlas::l_leg_uhz', 
      'atlas::r_leg_kny', 'atlas::r_leg_lax', 'atlas::r_leg_lhy', 'atlas::r_leg_mhx', 'atlas::r_leg_uay', 'atlas::r_leg_uhz', 
      'atlas::neck_ay']

    # initialize JointCommands message
    command = JointCommands()
    command.name = list(atlasJointNames)
    n = len(command.name)
    command.position     = zeros(n)
    command.velocity     = zeros(n)
    command.effort       = zeros(n)
    command.kp_position  = zeros(n)
    command.ki_position  = zeros(n)
    command.kd_position  = zeros(n)
    command.kp_velocity  = zeros(n)
    command.i_effort_min = zeros(n)
    command.i_effort_max = zeros(n)

    # now get gains from parameter server
    for i in xrange(len(command.name)):
      name = command.name[i]
      command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
      command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
      command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
      command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
      command.i_effort_min[i] = -command.i_effort_max[i]

    # set up the publisher
    pub = rospy.Publisher('/atlas/joint_commands', JointCommands, queue_size=1)

    # for each trajectory
    for i in xrange(0, len(atlasJointNames)):
      # get initial joint positions
      initialPosition = array(currentJointState.position)
      # first value is time duration
      dt = 10.0
      # subsequent values are desired joint positions
      commandPosition = array([ float(x) for x in currentJointState.position ])
      commandPosition[atlasJointNames.index('atlas::back_lbz')] = msg.data # overwrite command angle by subscribed value
      # desired publish interval
      dtPublish = 0.02
      n = ceil(dt / dtPublish)
      for ratio in linspace(0, 1, n):
        interpCommand = (1-ratio)*initialPosition + ratio * commandPosition
        command.position = [ float(x) for x in interpCommand ]
        pub.publish(command)

if __name__ == '__main__':
  try:
    # Initialize the node
    rospy.init_node('joint_control')

    # Setup subscriber to atlas states
    rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)
    rospy.Subscriber("/look_around_angle", Float64, jointTrajectoryCommandCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      rate.sleep()

  except rospy.ROSInterruptException: pass
