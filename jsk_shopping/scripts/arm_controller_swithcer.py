#!/usr/bin/env python

# mainly copied from fetch_robots fetch_bringup/scripts/controller_reset.py

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from robot_controllers_msgs.msg import QueryControllerStatesAction,\
                                        QueryControllerStatesGoal,\
                                        ControllerState

class ArmControllerReset:
    CONTROLLER_ACTION_NAME = "/query_controller_states"
    GRIPPER_ACTION_NAME = "/gripper_controller/gripper_action"

    def __init__(self):
        rospy.loginfo("Connecting to %s..." % self.CONTROLLER_ACTION_NAME)
        self.controller_client = actionlib.SimpleActionClient\
                                 (self.CONTROLLER_ACTION_NAME, QueryControllerStatesAction)
        self.controller_client.wait_for_server()
        rospy.loginfo("Done")

        rospy.loginfo("Connecting to %s..." % self.GRIPPER_ACTION_NAME)
        self.gripper_client = actionlib.SimpleActionClient\
                              (self.GRIPPER_ACTION_NAME, GripperCommandAction)

        self.gravity_compsensation = list()
        self.gravity_compsensation.append("arm_controller/gravity_compensation")

        self.follow_joint_trajectory = list()
        self.follow_joint_trajectory.append("arm_controller/follow_joint_trajectory")
        self.follow_joint_trajectory.append\
            ("arm_with_torso_controller/follow_joint_trajectory")
        self.follow_joint_trajectory.append("torso_controller/follow_joint_trajectory")
        self.follow_joint_trajectory.append("head_controller/follow_joint_trajectory")
        self.follow_joint_trajectory.append("head_controller/point_head")

    def servo_on(self):
        # Start controllers
        goal_servo_on = QueryControllerStatesGoal()

        for controller in self.gravity_compsensation:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal_servo_on.updates.append(state)

        for controller in self.follow_joint_trajectory:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal_servo_on.updates.append(state)

        self.controller_client.send_goal(goal_servo_on)

        # Able gripper torque
        # goal_servo_on = GripperCommandGoal()
        # goal_servo_on.command.max_effort = 60.0
        # self.gripper_client.send_goal(goal_servo_on)

    def servo_off(self):
        # Reset controllers
        goal_servo_off = QueryControllerStatesGoal()

        for controller in self.gravity_compsensation:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal_servo_off.updates.append(state)

        for controller in self.follow_joint_trajectory:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal_servo_off.updates.append(state)

        self.controller_client.send_goal(goal_servo_off)

        # Disable gripper torque
        # goal_servo_off = GripperCommandGoal()
        # goal_servo_off.command.max_effort = -1.0
        # self.gripper_client.send_goal(goal_servo_off)

# if __name__=="__main__":
#     rospy.init_node("change_controller_state")
#     c = ControllerStateChange()
