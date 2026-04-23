

import control_msgs.msg
from kxr_controller.kxr_interface import KXRROSRobotInterface


class PoohROSRobotInterface(KXRROSRobotInterface):

    @property
    def fullbody_controller(self):
        cont_name = "fullbody_controller"
        return {
            "controller_type": cont_name,
            "controller_action": cont_name + "/follow_joint_trajectory",
            "controller_state": cont_name + "/state",
            "action_type": control_msgs.msg.FollowJointTrajectoryAction,
            "joint_names": self.joint_names,
        }

    @property
    def larm_controller(self):
        return dict(
            controller_type='larm_controller',
            controller_action='larm_controller/follow_joint_trajectory',
            controller_state='larm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=['larm_shoulder_p',
                         'larm_shoulder_r',
                         'larm_elbow'])

    @property
    def rarm_controller(self):
        return dict(
            controller_type='rarm_controller',
            controller_action='rarm_controller/follow_joint_trajectory',
            controller_state='rarm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=['rarm_shoulder_p',
                         'rarm_shoulder_r',
                         'rarm_elbow'])

    @property
    def head_controller(self):
        return dict(
            controller_type='head_controller',
            controller_action='head_controller/follow_joint_trajectory',
            controller_state='head_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=['head_neck_y', 'head_neck_p', 'head_neck_r'])

    def default_controller(self):
        """Overriding default_controller.

        Returns
        -------
        List of limb controller : list
        """
        return [
            self.larm_controller,
            self.rarm_controller,
            self.head_controller,
        ]
