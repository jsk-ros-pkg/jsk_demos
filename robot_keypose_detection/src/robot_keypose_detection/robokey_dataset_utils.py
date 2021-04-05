from enum import IntEnum


class PR2JointType(IntEnum):

    torso_lift_joint = 0
    head_pan_joint = 1
    r_shoulder_pan_joint = 2
    r_shoulder_lift_joint = 3
    r_forearm_roll_joint = 4
    r_wrist_flex_joint = 5
    l_shoulder_pan_joint = 6
    l_shoulder_lift_joint = 7
    l_forearm_roll_joint = 8
    l_wrist_flex_joint = 9


pr2_joint_pairs = (
    (PR2JointType.torso_lift_joint, PR2JointType.head_pan_joint),
    (PR2JointType.torso_lift_joint, PR2JointType.r_shoulder_pan_joint),
    (PR2JointType.r_shoulder_pan_joint, PR2JointType.r_shoulder_lift_joint),
    (PR2JointType.r_shoulder_lift_joint, PR2JointType.r_forearm_roll_joint),
    (PR2JointType.r_forearm_roll_joint, PR2JointType.r_wrist_flex_joint),
    (PR2JointType.torso_lift_joint, PR2JointType.l_shoulder_pan_joint),
    (PR2JointType.l_shoulder_pan_joint, PR2JointType.l_shoulder_lift_joint),
    (PR2JointType.l_shoulder_lift_joint, PR2JointType.l_forearm_roll_joint),
    (PR2JointType.l_forearm_roll_joint,  PR2JointType.l_wrist_flex_joint))


pr2_joint_names = (
    'torso_lift_joint',
    'head_pan_joint',
    'r_shoulder_pan_joint',
    'r_shoulder_lift_joint',
    'r_forearm_roll_joint',
    'r_wrist_flex_joint',
    'l_shoulder_pan_joint',
    'l_shoulder_lift_joint',
    'l_forearm_roll_joint',
    'l_wrist_flex_joint')


class PR2RichJointType(IntEnum):

    torso_lift_joint = 0
    head_pan_joint = 1
    r_shoulder_pan_joint = 2
    r_shoulder_lift_joint = 3
    r_forearm_roll_joint = 4
    r_wrist_flex_joint = 5
    l_shoulder_pan_joint = 6
    l_shoulder_lift_joint = 7
    l_forearm_roll_joint = 8
    l_wrist_flex_joint = 9
    r_gripper_l_finger_tip_joint = 10
    r_gripper_r_finger_tip_joint = 11
    l_gripper_l_finger_tip_joint = 12
    l_gripper_r_finger_tip_joint = 13
    laser_tilt_mount_joint = 14
    wide_stereo_l_stereo_camera_frame_joint = 15
    wide_stereo_r_stereo_camera_frame_joint = 16
    base_laser_joint = 17


pr2_rich_joint_pairs = (
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.head_pan_joint),
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.r_shoulder_pan_joint),
    (PR2RichJointType.r_shoulder_pan_joint, PR2RichJointType.r_shoulder_lift_joint),
    (PR2RichJointType.r_shoulder_lift_joint, PR2RichJointType.r_forearm_roll_joint),
    (PR2RichJointType.r_forearm_roll_joint, PR2RichJointType.r_wrist_flex_joint),
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.l_shoulder_pan_joint),
    (PR2RichJointType.l_shoulder_pan_joint, PR2RichJointType.l_shoulder_lift_joint),
    (PR2RichJointType.l_shoulder_lift_joint, PR2RichJointType.l_forearm_roll_joint),
    (PR2RichJointType.l_forearm_roll_joint,  PR2RichJointType.l_wrist_flex_joint),
    (PR2RichJointType.l_wrist_flex_joint, PR2RichJointType.l_gripper_l_finger_tip_joint),
    (PR2RichJointType.l_wrist_flex_joint, PR2RichJointType.l_gripper_r_finger_tip_joint),
    (PR2RichJointType.r_wrist_flex_joint, PR2RichJointType.r_gripper_l_finger_tip_joint),
    (PR2RichJointType.r_wrist_flex_joint, PR2RichJointType.r_gripper_r_finger_tip_joint),
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.laser_tilt_mount_joint),
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.laser_tilt_mount_joint),
    (PR2RichJointType.head_pan_joint, PR2RichJointType.wide_stereo_l_stereo_camera_frame_joint),
    (PR2RichJointType.head_pan_joint, PR2RichJointType.wide_stereo_r_stereo_camera_frame_joint),
    (PR2RichJointType.torso_lift_joint, PR2RichJointType.base_laser_joint),
)


pr2_rich_joint_names = (
    'torso_lift_joint',
    'head_pan_joint',
    'r_shoulder_pan_joint',
    'r_shoulder_lift_joint',
    'r_forearm_roll_joint',
    'r_wrist_flex_joint',
    'l_shoulder_pan_joint',
    'l_shoulder_lift_joint',
    'l_forearm_roll_joint',
    'l_wrist_flex_joint',
    'r_gripper_l_finger_tip_joint',
    'r_gripper_r_finger_tip_joint',
    'l_gripper_l_finger_tip_joint',
    'l_gripper_r_finger_tip_joint',
    'laser_tilt_mount_joint',
    'wide_stereo_l_stereo_camera_frame_joint',
    'wide_stereo_r_stereo_camera_frame_joint',
    'base_laser_joint',
)
