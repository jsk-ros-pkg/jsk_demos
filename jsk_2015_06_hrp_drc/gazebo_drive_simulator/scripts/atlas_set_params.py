#!/usr/bin/env python
import rospy, yaml, sys

atlasJointNames = [
    'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
    'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
    'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
    'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
    'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']


def setAtlasJointGain():
    # now get gains from parameter server
    rospy.init_node('tutorial_atlas_control')
    for name in atlasJointNames:
        rospy.set_param('atlas_controller/gains/' + name[7::] + '/p', 100000.0)
        rospy.set_param('atlas_controller/gains/' + name[7::] + '/i', 0.0)
        rospy.set_param('atlas_controller/gains/' + name[7::] + '/d', 1000.0)

if __name__ == "__main__":
    setAtlasJointGain()
