#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from drc_com_common.cfg import DRCParametersConfig
from drc_task_common.msg import DRCParametersMessage


def reconfigureCallback(config, level):
    global pub
    msg = DRCParametersMessage()
    for (name) in DRCParametersConfig.type.keys():
      setattr(msg, name, getattr(config, name))  
    pub.publish(msg)
    return config


if __name__ == "__main__":
    rospy.init_node("ocs_dynamic_reconfigure")
    pub = rospy.Publisher("ocs_to_fc_reconfigure/input", DRCParametersMessage)
    srv = Server(DRCParametersConfig, reconfigureCallback)
    rospy.spin()
