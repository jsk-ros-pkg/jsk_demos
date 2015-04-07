#!/usr/bin/env python

import rospy

from drc_task_common.msg import DRCParametersMessage
from dynamic_reconfigure.msg import *
from dynamic_reconfigure.srv import *

def dynamicReconfigureService(service_name):
    return rospy.ServiceProxy(service_name + "/set_parameters", Reconfigure)

def setDynamicReconfigure(srv, name, type, value):
    config = Config()
    if type == "double":
        param = DoubleParameter()
        param.name = name
        param.value = value
        config.doubles = [param]
        srv(config)
    else:
        raise Exception("Unsupported type: %s" % (type))

def callback(msg):
    slots = list(msg.__slots__)           #copy
    for slot in slots:
        if slot == "lighting":
            multisense_srv = dynamicReconfigureService("/multisense")
            setDynamicReconfigure(multisense_srv, "led_duty_cycle", "double",
                                  getattr(msg, slot))
        elif slot == "exposure":
            setDynamicReconfigure(multisense_srv, "exposure_time", "double", 
                                  getattr(msg, slot))
        else:
            raise Exception("Unsupported slot: %s" % (slot))

if __name__ == "__main__":
    rospy.init_node("fc_dynamic_reconfigure")
    sub = rospy.Subscriber("fc_from_ocs_reconfigure/output",
                           DRCParametersMessage, callback)
    rospy.spin()
