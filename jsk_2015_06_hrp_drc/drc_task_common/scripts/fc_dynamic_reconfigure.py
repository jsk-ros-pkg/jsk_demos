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
    elif type == "bool":
        param = BoolParameter()
        param.name = name
        param.value = value
        config.bools = [param]
        srv(config)
    elif type == "int":
        param = IntParameter()
        param.name = name
        param.value = value
        config.ints = [param]
        srv(config)
    else:
        raise Exception("Unsupported type: %s" % (type))

def callback(msg):
    slots = list(msg.__slots__)           #copy
    multisense_srv = dynamicReconfigureService("/multisense")
    chest_srv = dynamicReconfigureService("/chest_camera")
    for slot in slots:
        
        if slot == "lighting":
            setDynamicReconfigure(multisense_srv, "led_duty_cycle", "double",
                                  getattr(msg, slot))
        elif slot == "exposure":
            setDynamicReconfigure(multisense_srv, "exposure_time", "double", 
                                  getattr(msg, slot))
        elif slot == "enable_auto_exposure":
            setDynamicReconfigure(multisense_srv, "auto_exposure", "bool", 
                                  getattr(msg, slot))
        elif slot == "enable_lighting":
            setDynamicReconfigure(multisense_srv, "lighting", "bool", 
                                  getattr(msg, slot))
        elif slot == "chest_camera_enable_auto_exposure":
            setDynamicReconfigure(chest_srv, "auto_exposure", "bool", 
                                  getattr(msg, slot))
        elif slot == "chest_camera_exposure":
            setDynamicReconfigure(chest_srv, "exposure", "double", 
                                  getattr(msg, slot))
        else:
            raise Exception("Unsupported slot: %s" % (slot))

if __name__ == "__main__":
    rospy.init_node("fc_dynamic_reconfigure")
    sub = rospy.Subscriber("fc_from_ocs_reconfigure/output",
                           DRCParametersMessage, callback)
    rospy.spin()
