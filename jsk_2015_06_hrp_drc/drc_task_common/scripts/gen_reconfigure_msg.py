#!/usr/bin/env python

# Generate msg/DRCParametersMessage.msg 
# from drc_com_common/cfg/DRCParameters

import rospy
import sys
from drc_com_common.cfg import DRCParametersConfig

output_file = sys.argv[1]

def msgType(tp):
  if tp == "double" or tp == "float":
    return "float32"
  elif tp == "int":
    return "int32"
  elif tp == "str" or tp == "string":
    raise Exception("String is not supported")
  elif tp == "bool":
    return "bool"
  else:
    raise Exception("Unknown type: %s" % (tp))

with open(output_file, "w") as f:
  for (name, type) in DRCParametersConfig.type.items():
    f.write("%s %s\n" % (msgType(type), name))

