#!/usr/bin/env python

import rospy

from drc_task_common.cfg import DRCParametersConfig

for (name, type) in DRCParametersConfig.type.items():
  print(name, type)
