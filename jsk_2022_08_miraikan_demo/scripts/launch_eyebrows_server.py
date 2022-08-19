#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib.packages
import subprocess
import os

try:
    dir_path = roslib.packages.get_pkg_dir('miraikan_demo') + "/../web_nodejs"
    os.chdir(dir_path)
    subprocess.call(["docker-compose", "up"])
except:
    print("Error: Cannot launch eyebrows server")
