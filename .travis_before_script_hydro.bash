#!/usr/bin/env bash

sudo -H pip install -q rosinstall_generator

rosinstall_generator --tar --rosdistro indigo \
  jsk_recognition_msgs \
>> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up -j1
