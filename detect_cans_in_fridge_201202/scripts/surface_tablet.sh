#!/bin/bash

export PS1=dummy
source ~/.bashrc

rossetrobot c1
rossetip

rosrun rviz rviz -d $(rospack find detect_cans_in_fridge_201202)/config/surface_tablet.rviz
