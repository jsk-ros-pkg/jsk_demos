#!/usr/bin/env bash

ssh leus@fc22 "source ~/ros/hydro/devel/setup.bash;rosrun drc_task_common field_computer.sh -o 10.20.2.31&"
