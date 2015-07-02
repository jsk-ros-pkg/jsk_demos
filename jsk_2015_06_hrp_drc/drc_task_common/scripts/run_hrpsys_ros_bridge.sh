#!/usr/bin/env bash

set -x
ssh leus@fc22 -t "bash -ci \"export DISPLAY=:0;xterm -geometry 80x100+0+0 -T \\\"Auditor\\\" -e \\\"source ~/ros/hydro/devel/setup.bash; rossetmaster hrp2017v;rossetip; bash -ci 'export DISPLAY=:0;`rospack find jsk_hrp2_ros_bridge`/scripts/hrp2-launch-ros-bridge.sh'\\\" &\""
