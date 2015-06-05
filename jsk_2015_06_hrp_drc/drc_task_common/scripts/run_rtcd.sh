#!/usr/bin/env bash

set -x
ssh leus@fc22 -t "bash -ci \"export DISPLAY=:0;xterm -T \\\"RTCD\\\" -e 'source ~/ros/hydro/devel/setup.bash;`rospack find jsk_hrp2_ros_bridge`/scripts/rtcd_start_command.sh --remote'&\""
