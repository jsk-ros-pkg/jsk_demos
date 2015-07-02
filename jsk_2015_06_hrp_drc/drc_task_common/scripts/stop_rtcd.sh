#!/usr/bin/env bash

set -x
ssh leus@fc22 -t "bash -ci \"export DISPLAY=:0;xterm -e \\\"ssh grxuser@hrp$(expr $HRP2NO + 2000)c 'sudo pkill rtcd'\\\"&\""
