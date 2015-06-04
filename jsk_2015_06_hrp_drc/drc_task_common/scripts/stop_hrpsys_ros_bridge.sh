#!/usr/bin/env bash

ssh leus@fc22 -t "bash -ic \"ssh hrpuser@hrp2017v -t 'bash -ic \\\"pgrep -f roslaunch.*hrp$(expr $HRP2NO + 2000).launch | xargs -n 1 kill\\\"'; pgrep -f roslaunch.*hrp$(expr $HRP2NO + 2000)_auditor.launch | xargs -n 1 kill\""
