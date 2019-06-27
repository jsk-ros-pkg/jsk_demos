#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


ARGS=""
for OPT in "$@"; do
  case "$OPT" in
    __*)  # remove ROS arguments
      shift
      ;;
    *)
      ARGS="$ARGS $1"
      shift
      ;;
  esac
done

roslaunch $THIS_DIR/relay_rgbd.xml $ARGS --screen 1>&2
