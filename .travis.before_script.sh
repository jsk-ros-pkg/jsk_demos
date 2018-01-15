#!/bin/bash

install_robot_rosinstall() {
  if [ "$ROBOT" = "pr2" ]; then
    ROSINSTALL_URL="https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_pr2_robot/jsk_pr2_startup/jsk_pr2.rosinstall"
  elif [ "$ROBOT" = "fetch" ]; then
    ROSINSTALL_URL="https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch.rosinstall"
  fi

  if [ "$ROSINSTALL_URL" != "" ]; then
    wget "$ROSINSTALL_URL" -O - | wstool merge -t ~/ros/ws_$REPOSITORY_NAME/src -
  else
    wstool merge -t ~/ros/ws_$REPOSITORY_NAME/src $CI_SOURCE_PATH/.travis.rosinstall.default
  fi
}

install_robot_rosinstall
