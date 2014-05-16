jsk_demos
=========

JSK demo programs

example 1
---------
```
sudo apt-get install ros-hydro-roseus-msgs
sudo apt-get install ros-hydro-pr2eus
source /opt/ros/hydro/setup.bash
mkdir -p path/to/install  ;; use arbitrary directory name
cd path/to/install
rosws init
rosws merge /opt/ros/hydro
rosws set jsk_demos http://github.com/jsk-ros-pkg/jsk_demos --git
source setup.bash
rosrun jsk_2013_04_pr2_610 demo.l "(demo)" ;; please ignore all error message
```

