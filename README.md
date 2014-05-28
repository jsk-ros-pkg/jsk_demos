jsk_demos
=========

JSK demo programs

2013 irt demo
-------------

PR2/ROS re-written version of [2009 irt demo](https://www.youtube.com/watch?v=ToL3egTOahg)
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

![tray](https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/5247c78b283030af0ddc66d7c42ae911e5d06bd7/jsk_2013_04_pr2_irt_1.png =200x200)
![wash](https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/9584321f8b5069d056e145752c3ecc8a1026babf/jsk_2013_04_pr2_irt_2.png =200x200)
![chair](https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/2eb3ca13d1b7ac2019da5ca3778fcc28afa3a92f/jsk_2013_04_pr2_irt_3.png =200x200)
![bloom](https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/c14d6c52d8bf35fd5c244d989beccd35caa6fa8a/jsk_2013_04_pr2_irt_4.png =200x200)
