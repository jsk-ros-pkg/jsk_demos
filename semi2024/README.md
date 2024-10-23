# semi 2024

## Prerequisite

```
sudo apt-get install -y libspatialindex-dev freeglut3-dev libsuitesparse-dev libblas-dev liblapack-dev ros-noetic-position-controllers ros-noetic-joint-trajectory-controller ros-noetic-controller-manager ros-noetic-diff-drive-controller ros-noetic-teleop-twist-keyboard python3-vcstool
mkdir -p ~/ros/semi2024/src
cd ~/ros/semi2024/src
wget https://raw.githubusercontent.com/iory/jsk_demos/semi2024/vcsinstall.noetic -O- | vcs import
cd ~/ros/semi2024
rosdep install --from-paths -i -y -r .
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros/semi2024/devel/setup.bash
```
