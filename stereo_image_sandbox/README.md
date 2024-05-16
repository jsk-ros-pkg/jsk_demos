# stereo image sandbox

```
mkdir -p ~/ros/stereo_ws/src
wget https://raw.githubusercontent.com/iory/jsk_demos/stereo/stereo_image_sandbox/rosinstall -O ~/ros/stereo_ws/src/.rosinstall
cd ~/ros/stereo_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros/stereo_ws/src
wstool up
cd ~/ros/stereo_ws
rosdep update
rosdep install --from-paths -i -y -r .
catkin b stereo_image_sandbox -j4
```

## Install udev

```
sudo cp ./udev/99-uvc.rules /etc/udev/rules.d/99-uvc.rules
```

## Install for text-to-speech

```
pip3 install pydub gtts
```

## Install stereodemo for depth estimation

```
pip3 install pathlib
pip3 install torch
pip3 install git+https://github.com/nburrus/stereodemo
```


## for radxa

```
wget https://gist.githubusercontent.com/iory/cb271ef71e6e1471a6d8577f1c643654/raw/7f2ce3d46f35ca2e881af1f7c88f8bd5d225beec/i2c3.dts -O /tmp/i2c3.dts
sudo dtc -I dts -O dtb -o /boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-i2c-ee-m3-gpioa-14-gpioa-15.dtbo /tmp/i2c3.dts
```

```
od -tx1 /sys/class/i2c-adapter/i2c-3/of_node/clock-frequency
0000000 00 0f 42 40
0000004
0x0f4240 (= 1000000)になっていれば設定できている。
```

### for object segmentation

Place this repository to your catkin workspace.

```
git clone --single-branch https://github.com/iory/jsk_demos -b kxr-demos ./kxr_demos
```

```
rosdep install --from-paths -i -y -r .
pip install ultralytics[export] ncnn dill -U --no-cache-dir
```

#### Quickstart

```
roslaunch stereo_image_sandbox d405_light.launch
```

```
roslaunch stereo_image_sandbox object_detection.launch
```
