# euslism memo

どの変数がどこの関節角を表しているかを知りたいとき

```lisp
(send *pr2* :joint-list)
```

l_shoulder_pan_jointを単独で動かしたいとき
```lisp
(send *pr2* :l_shoulder_pan_joint :joint-angle 80)
```

https://www.clearpathrobotics.com/assets/downloads/pr2/pr2_manual_r321.pdf のP16に関節名が書いてある

```lisp
reset-manip-pose ;;torso-up
tuck-arm-pose ;;腕閉じる
```

rosdistroではgo-pos-unsafeがroseus単体で使用可能なpr2eusバージョンが出ていない．単独でpr2eusをクローンしてビルドする必要がある


world
```
roslaunch pr2_gazebo pr2_empty_world.launch world_name:=/home/obinata/hogehoge.world
```

:move-trajectory-sequenceを使うとbaseとアームが同時に動かせるかも?
https://github.com/chiwunau/pr2_open_b2_door_demo/blob/master/euslisp/pull-open-door.l#L135-L140

https://github.com/chiwunau/pr2_open_b2_door_demo