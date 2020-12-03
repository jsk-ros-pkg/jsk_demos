# install
```
- git:
   local-name: openhrp3
   uri: https://github.com/fkanehiro/openhrp3.git
- git:
    local-name: hrpsys
    uri: https://github.com/ishiguroJSK/hrpsys-base.git
    version: wbms-dev
- git:
    local-name: rtmros_common
    uri: https://github.com/ishiguroJSK/rtmros_common.git
    version: wbms-dev
- git:
    local-name: rtmros_gazebo
    uri: https://github.com/start-jsk/rtmros_gazebo.git
- git:
    local-name: rtmros_tutorials
    uri: https://github.com/Naoki-Hiraoka/rtmros_tutorials.git
    version: melodic-test
- git:
    local-name: iiwa_stack
    uri: https://github.com/ishiguroJSK/iiwa_stack.git
    version: iiwa14d
- git:
    local-name: jsk_control
    uri: https://github.com/jsk-ros-pkg/jsk_control.git
- git:
    local-name: jsk_demos
    uri: https://github.com/ishiguroJSK/jsk_demos.git
    version: teleop_dual_arm
```
優先度付きIKやマスタ・スレーブ機能を使わないならhrpsysとrtmros_commonはmasterでいい  
rosdep忘れずに
```
rosdep install -y --from-paths . --ignore-src
```
gazebo model関係(クローズド)をダウンロードして
```
fnames='decoration
earthquake
electronics
food
furniture
kitchen
miscellaneous
shapes
stationery
tools
jsk_models
hospital_models'

tmp_model_path=''
for fname in ${fnames}; do
    tmp_model_path=${tmp_model_path}:/home/leus/gazebo_models/${fname}
done

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${tmp_model_path}
export GAZEBO_RESOURCE_PATH=/home/leus/gazebo_models/jsk_worlds
```
みたいな感じでパスに追加しておく

# Gazebo起動
```
reset; pkill -9 gzserver; roslaunch teleop_dual_arm gazebo_iiwa14d_no_controllers.launch
```

# hrpsys起動
```
reset; rtmlaunch teleop_dual_arm iiwa14d_hrpsys_bringup.launch
```
現状存在しないポートに対して`Port not found: /localhost:15005/wbms.rtc:master_lleg_pose`など出るが無視

# RViz起動
```
rviz -d `rospack find teleop_dual_arm`/iiwa14d.rviz
```

# マウスteleopデモ起動
```
sudo chmod +r /dev/input/mice
roslaunch teleop_dual_arm mouse_input.launch
```
マウス移動＝XY平面，左クリック＝PickUp動作  
画面誤操作しないようクリックしても反応しない場所で操作

# IKデモ起動
```
rosrun teleop_dual_arm sample_ik_call.py
```
ランダムにsetTargetPoseサービスを呼んでいるだけ

# xacro編集＆確認
改造したiiwa14dのxacroはiiwa_descriptionに置いてある
```
roslaunch urdf_tutorial display.launch model:=`rospack find iiwa_description`/urdf/iiwa14d.urdf.xacro gui:=false
```

# モデル再生成
iiwa_descriptionのxacroから諸々生成しているので
```
catkin bt --force-cmake
```
でteleop_dual_arm内の.urdfと.daeが手動で再生成できる

# 注意事項
- iiwa7   = KUKA LBR iiwa  7kg 可搬モデル
- iiwa14  = KUKA LBR iiwa 14kg 可搬モデル
- iiwa14d = iiwa14をdualに取り付けたJSK改造モデル
- iiwad   = dualなiiwaに関する設定のために追加
- iiwa_descriptionの慣習により，**_joint_0は土台としてのfixedジョイント

# TODO
- OpenHRPのModelLoader系関数を独立で呼ぶ
- iiwa14dのxacroを完全新規作成でなく，片手ずつスマートに継承して書けないか
- EffortJointControlへ移行＋トルク司令
- OpenPoseとかでノートPCのインカムでも両手Pose入力できないか

# test
