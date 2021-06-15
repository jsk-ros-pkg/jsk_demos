# AR Sensor View

## Description

実画像にセンサーデータを重畳するために、簡易的にカメラ位置のキャリブレーションを行う。
ロボットと外部カメラの相対位置をキャリブレーションするために、ロボットに予め設定された点（TF）をカメラ画像上でクリックする。

![センサ画像サンプル](https://github.com/jsk-ros-pkg/jsk_demos/blob/master/ar_sensor_view/docs/jaxon_ar_real.png)

## Settings

- キャリブレーションされたカメラ (camera_infoメッセージが出ている)
- ロボット上に配置されたTFのリスト (LARM_LINK5等のロボットの姿勢に依存して変化するTF)

## Usage

- cameraノードのlaunch ( /for_ar_camera/image_rect_color, /for_ar_camera/camera_info 等のトピックが出ている )
- ar_sensor_view.launchのlaunch
- 指定したTFの順に、ロボットのTFに相当する部分をカメラ画像上で左クリックする
- (sample.launchでは下画像の click_target_0 ~ click_target_5 の順)
- 最後のTFをクリックした後に右クリックする（メニューが出るが、キャンセルすれば良い）

![TF位置サンプル](https://github.com/jsk-ros-pkg/jsk_demos/blob/master/ar_sensor_view/docs/jaxon_ar_v.png)

## ar_sensor_view.launch
### arguments
- カメラのトピックとして、camera/camera_info, camera/image_rect を使う
- CAMERA_NAME (default: camera)
- IMAGE_TYPE (default: image_rect)
- 使用するframe
- ORIGIN_FRAME (default: map) / camera_infoのframe_idとORIGIN_FRAME間のtfを推定し出力する
- ROBOT_FRAME (default: BODY) / robotに固定されていて、TARGET_LISTとの間のtransformationを取得できるframe
- USE_TARGET_LIST (default: true)
- TARGET_LIST (default: [] ) / クリックする対象となるframeのリスト
- 推定するカメラの初期位置(ROBOT_FRAMEに対する位置)
- DEFAULT_x (default: 3.0)
- DEFAULT_y (default: -1.2)
- DEFAULT_z (default: 1.0)
- DEFAULT_roll (default: -1.570796)
- DEFAULT_pitch (default: 0)
- DEFAULT_yaw (default: 1.570796)
