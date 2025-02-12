# 柏木さんとくろみつ（2023年12月のcodesignにより作成）を動かす方法（2025年1月時点）
## 環境構築
```
repositories:
  jsk_demos:
    type: git
    url: git@github.com:sawada10/jsk_demos.git
    version: add-codesigned-robot-demos
  modular_robot_model_zoo:
    type: git
    url: git@gitlab.jsk.imi.i.u-tokyo.ac.jp:sawada1/modular_robot_model_zoo.git
    version: add-kashiwagi-kuromitsu-models
  rcb4:
    type: git
    url: git@github.com:iory/rcb4.git
    version: 930687bdba6f527b366b5cebb3c807ca2c9182e9
```
を repos.yamlとして`<ワークスペース>/src/repos.yaml` に保存する。
その後以下を実行。
```
cd src
source <ワークスペース>/devel/setup.bash
vcs import < repos.yaml
rosdep install -iry --from-paths .
cd ..
catkin build

```
## 任意動作の作成と保存方法
前提：ロボットの電源を入れる（柏木さん/くろみつの場合は7.40v）
### launchファイルを立ち上げる
```
source <ワークスペース>/devel/setup.bash
roslaunch jsk_2023_12_codesign codesigned_robot.launch robot_name:=kuromitsu
```
※launchするときにrobot_nameに動かしたいロボットの名前（kashiwagi/kuromitsu）を指定する。
※joyを使いたいときは、joyをlaunchファイルを立ち上げているPCにUSB接続したうえで、`joy`を`true`に設定する。
e.g.
```
roslaunch jsk_2023_12_codesign codesigned_robot.launch joy:=true
```

### 動作を保存する
```
$ roscd jsk_2023_12_codesign
$ cd scripts
$ ipython -i robot_interface.py
> from save_angle_vector import save_angle_vector_mode
> import os
> save_angle_vector_mode(ri,“path/to/<your file name>.json”) #ここで関節角度列を保存するjsonファイルのpathを指定
> you : save # saveと打つことで動作の記録を開始
> saving angles you: end # endと打つことで動作の記録を終了
> you : playing motions # 動きの自動再生
[ENTER] saving
please input motion name : "保存動作の名前" # jsonファイルの中にshake_headで保存される（ここで保存した名前をact()関数で使える）
```

### 動作を再生する
```
$ roscd jsk_2023_12_codesign
$ cd scripts
$ ipython
> from robotmodel import *
> act("保存動作の名前",“path/to/<your file name>.json”)
```

### サーボのON/OFF
```
$ roscd jsk_2023_12_codesign
$ cd scripts
$ ipython
> from robotmodel import *
> servo_on()
> servo_off()
```

## TODO(2025-01-31)
### 目・ほっぺ・梅の光らせ方
```
source <ワークスペース>/devel/setup.bash
roslaunch jsk_2023_12_codesign codesigned_module.launch
```
した上で、以下の表に示すトピックにpublishすることで変化する

| 箇所  |　トピック | 型 | 機能 |
| ------------- | ------------- |---| --- |
| 目 | `/eye_status`  |UInt16| 表情変化 |
| ほっぺ  | `/cheek/led_blink_time`  | UInt16 |点滅回数を指定|
||`/cheek/led_duration`| UInt16 | 点滅時間を秒数で指定|
||`/cheek/led_mode`| UInt16 | 光り方を1,2,3で指定 1:常時 2:点滅 3:虹色|
||`/cheek/led_rainbow_delta_hue`| UInt16 | 虹色の光り方を1-50で指定(数字が大きいほど隣同士が別の色)|
||`/cheek/led_rgb`| ColorRGBA |r:Red, g:Green, b:Blue, a:明るさ (rgbは0-255で指定, aは10くらいでOK) |
| 梅（柏木さんのみ) | `/ume/led_blink_time` |UInt16|点滅回数を指定 |
||`/ume/led_duration`| UInt16 | 点滅時間を秒数で指定|
||`/ume/led_mode`| UInt16 |光り方を1,2,3で指定 1:常時 2:点滅 3:虹色 |
||`/ume/led_rainbow_delta_hue`| UInt16 |虹色の光り方を1-50で指定(数字が大きいほど隣同士が別の色) |
||`/ume/led_rgb`| ColorRGBA |r:Red, g:Green, b:Blue, a:明るさ (rgbは0-255で指定, aは10くらいでOK)|

<表情の対応表>
| 番号 | くろみつ |柏木さん|
|---|---|---|
|0| 通常|通常|
|1| まばたき|まばたき|
|2| キラキラ|おどろき|
|3| 眠い |眠い|
|4| 真剣 |怒る|
|5| 悲しい|悲しい|
|6| 嬉しい |楽しい|
|7|悔しい１|困る|
|8| 悔しい2|嬉しい|
|9| 退屈|期待|
|10| ハート ||
|11| ぐるぐる ||
|12| キラキラ2||

### くろみつでの動作確認
