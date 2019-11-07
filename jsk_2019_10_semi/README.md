# jsk_2019_10_semi

## ロボットモデルの作り方

```
source /opt/ros/melodic/setup.bash
mkdir -p semi_ws/src
cd semi_ws/src
wstool init
wstool merge https://gist.githubusercontent.com/k-okada/db02de337e957d482ebb63c7a08a218b/raw/20b19f3ad92a8576510c0e0aa02bcd311b347beb/semi.rosinstall
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ..
catkin build -vi
source devel/setup.bash
```

とすると以下のプログラムでロボットのモデルを作ることが出来ます．

```
(load "package://peppereus/pepper.l")
(setq *pepepr* (pepper))
(objects (list *pepper*))

(load "package://naoeus/nao.l")
(setq *nao* (NaoH25V50))
(objects (list *nao*))

(load "package://baxtereus/baxter.l")
(setq *pepepr* (baxter))
(objects (list *baxter*))

(load "package://fetcheus/fetch.l")
(setq *pepepr* (fetch))
(objects (list *fetch*))

(load "package://pr2eus/pr2.l")
(setq *pepepr* (pr2))
(objects (list *pr2*))
```

## Coral TPU環境のセットアップのしかた

### Coral TPUのインストール を行う

https://github.com/knorth55/coral_usb_ros#install-the-edge-tpu-runtime をみてCoral TPUをインストールする

```
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install libedgetpu1-max
sudo apt-get install python3-edgetpu
```

### Tensorflowliteのインストール を行う

https://github.com/knorth55/coral_usb_ros#install-just-the-tensorflow-lite-interpreter をみてtensorflowlite interpreterをインストールする
```
sudo apt-get install python3-pip
wget https://dl.google.com/coral/python/tflite_runtime-1.14.0-cp36-cp36m-linux_x86_64.whl
pip3 install tflite_runtime-1.14.0-cp36-cp36m-linux_x86_64.whl
```

### ワークスペースをビルドする

https://github.com/knorth55/coral_usb_ros#workspace-build-melodic
をみてワークスペースを作成しコンパイルする

```
source /opt/ros/melodic/setup.bash
mkdir -p ~/coral_ws/src
cd ~/coral_ws/src
git clone https://github.com/knorth55/coral_usb_ros.git
wstool init
wstool merge coral_usb_ros/fc.rosinstall.melodic
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/coral_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -vi
````````````````

### 学習済みモデルをダウンロードする

https://github.com/knorth55/coral_usb_ros#model-download をみてモデルをダウンロードする

```
source /opt/ros/melodic/setup.bash
source ~/coral_ws/devel/setup.bash
roscd coral_usb/scripts
python download_models.py
`````

## Coral TPUを試してみる

### USBカメラを立ち上げる

カメラノードを立ち上げる

```
source /opt/ros/melodic/setup.bash
rosrun usb_cam usb_cam_node
```

### Coralの認識ノードを立ち上げる

認識ノードを立ち上げる

```
source /opt/ros/melodic/setup.bash
source ~/coral_ws/devel/setup.bash
roslaunch coral_usb edgetpu_object_detector.launch INPUT_IMAGE:=/usb_cam/image_raw
```

### 結果を見てみる

表示ノードを立ち上げる

```
source /opt/ros/melodic/setup.bash
rosrun image_view image_view image:=/edgetpu_object_detector/output/image
```

## GitHubの使い方

例：new.lを作った時</br>
初めにやること</br>
`source ~/semi_ws/devel/setup.bash `</br>
`roscd jsk_2019_10_semi`</br>

ブランチをを移動<br>
`git checkout add_jsk_2019_10_semi`</br>

大元の内容を自分のところにもってくる</br>
`git pull origin add_jsk_2019_10_semi`

ブランチを作って移動 (ブランチが存在しない時は -bをつける)</br>
`git checkout (-b) new_branch`</br>

自分のGitを更新（addは初回のみ）</br>
`git add new.l`</br>
`git commit -m "コメント" new.l`</br>
`git push アカウント名 new_branch`

最後に自分のGitのページを開いて自分のアカウントのnew_branchからk-okada/jsk-demosのadd_jsk_2019_10_semiブランチにPullreqを送る。</br>
</br>

## ロボットシミュレーションの基本

### pepperを使った場合

* viewerのpepperを動かす：\*pepper*<br>
* シミュレーターor実機を動かす：\*ri*</br>

✱roscoreを忘れずに！</br>
<br>

どんなjointがあるのかを調べる</br>
`(send *pepper* :methods :joint)`</br>
全てのパラメータを指定する時</br>
`(send *pepper* :angle-vector #f(...))`</br>
viewerのpepperの今の状態のパラメータを知る</br>
`(send *pepper* :angle-vector)`</br>
上記のパラメータをシミュレーターに送る。</br>
`(send *ri* :angle-vector (send *pepper* :angle-vector))`</br>
実機を動かす場合は何秒かけて行うかも指定</br>
` (send *ri* :angle-vector (send *pepper* :angle-vector) 5000)`</br>
実機の今の状態のパラメータを知る</br>
`(send *ri* :state :potentio-vector)`</br>
逆運動学</br>
` (send *pepper* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 1000)) :revert-if-fail nil)`</br>
✱revert-if-fail nil : 逆運動学に失敗しても解けたところまでで中断する。</br>
<br>
fetchについては以下を参照</br>
[fetchの詳細](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot)


## Emacs豆知識
C-x C-f : ファイル作成</br>
C-x 2 : 画面上下２分割</br>
C-x b \*shell*: shellをLispにする。</br>
C-x o : 画面移動</br>

