# jsk_2021_10_semi

https://github.com/k-okada/jsk_demos/tree/jsk_2021_10_semi/jsk_2021_10_semi

## ロボットモデルの作り方

```
source /opt/ros/melodic/setup.bash
mkdir -p semi_ws/src
cd semi_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/k-okada/jsk_demos/jsk_2021_10_semi/jsk_2021_10_semi/semi.rosinstall
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ..
git clone https://github.com/k-okada/jsk_demos -b jsk_2021_10_semi
cd src
ln -sf ../jsk_demos/jsk_2021_10_semi/ .
cd ..
catkin build -vi
source devel/setup.bash
```

とすると以下のプログラムでロボットのモデルを作ることが出来ます．インタプリタは `roseus` として実行してください．

```
(load "package://peppereus/pepper.l")
(setq *pepper* (pepper))
(objects (list *pepper*))

(load "package://naoeus/nao.l")
(setq *nao* (NaoH25V50))
(objects (list *nao*))

(load "package://baxtereus/baxter.l")
(setq *baxter* (baxter))
(objects (list *baxter*))

(load "package://fetcheus/fetch.l")
(setq *fetch* (fetch))
(objects (list *fetch*))

(load "package://pr2eus/pr2.l")
(setq *pr2* (pr2))
(objects (list *pr2*))
```

