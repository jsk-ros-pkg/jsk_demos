# jsk_2021_10_semi

https://github.com/k-okada/jsk_demos/tree/jsk_2021_10_semi/jsk_2021_10_semi

## ロボットモデルの作り方

```bash
sudo apt install python3 python3-pip
python3 -m pip install --user conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
```

ターミナルを立ち上げ直す.

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/semi_ws/src
cd ~/semi_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/k-okada/jsk_demos/jsk_2021_10_semi/jsk_2021_10_semi/semi.rosinstall
wstool update
rosdep update
rosdep install --from-paths . --ignore-src -y -r
cd ..
git clone https://github.com/k-okada/jsk_demos -b jsk_2021_10_semi
git clone https://github.com/sktometometo/jsk_robot -b develop/spot
cd src
ln -sf ../jsk_demos/jsk_2021_10_semi/ .
ln -sf ../jsk_robot/jsk_spotkinova_robot/ .
ln -sf ../jsk_robot/jsk_spot_robot/ .
ln -sf ../jsk_robot/jsk_kinova_robot/jsk_kinova_description/ .
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

(load "package://spotkinovaeus/spotkinova.l")
(setq *spotkinova* (spotkinova))
(objects (list *spotkinova*))

(load "package://pr2eus/pr2.l")
(setq *pr2* (pr2))
(objects (list *pr2*))
```

