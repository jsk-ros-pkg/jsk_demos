# soup-from-boil
スープを作るデモの移動無しver. 

![IMG_5826](https://user-images.githubusercontent.com/38127823/136733293-a7e58b12-2150-4099-ad30-2df666da0a86.jpg)

## 準備

ワークスペースの作成

```
mkdir ~/soup_ws/src -p
cd ~/soup_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_demos.git
rosdep install --from-paths . --ignore-src -y -r
cd ~/soup_ws
catkin build jsk_2021_10_soup_from_boil
```

[google スライド](https://docs.google.com/presentation/d/1uuL0VSfQScqvSo1AYunSH2SAyWI3LbZ4ZQoqa1mCs14/edit?usp=sharing)に従いながらハードウェア等の準備をする．  

行うのは
- 右グリッパの付替え
- 台座防水シート
- 地図合わせ
- 鍋のセット
- 換気扇とIHコンロの準備

PR2を移動させるために、電源ケーブルを抜く。
準備が出来たら
```
# ROS_MASTER_URIをPR1040にすることを忘れない (e.g. rossetmaster pr1040)
source ~/soup_ws/devel/setup.bash
roscd jsk_2021_10_soup_from_boil/euslisp
roseus move-to-kitchen-with-map.l
(move-to-arrange-ri-direct)
```
としてPR2を位置に移動させる．
```
move-to : succeeded
```
と出たら移動に成功．
移動が失敗した場合はps3joyでPR2をコンロの正面まで動かす．
移動が終わったら電源ケーブルを挿す．

### 位置のチェック

```
source ~/soup_ws/devel/setup.bash
roscd jsk_2021_10_soup_from_boil/euslisp
roseus soup-arrange-test-20211008.l
(ih-check)
```
としてIHコンロの操作が成功するか確認することができる．
失敗した場合は、ps3joyでPR2の位置を調節する．

## 実行
```
source ~/soup_ws/devel/setup.bash
roscd jsk_2021_10_soup_from_boil/euslisp
roseus soup-arrange-test-20211008.l
(soup-arrange-all)
```
でプログラムを実行する．


### デモの内容
```
(defun soup-arrange-all ()
  (soup-arrange-0) ;; 最初の準備
  (unix:sleep 2)
  (soup-arrange-1) ;; 沸騰させる
  (unix:sleep 2)
  (soup-arrange-2) ;; お湯を注ぐ
  (unix:sleep 2)
  (soup-arrange-3) ;; 冷ます
  )
```

- 最初の準備 : PR2の音声に従いながらおたまとコップをセットする．
- 沸騰させる : IHコンロを操作して沸騰させる．
- お湯を注ぐ : おたまを使ってお湯をコップに注ぐ．
- 冷ます : WIP!!
