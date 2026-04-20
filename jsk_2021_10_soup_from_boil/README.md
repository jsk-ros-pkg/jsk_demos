# soup-from-boil

スープを作るデモ

![IMG_5826](https://user-images.githubusercontent.com/38127823/136733293-a7e58b12-2150-4099-ad30-2df666da0a86.jpg)

## 準備

以下、準備の実行手順

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
準備が出来たら、

```
# ROS_MASTER_URIをPR1040にすることを忘れない (e.g. rossetmaster pr1040)
source ~/soup_ws/devel/setup.bash
roscd jsk_2021_10_soup_from_boil/euslisp/demo
roseus soup-from-boil.l
(setup)
```

で、デモの物品配置やPR2の位置調整を行う。

### setup関数の説明

最初に、PR2がキッチンのIHコンロの前まで移動する。移動前後に充電ケーブルを抜き差しするように指示されるので、抜き差しする。

```
  (send *ri* :speak-jp "準備を開始します。キッチンに移動します。")
  (move-to-kitchen)
  (update-pos)
```

以下のように出たら移動に成功．
PR2が動かなくなるなど移動に失敗した場合は、ps3joyでPR2をコンロの正面まで動かす．

```
move-to : succeeded
```

次に、PR2が操作できる位置におたまやお皿を配置する。PR2に言われたものをPR2に手渡したりキッチンに置いたりする。
注意
 - 「OKと合図をしてください」と言われる -> OKと答えるまでPR2は待つ
 - それ以外の疑問形 -> 「はい」 or 「いいえ」で答える -> 「いいえ」の場合、PR2が動作をやり直す

```
  (set-pose)
  (now-set-ladle-a-and-plate-with-dialogue)
  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)
```

最後に、IHコンロのツマミを操作できるかを試す。

```
  (send *ri* :speak-jp "IHコンロを操作できるかチェックします" :wait t)
  (ih-check)
```


## デモ

棚に物を入れるとき、IKが失敗することがある。
触覚で位置を補正しているものの、そもそものPR2のmove\_baseによる移動位置が悪ければ、棚の中に物を入れる姿勢のIKが解けない。
この場合、現状では、PR2の移動（setup関数）からやりなおすことでPR2が良い初期位置に移動するまで繰り返すしかない。

```
source ~/soup_ws/devel/setup.bash
roscd jsk_2021_10_soup_from_boil/euslisp/demo
roseus soup-from-boil.l
(main)
```
でプログラムを実行する．


### デモの内容
```
(defun main ()
  ;; 沸騰させる
  (boil-soup)
  ;; お湯を注ぐ
  (pour-soup)
  ;; 冷ます
  (cool-soup)
  )
```

- 沸騰させる : IHコンロを操作して沸騰させる．
- お湯を注ぐ : おたまを使ってお湯をコップに注ぐ．
- 冷ます : WIP!!
