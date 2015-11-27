# IREX carrying trass
## 起動方法
```
(init :real? t);;*robot* と*ri*と*trass*を作成
```
## デモの再生
まず空中でabcを入れる。
```
(start-demo)
```
で入る。デモの再生方法
```
(demo)
```

## 主要関数の説明

### reach-trass
    トラスに手を伸ばす。
    主要な動作はこの関数のオプションを変更することで作られている。

### set-pose-trass
    rviz上にインタラクティブマーカーを表示する。

### adjust-robot-by-primitive-marker
    インタラクティブマーカーで合わせたトラスの位置を元に、ロボットの位置をeuslispのモデルに反映させる。

### *rsl*
    robot state listの略で、ロボットの姿勢やロボットに送りたいコマンドを事前に保存しておける。
    demo関数は*rsl* に格納したangle-vectorや関数を順に呼び出している。
    
    ```
    send *rsl* :play-for-check;;動作の確認
    send *rsl* :play-in-real  ;;*ri*へコマンドを送る
    send *rsl* :set-rs :name :pre-reach
    ```
    など。

### *trass*
    trass.lの中で定義されているcascaded-linkの小クラス。
    指定した座標をロッドでつなぐことでトラスを生成する。
```
(defun make-rod ()
       (setq *trass* (instance trass 
             :init 
             :size (float-vector 10 1000 10)
             :pos-pair-list
             (list (list (float-vector 0 0 0) (float-vector 0 500 0))
                   (list (float-vector 0 0 0) (float-vector 0 -500 0))
                   (list (float-vector dx 0 0) (float-vector dx dy2 0))
                   )))
       (objects (list *trass*))
       *trass*
       )              
```
     で作られている。
     把持位置は、左手の場合
     (send *trass* :grasp-coords :larm)
     で取得できて、
     initでロッドを作る際に、:larmと名付けられたロッドの中点に座標を作っている。


