# 阿部メモ
## 2019/11/6

### roseus_fetch
lisp
```
source ~/semi_ws/devel/setup.bash

roscd fetcheus

;;start roscore in another terminal

rlwrap roseus

(load "package://fetcheus/fetch-interface.l")

(fetch-init)

(objects (list *fetch*))  ;;with this command, you can start a simulation instead of *ri*

(send *fetch* :init-pose)
#f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)

(send *fetch* :reset-pose)
#f(20.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0)



(send *ri* :state :potentio-vector)
  //実機での、今の状態をとってくる

(send *fetch* :angle-vector #f(0 02  20 0  とか))
or
(send *fetch* :angle-vector (send *ri* :state :potentio-vector))  //これでとってきた状態をIRTviewerに表示させる。
```

### roseus_fetch_speak

lisp
```
source ~/semi_ws/devel/setup.bash

rossetip

rossetmaster fetch15  ;;この中でroscoreが入ってる


;;有線でつなぐ

rlwrap roseus

(load "package://fetcheus/fetch-interface.l")

(fetch-init)

(send *ri* :speak-jp "日本語")

(send *ri* :speak "english")

;;関数にするには、:wait t でつぎの開始まで待つ。

```
