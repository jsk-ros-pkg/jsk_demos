# 金沢メモ

第四回ゼミメモ
20191106

## fetch 　
腕とベースの干渉を計算してから動かす。
moveitに送って干渉計算をしてから動かしている。
（:angle-vector-raw　だとそれをしないで実行）
eusがインターフェースでそこからROSとかでロボットに送っている。

fetchの動かし方とかはgithubを見ながら。
https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_fetch_robot

・Rviz
レーザー。
コストマップ。とか出てきてる。
```
roscd jsk_fetch_startup
cd launch
roslaunch jsk_fetch_startup rviz.launch
```
でfetch用のRvizを立ち上げられる。
2D pose estimationとかで位置を修正できる。

喋らせるとか動かすとかはlispで簡単に出来る。
・speak
```lisp
(send *ri* :speak-jp "こんにちは")
```
とかで簡単に出来る。

・移動
:go-pos　x[m] y[m] z回転[degree]　（PR2用だった。真横とかにはいきなり動けない。）
go-to-sopt （部屋のスポットを指定して移動させる。メッセージの型の問題があるから今は使えない）

物体を認識して位置を取って:go-posで相対位置で移動する。
```
(send *ri* :go-pos 1 0 0)
```


・実機で動作させる
有線でつないてwifiを切る。
```
rossetip
```
でipアドレスを確認
```
rossetmaster fetch15
```
でfetchにつなぐ。


・チート
実機でポーズを使ってこれで角度とる。
```
(send *ri* :state :potentio-vector) ;;実機の状態を取ってくる。
```

## git関連

コンフリクトするとファイルにその情報が加わって、手動でコンフリクトを解消出来る。

マークダウン方式で.md方式でメモとか作れる。
https://gist.github.com/mignonstyle/083c9e1651d7734f84c99b8cf49d57fa

・JSKのパッケージ
jsk-ros-pkg
https://github.com/jsk-ros-pkg
jsk-recognition
認識系のやつ。
コンペとか出るとこういうのバリバリ書くらしい。そういうの出来るようになりたい！
https://jsk-recognition.readthedocs.io/en/latest/

の中にある。
