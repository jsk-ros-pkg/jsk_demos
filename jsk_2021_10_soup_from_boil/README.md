# soup-from-boil
スープを作るデモの移動無しver. 

![IMG_5826](https://user-images.githubusercontent.com/38127823/136733293-a7e58b12-2150-4099-ad30-2df666da0a86.jpg)

## 準備
[google スライド](https://docs.google.com/presentation/d/1uuL0VSfQScqvSo1AYunSH2SAyWI3LbZ4ZQoqa1mCs14/edit?usp=sharing)に従いながらハードウェア等の準備をする．  

行うのは
- 右グリッパの付替え
- 台座防水シート
- 地図合わせ
- 鍋のセット
- 換気扇とIHコンロの準備

準備が出来たら
```
roscd jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil
roseus move-to-kitchen-with-map.l
(move-to-arrange-ri-direct)
```
としてPR2を位置に移動させる．移動が失敗した場合はps3joyでアシストする．

### 位置のチェック

```
roscd jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil
roseus soup-arrange-test-20211008.l
(ih-check)
```
としてIHコンロの操作が成功するか確認することができる．

## 実行
```
roscd jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil
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
  ;; (unix:sleep 2)
  ;; (soup-arrange-3) ;; 冷ます
  )
```

- 最初の準備 : PR2の音声に従いながらおたまとコップをセットする．
- 沸騰させる : IHコンロを操作して沸騰させる．
- お湯を注ぐ : おたまを使ってお湯をコップに注ぐ．
- 冷ます : WIP!!