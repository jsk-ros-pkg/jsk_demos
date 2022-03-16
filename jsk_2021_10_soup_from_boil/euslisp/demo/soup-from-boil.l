(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)

(defun setup ()
  ;; 最初の位置はarrangeの場所
  (move-to-arrange-with-cable-check)

  ;; IHコンロの位置を触覚で確認
  ;; *ih-knob-coords*の値を書き換えるので、注意
  (correct-ih-knob-coords)

  ;; IHコンロの操作チェック
  (ih-check)

  ;; 設置するもの：おたまとお皿
  (set-ladle-at-arrange-and-plate-with-dialogue)
  )

(defun main ()
  ;; 沸騰させる
  (boil-soup)

  ;; お湯を注ぐ
  (pour-soup)

  ;; 冷ます
  (cool-soup)
  )
