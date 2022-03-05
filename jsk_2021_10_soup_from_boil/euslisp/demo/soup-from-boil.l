(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)

(defun setup ()
  ;; 最初の位置はarrangeの場所
  (send *ri* :speak-jp "準備を開始します。キッチンに移動します。")
  (move-to-arrange-with-cable-check)
  (update-pos)

  ;; 設置するもの：おたまとお皿
  (set-pose)
  (now-set-ladle-a-and-plate-with-dialogue)
  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)

  ;; IHコンロの位置を確認
  (send *ri* :speak-jp "IHコンロを操作できるかチェックします" :wait t)
  (ih-check)
  )

(defun main ()
  ;; 沸騰させる
  (boil-soup)

  ;; お湯を注ぐ
  (pour-soup)

  ;; 冷ます
  (cool-soup)
  )
