(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)


(defun setup()
  ;; 最初の場所はキッチン
  (send *ri* :speak-jp "準備を開始します。キッチンに移動します。")
  (move-to-kitchen-with-cable-check)
  (update-pos)

  (shelf-check)

  ;; PR2がデモ前におたまなどを棚の中にセットするための関数
  ;; (set-pose)
  ;; (now-set-in-shelf-with-dialogue-and-fail-detection)

  )
