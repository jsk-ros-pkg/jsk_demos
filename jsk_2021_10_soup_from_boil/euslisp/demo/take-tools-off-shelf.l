(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)


(defun setup()
  ;; 最初の場所はキッチン
  (move-to-kitchen-with-cable-check)

  ;; 棚の取っ手の位置を触覚で確認
  ;; *shelf-door*の値を書き換えるので、注意
  (correct-shelf-door-coords)

  ;; 棚の取っ手の位置を触覚で認識 + 棚の開閉ができるかテスト
  (shelf-check)

  ;; PR2がデモ前におたまなどを棚の中にセットするための関数
  (set-pose)
  (now-set-in-shelf-with-dialogue-and-fail-detection)
  )
