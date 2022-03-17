(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)


(defun setup ()
  ;; 最初の場所はキッチン
  (move-to-kitchen-with-cable-check)

  ;; 棚の取っ手の位置を触覚で確認
  ;; *pr2*のcoordsの値を書き換えるので、注意
  (correct-pr2-coords-at-kitchen)

  ;; PR2がデモ前におたまなどを棚の中にセットするための関数
  (set-in-shelf-with-dialogue-and-fail-detection)
  )

(defun main ()
  ;; 棚からおたまを取り出す
  ;; TODO: (set-in-shelf-with-dialogue-and-fail-detection)が成功するのにこちらのIKに失敗することがある
  (set-from-shelf-with-failure-detection)
  )
