;; soup-from-boil.lとtake-tools-off-shelf.lを組み合わせたデモ
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)

(defun setup ()
  ;; 最初の場所はキッチン。
  (speech-before-move-with-rec)
  (move-to-kitchen-ri)
  (after-move)

  ;; 棚の取っ手の位置を触覚で確認
  ;; *pr2*のcoordsの値を書き換えるので、注意
  (correct-pr2-coords-at-kitchen)

  ;; PR2がデモ前におたまなどを棚の中にセットするための関数
  (set-in-shelf-with-dialogue-and-fail-detection)
  )

(defun setup()
  ;; 最初の位置はキッチン。(move-to-kitchen-ri)している状態。
  ;; 棚からおたまを取り出し、キッチンに置く
  ;; TODO: おたまだけでなく、コップも1つ取り出す
  (speech-before-move-with-rec)
  (set-from-shelf-with-failure-detection)

  ;; IHコンロの前に移動
  (go-to-arrange-from-kitchen)

  ;; IHコンロの位置を触覚で確認
  ;; *ih-knob-coords*の値を書き換えるので、注意
  (correct-ih-knob-coords)

  ;; 沸騰させる
  (boil-soup)

  ;; お湯を注ぐ
  (pour-soup)

  ;; 冷ます
  (cool-soup)
)
