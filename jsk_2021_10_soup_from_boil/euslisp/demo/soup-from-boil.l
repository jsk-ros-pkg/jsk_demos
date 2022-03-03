(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/interaction.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/tool-at-arrange.l")

;; 最初に換気扇とIHの電源を入れていく

;; add for voice recognition
(ros::roseus "curry_rec_test" :anonymous t)

(defun now-set-ladle-a-and-plate-with-dialogue ()
  ;; おたまを受け取る
  (up-rarm)
  (receive-ladle-with-dialogue)

  ;; 盛り付ける動作をして皿を人にセットしてもらう
  (arrange-motion-set)
  (arrange-motion)
  (have-the-plate-set-with-dialgue)
  (after-arrange-motion)

  ;; おたまをセットする
  (put-ladle-a)
  (reset-rarm)
  )

;; ケーブルの状態を確認し、キッチンに移動する。
(defun move-to-kitchen ()
  (speech-before-move-with-rec)
  (unix:sleep 2)
  (move-to-arrange-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep 2)
  (after-move)
  )

;; IHコンロを使ってお湯を沸騰させる
(defun boil-soup ()
  (send *ri* :speak-jp "沸騰させる調理を開始します")

  ;; 着火する
  (now-start-ih-with-fail-detection :deg 10)

  ;; 沸騰するまで待つ
  (look-at-pot)
  (ros::spin-once)
  (let ((max-temp
         (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data)))
    (if max-temp
      ;; If PR2 have thermal sensor
      (progn
        (while (< max-temp 75)
          (send *ri* :speak-jp "まだお湯は沸騰していません" :wait t)
          (unix:sleep 15)
          (setq max-temp
                (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data)))
        (send *ri* :speak-jp "お湯が沸騰しました。" :wait t))
      ;; If PR2 does not have thermal sensor
      (progn
        (look-at-pot)
        (now-boil-with-dialogue))))

  ;; 火を止める
  (push-knob)
  (set-pose-3)

  (send *ri* :speak-jp "沸騰させる調理を行いました．" :wait t)
  (send *ri* :speak-jp "次の工程に移ります" :wait t)
  )

;; スープを注ぐ
(defun pour-soup ()
  ;; 盛り付けの位置に移動する
  (move-to-arrange-with-speech-before-and-before-with-rec)

  ;; おたまを掴む
  (update-pos)
  (set-pose)
  (grasp-ladle-with-fail-detection)

  ;; 鍋を掴む
  (grasp-pot-from-below-model-with-fail-detection)

  ;; 止められるまでお湯を注ぐ
  (let ((pour-flag t))
    (until (not pour-flag)
           (now-arrange-curry)
           (send *ri* :speak-jp "まだお湯が必要ですか？" :wait t)
           (setq pour-flag (listen-speech "はい"))
           ))

  ;; 鍋とおたまを手から離す
  (release-pot-from-below-model)
  (put-ladle-a)
  (reset-rarm)
  (set-pose)

  ;;完成！！
  (send *ri* :speak-jp "お湯を注ぐ工程を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)

  (unix:sleep 2)
  )

;; スープを冷ます
(defun cool-soup ()
  (look-at-dish)
  (ros::spin-once)
  (let ((max-temp
         (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data)))
    ;; If PR2 have thermal sensor
    (if max-temp
      (while (> max-temp 65)
        (send *ri* :speak-jp "まだスープが熱いです。もう少しお待ちください。" :wait t)
        (unix:sleep 15)
        (setq max-temp
              (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data))
        )))
  (send *ri* :speak-jp "スープが完成しました" :wait t)
  )

(defun setup ()
  ;; 最初の位置はarrangeの場所 (move-to-arrange-ri-direct)
  (send *ri* :speak-jp "準備を開始します。キッチンに移動します。")
  (move-to-kitchen)

  ;; 設置するもの：おたまとお皿
  (update-pos)
  (set-pose)
  (now-set-ladle-a-and-plate-with-dialogue)
  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)

  ;; IHコンロの位置を確認
  (send *ri* :speak-jp "IHコンロを操作できるかチェックします" :wait t)
  (ih-check)
  )

(defun main ()
  (move-to-kitchen)
  (boil-soup) ;; 沸騰させる
  (pour-soup) ;; お湯を注ぐ
  (cool-soup) ;; 冷ます
  )
