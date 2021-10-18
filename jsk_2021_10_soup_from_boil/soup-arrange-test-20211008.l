(load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/move-to-kitchen-with-map.l")
(warn "please run (soup-arrange-0) or (soup-arrange-all)~%")

;; 最初に換気扇とIHの電源を入れていく

;; add for voice recognition
(ros::roseus "curry_rec_test" :anonymous t)
(ros::load-ros-manifest "speech_recognition_msgs")
(setq *data-flag* nil)
(defun voice-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *data* (send *msg* :transcript))
    (setq *data-flag* t)
    (format t "data: ~A ~%" *data*)
    )
  )

(setq *ok-flag* nil)
(defun voice-ok-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *data* (send *msg* :transcript))
    (if (equal *data* '("OK"))
        (progn
          (format t "了解しました！ ~%" *data*)
          (setq *ok-flag* t)
          )
      (format t "data: ~A ~%" *data*)
      )
    )
  )

(defun curry-dialogue ()
  (ros::unsubscribe "/speech_to_text")
  (setq *data-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-cb)
  (until *data-flag*
         (ros::spin-once)
         (ros::sleep)
         )
  (send *ri* :speak-jp "ありがとうございます")
  (unix:sleep 1)  
  )

(defun curry-ok-wait ()
  (ros::unsubscribe "/speech_to_text")
  (setq *ok-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-ok-cb)
  (until *ok-flag*
         (ros::spin-once)
         (ros::sleep)
         )
  (send *ri* :speak-jp "ありがとうございます")
  (unix:sleep *w-time*)
  )

(defun ih-check ()
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/ih-use-arrange-codes.l")
  ;; (update-pos)
  (set-pose)
  (now-start-ih-with-fail-detection :deg 10)
  (push-knob)
  (set-pose)
  )

(defun soup-arrange-0 () ;; 最初の準備
  (send *ri* :speak-jp "準備を開始します") ;; 最初の位置はarrangeの場所 (move-to-arrange-ri-direct)
  
  ;; 設置するもの
  
  ;; おたまとお皿を用意する TODOここで鍋の位置も調整してもいいかも 最初にコンロ操作を確認するのもあり？
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/tool-use-arrange-codes.l")
  ;; (update-pos)
  (set-pose)
  (now-set-ladle-a-and-plate-with-dialogue)
  
  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい")
  (curry-ok-wait)
  (send *ri* :speak-jp "次の工程に移ります")
  
  (warn "please run (soup-arrange-1) ~%")
  )


(defun soup-arrange-1 () ;; 沸騰させる
  (send *ri* :speak-jp "沸騰させる調理を開始します")
  
  ;; 着火する
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/ih-use-arrange-codes.l")
  ;; (update-pos)
  (now-start-ih-with-fail-detection :deg 10)
  
  ;; 沸騰させる
  (look-at-pot)
  (ros::spin-once)
  (if *max-temp*
    (progn
      (while (< *max-temp* 75)
        (send *ri* :speak-jp "まだお湯は沸騰していません" :wait t)
        (unix:sleep 15)
        (ros::spin-once))
      (send *ri* :speak-jp "お湯が沸騰しました。" :wait t))
    (now-boil-with-dialogue))
  
  ;; 火を止める
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/ih-use-arrange-codes.l")
  ;; (update-pos)
  (push-knob)
  (set-pose-3)

  (if *max-temp*
    (send *ri* :speak-jp "沸騰させる調理を行いました．" :wait t)
    (progn
      (send *ri* :speak-jp "沸騰させる調理を行いました．確認してokと合図をして下さい")
      (curry-ok-wait)))
  (send *ri* :speak-jp "次の工程に移ります")
  
  (warn "please run (soup-arrange-2) ~%")
  )

(defun soup-arrange-2 () ;; お湯を注ぐ
  ;; ;; 盛り付けの位置に移動する 
  ;; (move-to-arrange-with-speech-before-and-before-with-rec)
 
  ;; おたまを掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/arrangement-codes.l")
  ;; (update-pos)
  (set-pose) ;; よけいなset-poseになってしまっているがデモ固定化のために一応のこしておく．右腕がここで動いていなければなくしても問題ないかも．
  (grasp-ladle-with-fail-detection)
  
  ;; 鍋を掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/ih-use-arrange-codes.l")
  ;; (update-pos)  
  (grasp-pot-from-below-model-with-fail-detection) ;; 鍋を掴む
  
  ;; お湯を注ぐ ;; 何回かやる？
  (setq *pour-flag* nil)
  (until *pour-flag*
         (now-arrange-curry) ;; ここを改良したい？？認識を入れるなどしたい？IKが失敗したらやめるなど？
         ;; お湯を注ぐ状態を確認する
         ;; (look-at-pot) 
         (send *ri* :speak-jp "まだお湯が必要ですか？")
         (curry-dialogue)
         (unless (equal *data* '("はい"))
           (setq *pour-flag* t)
           )
         )

  (release-pot-from-below-model)
  (put-ladle-a)
  (reset-rarm)
  (set-pose)
  
  ;;完成！！
  (send *ri* :speak-jp "お湯を注ぐ工程を行いました．確認してokと合図をして下さい" :wait t)
  (curry-ok-wait)

  (warn "please run (soup-arrange-3) ~%")
  )


(defun soup-arrange-3 () ;; 冷ます
  (look-at-dish)
  (ros::spin-once)
  (if *max-temp*
    (while (> *max-temp* 65)
      (send *ri* :speak-jp "まだスープが熱いです。もう少しお待ちください。" :wait t)
      (unix:sleep 15)
      (ros::spin-once)))
  (send *ri* :speak-jp "スープが完成しました")
  ;; (warn "please run (soup-arrange-4) ~%")
  )

(defun soup-arrange-all ()
  (soup-arrange-0) ;; 最初の準備
  (unix:sleep 2)
  (soup-arrange-1) ;; 沸騰させる
  (unix:sleep 2)
  (soup-arrange-2) ;; お湯を注ぐ
  (unix:sleep 2)
  (soup-arrange-3) ;; 冷ます
  )
