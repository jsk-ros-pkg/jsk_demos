;; FIX
(setq *demo* t)

(load "juggle.l")
(init)

(if (eq *demo* t)
    (progn
      (require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
      (send *ri* :wait-interpolation)

      (speak-jp "今からディアボロを披露します")
      (unix::usleep 2500000)
      (speak-jp "スティックとディアボロを用意してください")

      ;; default-pose
      (send *ri* :angle-vector *default-pos* 3000)
      (send *ri* :wait-interpolation)

      ;; right hand
      (speak-jp "右手にスティックを")
      (send *ri* :stop-grasp :rarm)
      (unix::usleep 3500000)      
      (wait-for-hand-impact :rarm)
      (send *ri* :start-grasp :rarm)

      ;; left hand
      (speak-jp "左手にスティックを")
      (send *ri* :stop-grasp :larm)
      (unix::usleep 3500000)
      (wait-for-hand-impact :larm)
      (send *ri* :start-grasp :larm)

      ;; set diabolo
      (speak-jp "ディアボロを紐に一周巻いてください")
      (unix::usleep 3000000)      
      (speak-jp "ディアボロの中心の黒い部分が手前になるように注意してください")
      (wait-for-hand-impact :larm)
      (wait-for-hand-impact :rarm)      

      ;; let's diabolo
      (speak-jp "ディアボロをします")      
      (unix::usleep 3000000)      
      )
  (progn
    (unix::usleep 3000000)
    )
  )

(idle t t t)
