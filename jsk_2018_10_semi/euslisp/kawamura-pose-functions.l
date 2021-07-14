(setq av-list nil)
(setq tm-list (list 1000 1000 1000 1000 1000 1000 1000 1000))
(setq default-pos #f(20.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0))
(setq recieve-pos #f(20.0 46.5716 32.6914 -175.429 118.842 -224.192 70.502 -0.04456 0.0 0.0))
;;(send *fetch* :rarm :inverse-linematics (sned (send *fetch* :copy-worldcoords) :transform (make-coords #(600 0 500))) :debug-view t)

;;投げる動作と時間の設定
(setq thr1 #f(25.3106 68.8275 50.8513 -219.216 127.925 -465.502 107.636 -15.8476 0.0 0.0))
(setq thr2 #f(118.474 31.4167 29.9995 -206.956 96.9245 -391.023 -101.781 34.9092 0.0 0.0))
(setq thr3 #f(0.0 70.066 1.07513 -282.484 116.623 259.83 -123.758 171.212 0.0 0.0))
(setq throw-list (list thr3))
(setq throw-tm-list (list 1000))

(defun give-hand ()
  (send *ri* :angle-vector recieve-pos)
  (send *fetch* :angle-vector recieve-pos)
  (send *irtviewer* :look-all *fetch*)
  )

;;手を閉じる
(defun grab ()
  (send *ri* :start-grasp)
  (send *irtviewer* :look-all *fetch*)

  )
;;手を開く
(defun release ()
  (send *ri* :stop-grasp)
  (send *irtviewer* :look-all *fetch*)
  )

;;投げる
(defun throw ()
  (send *ri* :angle-vector thr1)
  (send *ri* :wait-interpolation)
  (send *ri* :go-pos-unsafe 0.1 0 0)
  ;;(send *ri* :go-pos 0.3 0 0 )
  ;(send *ri* :move-to (make-coords :pos #f(200 0 0) ) :no-wait t)
  ;;(send *ri* :angle-vector-sequence throw-list throw-tm-list)
  (send *ri* :angle-vector thr2 10)
  (unix:usleep (* 5 1000))
  (send *ri* :stop-grasp)
)


(defun dpos ()
  (send *ri* :angle-vector default-pos)
  (send *fetch* :angle-vector default-pos)
)

(defun shake-hand ()
  (dotimes (i 10)
    ;;(setq dest-y (* (cos(/ i pi)) 20.0))
    ;;(setq dest-z (+ 900 (* (cos (/ i pi)) 100.0)))
    (setq dest-y (* (sin (* i (/ pi 10.0))) 0.0))
    (setq dest-z (+ 900 (* (cos (* i (/ pi 1.0))) 50.0)))
    (setq dest-pos (float-vector 800 dest-y dest-z))
    (send *fetch*  :rarm :inverse-kinematics (make-coords :pos dest-pos :degug-view t))

    (setq av-list (append av-list (list (send *fetch* :angle-vector))))
    )
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)))
  (send *ri* :angle-vector-sequence av-list tm-list)
  (send *ri* :wait-interpolation)
  (objects (list *fetch*))
  (format t "dest-pos = ~A ~%" (list (send *fetch* :angle-vector))
  (format t "angle-vector = ~A ~%" (send *fetch* :angle-vector))
    )
  )

;;(send *ri* :state :potentio-vector)
