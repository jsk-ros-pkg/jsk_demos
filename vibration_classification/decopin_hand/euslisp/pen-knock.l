;; /hit_volume_threshold: 0.09

;; initial pose for fetch
;;  (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 17.5447 56.4598 -150.959 116.736 -171.816 59.0524 81.093 0.305897 15.7652)) 2000)
;;  (send *ri* :wait-interpolation)

;; initial pose for decopin-hand
;; (send *robot* :angle-vector (v+ (send *robot* :reset-pose) #f(0 110 -110 110)))
;; (send *ri* :angle-vector (send *robot* :angle-vector))

;; /hit_volume_threshold: 0.09
(ros::load-ros-manifest "sound_classification")

;; detect vibration
(defun cb (msg)
  (when (> (send msg :volume) 0.09)
    ;; please execute these command manually
    ;; open hand
    ;; (send *ri* :angle-vector (send *robot* :angle-vector (v+ (send *robot* :angle-vector) #f(0 -50 0 0))))
    ;; hand over
    ;; (send *robot* :angle-vector (v+ (send *robot* :angle-vector) #f(0 0 50 -50)))
    (format t "I heard knocking !!~% Please Ctrl-C~%")
    (format t "open hand:~%(send *ri* :angle-vector (send *robot* :angle-vector (v+ (send *robot* :angle-vector) #f(0 -50 0 0))))~%")
    (format t "hand over:~%(send *robot* :angle-vector (v+ (send *robot* :angle-vector) #f(0 0 50 -50)))~%")
    (unix:sleep 1000) ;; end of this demo
    ))
(ros::subscribe "/microphone/volume" sound_classification::Volume #'cb)

;; knock pen
(while t
 ;; start knocking
 (setq prev-time (send (ros::time-now) :sec))
 (send *robot* :angle-vector (v+ (send *robot* :angle-vector) #f(0 50 0 0)))
 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
 (unix:usleep (* 500 1000))

 ;; wait for vibration
 (while t
   (when (> (- (send (ros::time-now) :sec) prev-time) 5)
     (return))
   (ros::spin-once))
 )
