;; /hit_volume_threshold: 0.09

;; initial pose for fetch
;;  (send *ri* :angle-vector (send *fetch* :angle-vector #f(192.626 13.1919 44.2456 -137.902 44.9591 -28.9232 96.7847 -45.0 0.011324 16.2705)) 10000)
;;  (send *ri* :wait-interpolation)

;; initial pose for decopin-hand
;; (send *robot* :angle-vector (v+ (send *robot* :reset-pose) #f(0 110 -110 110)))
;; (send *ri* :angle-vector (send *robot* :angle-vector))

;; /hit_volume_threshold: 0.09
(ros::load-ros-manifest "sound_classification")

;; detect vibration
(setq knock-count 0)
(defun cb (msg)
  (format t "cb~%")
  (when (> (send msg :volume) 0.09)
    ;; stop motion
    (send *ri* :cancel-angle-vector)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))

    (format t "I heard knocking !!~% Please Ctrl-C~%")
    (setq knock-count (+ 1 knock-count))
    (unix:usleep (* 500 1000))
    (when (eq knock-count 2)
      (send *ri* :speak "hello")
      (unix:sleep 1000) ;; end of this demo
      )
    ))
(ros::subscribe "/microphone/volume" sound_classification::Volume #'cb)

;; knock pen
(while t
 ;; start knocking
  (setq prev-time (send (ros::time-now) :sec))
  ;; initial pose
  (send *ri* :angle-vector (send *fetch* :angle-vector #f(192.626 13.1919 44.2456 -137.902 44.9591 -28.9232 96.7847 -45.0 0.011324 16.2705)) 2000)
  (send *ri* :wait-interpolation)
  ;; door knock pose
  (send *ri* :angle-vector (send *fetch* :angle-vector #f(192.626 9.52753 44.0621 -139.391 31.7286 -35.2706 78.2979 -49.0853 0.011324 16.2705)) 3000)
  (unix:usleep (* 500 1000))

 ;; wait for vibration
 (while t
   (when (> (- (send (ros::time-now) :sec) prev-time) 5)
     (return))
   (ros::spin-once))
 )
