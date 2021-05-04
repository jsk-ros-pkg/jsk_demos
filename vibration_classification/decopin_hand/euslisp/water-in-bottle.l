;; /hit_volume_threshold: 0.09

(ros::load-ros-manifest "sound_classification")

;; detect vibration
(defun cb (msg)
  (when (> (send msg :volume) 0.09)
    (send *fetch* :angle-vector #f(48.096 17.5447 56.4598 -150.959 116.736 -171.816 59.0524 81.093 0.305897 15.7652))
    (send *fetch* :rarm :wrist-r :joint-angle (+ 90 (send *fetch* :rarm :wrist-r :joint-angle)))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1000) ;; end of this demo
    ))
(ros::subscribe "/microphone/volume" sound_classification::Volume #'cb)

;; initial pose
;;  (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 17.5447 56.4598 -150.959 116.736 -171.816 59.0524 81.093 0.305897 15.7652)) 2000)
;;  (send *ri* :wait-interpolation)

;; shake bottle
(while t
 ;; start shaking
 (setq prev-time (send (ros::time-now) :sec))
 (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 25.0667 63.4532 -149.067 114.318 -175.933 48.3041 40.0 0.305897 15.7652)) 2000)
 (unix:usleep (* 500 1000))

 ;; wait for vibration
 (while t
   (when (> (- (send (ros::time-now) :sec) prev-time) 2)
     (return))
   (ros::spin-once))

 ;; reset to initial pose
 (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 17.5447 56.4598 -150.959 116.736 -171.816 59.0524 81.093 0.305897 15.7652)) 2000)
 (send *ri* :wait-interpolation)
 )
