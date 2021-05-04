;; /hit_volume_threshold: 0.09

;; initial pose for fetch
;;  (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 39.4359 3.74076 -101.818 111.029 -184.879 71.7326 103.034 0.305897 15.7652)) 10000)
;; initial pose for bin
;; (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 38.5948 -5.15484 -109.725 107.609 -196.867 72.4434 107.91 0.305897 15.7652)) 10000)
;;  (send *ri* :wait-interpolation)

;; initial pose for decopin-hand
;; (send *ri* :angle-vector (send *robot* :angle-vector (v+ (send *robot* :reset-pose) #f(0 0 -120 120))))

;; /hit_volume_threshold: 0.09
(ros::load-ros-manifest "sound_classification")

;; detect vibration
(defun cb (msg)
  (when (> (send msg :volume) 0.09)
    (format t "big sound~%")
    (unix:sleep 1000))
  (format t "cb~%")
  ;; stop motion
  ;; (send *ri* :cancel-angle-vector)
  ;; (send *fetch* :angle-vector (send *ri* :state :potentio-vector))

  ;; TODO: set the action name which you want to detect
  ;; (when (equal (send msg :data) "place_bin")
  ;;   (format t "I heard knocking !!~% Please Ctrl-C~%")
  ;;   (unix:usleep (* 500 1000))
  ;;   ;; move the object to trash box
  ;;   (send *fetch* :angle-vector #f(48.096 39.4359 3.74076 -101.818 111.029 -184.879 71.7326 103.034 0.305897 15.7652))
  ;;   (send *fetch* :inverse-kinematics (make-coords :pos #f(500 -280 1050) :rpy (float-vector -pi/2 0 pi)) :use-torso nil)
  ;;   (send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
  ;;   (send *ri* :wait-interpolation)
  ;;   ;; for trash box 1 (coke can)
  ;;   ;; (send *fetch* :inverse-kinematics (make-coords :pos #f(500 -280 700) :rpy (float-vector -pi/2 0 pi)) :use-torso nil)
  ;;   ;; for trash box 2 (plastic bottle)
  ;;   ;; (send *fetch* :inverse-kinematics (make-coords :pos #f(400 -500 700) :rpy (float-vector -pi/2 0 pi)) :use-torso nil)
  ;;   ;; for trash box 3 (bin)
  ;;   (send *fetch* :inverse-kinematics (make-coords :pos #f(500 -780 700) :rpy (float-vector -pi/2 0 pi)) :use-torso nil)
  ;;   ;; move fetch
  ;;   (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  ;;   (send *ri* :wait-interpolation)
  ;;   (unix:sleep 1000) ;; end of this demo
  ;;   )
  )
(ros::subscribe "/microphone/volume" sound_classification::Volume #'cb)

;; start distinguishing gabage class
(while t
 ;; start
  (setq prev-time (send (ros::time-now) :sec))
  ;; initial pose for not bin
  ;; (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 39.4359 3.74076 -101.818 111.029 -184.879 71.7326 103.034 0.305897 15.7652)) 2000)
  ;; initial pose for bin
  ;; (send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 38.5948 -5.15484 -109.725 107.609 -196.867 72.4434 107.91 0.305897 15.7652)) 2000)
  ;; (send *ri* :wait-interpolation)
  ;; place the gabages on table
  (progn
  (send *ri* :angle-vector (send *fetch* :rarm :move-end-pos #f(0 0 -50) :world :use-torso nil) 3000)
  (unix:usleep (* 500 1000))

 ;; wait for vibration
 (while t
   ;; (when (> (- (send (ros::time-now) :sec) prev-time) 5)
   ;;   (return))
   (ros::spin-once))
 )
 )
