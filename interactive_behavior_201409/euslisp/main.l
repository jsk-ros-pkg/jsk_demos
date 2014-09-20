#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::roseus-add-msgs "people_msgs")
(ros::roseus-add-msgs "jsk_hark_msgs")
(ros::roseus "interactive_behavior_server")

;; initialization
(ros::ros-info "initializing")
(pr2-init)
(setq *tfl* (instance ros::transform-listener :init))
(setq *behavior-server-enabled* nil)
(setq *current-behavior* :idle)
(setq *behavior-timeout* 0)
(setq *behavior-start-at* (ros::time-now))
;; behaviors
;;   :idle
;;   :look-at-face
;;   :look-at-sound

(defun check-behavior-timeout ()
  (let ((now (ros::time-now)))
    (if (not (eq *current-behavior* :idle))
      (when (> (send (ros::time- now *behavior-start-at*) :to-sec) *behavior-timeout*)
        (ros::ros-warn "~A is timeout (behavior timeout was ~A)" *current-behavior* *behavior-timeout*)
        (setq *current-behavior* :idle))
      (progn                            ;idle -> idle
        (if (= (random 50) 0)          ;2%
            (progn
              (ros::ros-info "random-lookat")
              (run-behavior :idle-random-look)))))))

(defun run-behavior (behavior &rest args)
  (when *behavior-server-enabled*
    (setq *current-behavior* behavior)
    (setq *behavior-start-at* (ros::time-now))
    (case behavior
      (:look-at-face
       (apply #'look-at-pos args)
       )
      (:look-at-sound
       (apply #'look-at-pos args)
       )
      (:idle-random-look
       (look-at-pos-random)))))
  
(defun look-at-pos-random ()
  (let ((theta (deg2rad (- (random 30.0) 15))))
    (ros::ros-info "theta is ~A" theta)
    (let ((local-pos (float-vector (* 3000.0 (cos theta))
                                   (* 3000.0 (sin theta))
                                   0)))
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       "/head_pan_link"
                       (ros::time 0))))
        (if cds (look-at-pos (send cds :transform-vector local-pos)))))))

(defun look-at-pos (world-pos)
  (ros::ros-info "looking at ~A" world-pos)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((current-pitch (send *pr2* :head :neck-p :joint-angle))
        (current-yaw (send *pr2* :head :neck-y :joint-angle)))
    (send *pr2* :head :look-at world-pos) ;LOOK AT!!
    (let ((next-pitch (send *pr2* :head :neck-p :joint-angle))
          (next-yaw (send *pr2* :head :neck-y :joint-angle)))
      (let ((diff (max (abs (- next-pitch current-pitch))
                       (abs (- next-yaw current-yaw)))))
        (let ((tm (/ diff 40.0)))
          (send *ri* :angle-vector (send *pr2* :angle-vector) (* 1000.0 tm) :head-controller)
          (setq *behavior-timeout* (* 2 tm))
          )))))

(defun position-measurement-cb (msg)
  (if (or (eq *current-behavior* :idle)
          (eq *current-behavior* :idle-random-look)
          (eq *current-behavior* :look-at-sound)
          (eq *current-behavior* :look-at-face))
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       (send msg :header :frame_id)
                       (send msg :header :stamp))))
        (if cds
            (progn
              (let ((world-pos (send cds :transform-vector (ros::tf-point->pos (send msg :pos)))))
                (let ((distance-from-robot (distance
                                            (float-vector (elt world-pos 0)
                                                          (elt world-pos 1)
                                                          0)
                                            (float-vector 0 0 0))))
                  (if (< distance-from-robot 2000.0)
                      (run-behavior :look-at-face world-pos)))))
          (progn
            (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id)))))))

(defun find-max-powers-index (powers)
  (let ((max-value (elt powers 0))
        (max-index 0))
    (dotimes (i (length powers))
      (when (< max-value (elt powers i))
        (setq max-value (elt powers i))
        (setq max-index i))
      )
    max-index))
  

(defun sound-cb (msg)
  ;; offset ... - pi
  (if (or (eq *current-behavior* :idle)
          (eq *current-behavior* :idle-random-look)
          (eq *current-behavior* :look-at-sound))
      ;; (send *tfl* :wait-for-transform 
      ;;       "/base_footprint"
      ;;       (send msg :header :frame_id)
      ;;       (send msg :header :stamp) 1)
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       ;;(send msg :header :frame_id)
                       "/head_pan_link"
                       (send msg :header :stamp))))
                       ;;(ros::time- (send msg :header :stamp) (ros::time 0.5)))))
                       ;;(ros::time 0))))
        (if cds
            (let ((powers (send msg :powers)))
              ;; look up maximum position
              (let* ((max-index (find-max-powers-index powers))
                     (max-power (elt powers max-index)))
                (when (and (> max-index 4) (> max-power 32))
                    (ros::ros-info "max-power: ~A" max-power)
                  (ros::ros-info "max-index: ~A/~A" max-index (length powers))
                    ;; look at direction!!
                    (let ((theta (- (* (/ 2pi (length powers)) max-index) pi)))
                      ;; compute the posistion to look at
                      (let ((local-pos (float-vector (* 3000 (cos theta)) (* 3000 (sin theta)) 0)))
                        (let ((world-pos (send cds :transform-vector local-pos)))
                          (run-behavior :look-at-sound world-pos)
                          ))
                      )
                  ))))))
  )

(defun enable-cb (req)
  (setq *behavior-server-enabled* t)
  (send req :response))

(defun disable-cb (req)
  (setq *behavior-server-enabled* nil)
  (send req :response))

;; subscribe topics
(ros::subscribe
 "/face_detector/people_tracker_measurements"
 people_msgs::PositionMeasurement
 #'position-measurement-cb)
(ros::subscribe
 "/HarkPower"
 jsk_hark_msgs::HarkPower
 #'sound-cb)
(ros::advertise-service
 "/interactive_behavior_enable"
 std_srvs::Empty
 #'enable-cb)
(ros::advertise-service
 "/interactive_behavior_disable"
 std_srvs::Empty
 #'disable-cb)

(ros::ros-info "running main loop")

;;(ros::spin)

(ros::rate 5)
(while (ros::ok)
  (check-behavior-timeout)
  (ros::spin-once)
  (ros::sleep))

