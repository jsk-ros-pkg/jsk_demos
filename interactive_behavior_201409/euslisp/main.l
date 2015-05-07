#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-behavior-utils "pr2-behavior-utils.l")

(ros::roseus-add-msgs "people_msgs")
(ros::roseus-add-msgs "jsk_hark_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus "interactive_behavior_server")

;; initialization
(ros::ros-info "initializing")
(pr2-init)
(setq *tfl* (instance ros::transform-listener :init))
(setq *behavior-server-enabled* t)
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
  
(defun position-measurement-cb (msg)
  (when (or (eq *current-behavior* :idle)
          (eq *current-behavior* :idle-random-look)
          (eq *current-behavior* :look-at-sound)
          (eq *current-behavior* :look-at-face))
      (setq msg (elt (send msg :people) 0))
      (let ((world-pos (position-measurement->look-at-pos msg)))
        (when world-pos (run-behavior :look-at-face world-pos)))))

#|
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
|#

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
          ;;(eq *current-behavior* :look-at-sound)
          )
      ;; (send *tfl* :wait-for-transform 
      ;;       "/base_footprint"
      ;;       (send msg :header :frame_id)
      ;;       (send msg :header :stamp) 1)
      (let ((world-pos (sound-direction->look-at-pos msg)))
        (when world-pos (run-behavior :look-at-sound world-pos)))))

#|
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       ;;(send msg :header :frame_id)
                       "/head_pan_link"
                       (send msg :header :stamp))))
                       ;;(ros::time- (send msg :header :stamp) (ros::time 0.5)))))
                       ;;(ros::time 0))))
        (if cds
            (let ((local-pos (scale 3000 (float-vector (send msg :vector :x)
                                                       (send msg :vector :y)
                                                       (send msg :vector :z)))))
              (let ((world-pos (send cds :transform-vector local-pos)))
                (run-behavior :look-at-sound world-pos)))))))
|#

(defun enable-cb (req)
  (setq *behavior-server-enabled* t)
  (send req :response))

(defun disable-cb (req)
  (setq *behavior-server-enabled* nil)
  (send req :response))

;; subscribe topics
(ros::subscribe
 "/face_detector/people_tracker_measurements_array"
 people_msgs::PositionMeasurementArray
 #'position-measurement-cb)
(ros::subscribe
 "/sound_direction"
 geometry_msgs::Vector3Stamped
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

