#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-behavior-utils "pr2-behavior-utils.l")

(ros::roseus-add-msgs "people_msgs")
(ros::roseus-add-msgs "face_recognition")
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
(setq *face-detected* 0)
(setq *face-recognized* "")
(setq *face-recognized-count* 0)
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
    (ros::spin-once)
    (case behavior
      (:look-at-face
       (apply #'look-at-pos args)
       )
      (:look-at-sound
       (apply #'look-at-pos args)
       )
      (:idle-random-look
       (look-at-pos-random))
      (:reply-to-person
       (apply #'reply-to-person args)))))
  
(defun position-measurement-cb (msg)
  (when (or (eq *current-behavior* :idle)
            (eq *current-behavior* :idle-random-look)
            (eq *current-behavior* :reply-to-person)
            (eq *current-behavior* :look-at-sound)
            (eq *current-behavior* :look-at-face))
      (setq msg (elt (send msg :people) 0))
      (let ((world-pos (position-measurement->look-at-pos msg)))
        (if world-pos
            (progn
              (incf *face-detected*)
              (when (< *face-detected* 0) (setq *face-detected* 0))
              (run-behavior :look-at-face world-pos))
          (decf *face-detected*)))))

(defun people-cb (msg)
  (when (and (< *face-detected* 0)
             (or (eq *current-behavior* :idle)
                 (eq *current-behavior* :idle-random-look)
                 (eq *current-behavior* :look-at-sound)
                 (eq *current-behavior* :reply-to-person)))
;;                 (eq *current-behavior* :look-at-face)))
    (let ((world-pos (people->look-at-pos msg)))
      (when world-pos
        (run-behavior :look-at-face world-pos)))))

(defun face-recognition-feedback-cb (msg)
  (let* ((f (send msg :feedback))
         (order-id (send f :order_id)))
    (when (and
           (eq order-id 1)
           (send f :names))
      (let ((name (elt (send f :names) 0))
            (confidence (elt (send f :confidence) 0)))
        (if (string= *face-recognized* name)
            (inc *face-recognized-count*)
          (progn
            (setq *face-recognized* name)
            (setq *face-recognized-count* 0)))
        (when (and *face-detected*
                   (eq *current-behavior* :look-at-face)
                   (> *face-recognized-count* 20))
          (run-behavior :reply-to-person name)
          (setq *face-recognized-count* -50))))))

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

(defun enable-cb (req)
  (setq *behavior-server-enabled* t)
  (ros::ros-info "behavior server enabled")
  (send req :response))

(defun disable-cb (req)
  (setq *behavior-server-enabled* nil)
  (ros::ros-info "behavior server disabled")
  (send req :response))

;; subscribe topics
(ros::subscribe
 "/face_detector/people_tracker_measurements_array"
 people_msgs::PositionMeasurementArray
 #'position-measurement-cb)
(ros::subscribe
 "/face_recognition/feedback"
 face_recognition::FaceRecognitionActionFeedback
 #'face-recognition-feedback-cb)
(ros::subscribe
 "/people"
 people_msgs::People
 #'people-cb)
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

