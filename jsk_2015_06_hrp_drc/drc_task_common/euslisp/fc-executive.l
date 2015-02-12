#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "drc_com_common")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defclass fc-executive-app
  :super propertied-object
  :slots (state go-pos-goal ocs2fc-message-handler))

(defmethod fc-executive-app
  (:init ()
    (setq state :initial)
    (setq ocs2fc-message-handler (instance narrowband-message-handler :init))
    ;; register callbacks
    (send ocs2fc-message-handler :register-callbacks
          (list drc_com_common::OCS2FCSmall::*GO_POS_REGION*
                self :go-pos-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*GO_POS_EXEC*
                self :go-pos-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*
                self :look-at-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_EXEC*
                self :look-at-exec-command-callback))
    self)
  (:ocs2fc-message-handler () ocs2fc-message-handler)
  (:go-pos-region-command-callback (msg)
    (when (eq state :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish (format nil "~A/output/go_pos_rect" (ros::get-name)) 
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/go_pos_recognition/snapshot/request")
      (setq state :recognizing-go-pos-location)
      ))
  (:go-pos-exec-command-callback (msg)
    (when (eq state :initial)
      (let ((goal-coords (float-vector-to-coords (send msg :float_params))))
        (setq state :executing-go-pos)
        (format t "go-pos ~A ~A ~A~%"
                (* 0.001 (elt (send goal-coords :worldpos) 0))
                (* 0.001 (elt (send goal-coords :worldpos) 1))
                (rad2deg (elt (car (send goal-coords :rpy-angle)) 0)))
        (send *ri* :go-pos 
              (* 0.001 (elt (send goal-coords :worldpos) 0))
              (* 0.001 (elt (send goal-coords :worldpos) 1))
              (rad2deg (elt (car (send goal-coords :rpy-angle)) 0)))
        )
      (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
        (send msg :type drc_com_common::FC2OCSSmall::*GO_POS_FINISHED*)
        (send self :send-to-ocs msg))
      (setq state :initial)))
  (:look-at-region-command-callback (msg)
    (when (eq state :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish (format nil "~A/output/look_at_rect" (ros::get-name)) 
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/look_at_recognition/snapshot/request")
      (setq state :recognizing-look-at-point)))
  (:look-at-exec-command-callback (msg)
    (when (eq state :initial)
      (setq state :executing-look-at)
      (let ((pos (float-vector (elt (send msg :float_params) 0)
                               (elt (send msg :float_params) 1)
                               (elt (send msg :float_params) 2))))
        ;; position is respected to BODY. need to convert to tf...
        ;; update model
        (send *robot* :angle-vector (send *ri* :state :reference-vector))
        ;; fix leg to coords
        (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
        (let ((origin-to-body 
               (send (send *robot* :body_lk) :copy-worldcoords)))
          (let ((pos-from-origin (send origin-to-body :transform-vector pos)))
            (ros::ros-info "looking at ~A" pos-from-origin)
            (send *robot* :head :look-at pos-from-origin)
            (send *ri* :angle-vector (send *robot* :angle-vector) 5000) ;5 sec
            (send *ri* :wait-interpolation)))
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*LOOK_AT_FINISHED*)
          (send self :send-to-ocs msg))
        (setq state :initial)
        )))
  (:look-at-recog-callback (msg)
    (when (eq state :recognizing-look-at-point)
      (let ((trans (send *tf* :lookup-transform "BODY" 
                         (send msg :header :frame_id) (send msg :header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "BODY" 
                            (send msg :header :frame_id) (ros::time 0))))
        (let ((coords (ros::tf-pose-stamped->coords msg)))
          (let ((from-body-coords (send trans :transform coords)))
            ;; only position is important
            (let ((pos (float-vector (elt (send from-body-coords :worldpos) 0)
                                     (elt (send from-body-coords :worldpos) 1)
                                     (elt (send from-body-coords :worldpos) 2))))
              (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
                (send msg :type drc_com_common::FC2OCSSmall::*LOOK_AT_POS*)
                (setf (elt (send msg :params) 0) (elt pos 0))
                (setf (elt (send msg :params) 1) (elt pos 1))
                (setf (elt (send msg :params) 2) (elt pos 2))
                (send self :send-to-ocs msg)
                (setq state :initial)
                )))))))
  (:go-pos-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (eq state :recognizing-go-pos-location)
      (let ((trans (send *tf* :lookup-transform "BODY" 
                         (send msg :header :frame_id) (send msg :header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "BODY" 
                            (send msg :header :frame_id) (ros::time 0))))
        (let ((coords (ros::tf-pose-stamped->coords msg)))
          (let ((from-body-coords (send trans :transform coords)))
            (let ((theta (atan2 (elt (send from-body-coords :pos) 1)
                                (elt (send from-body-coords :pos) 0))))
              (let ((ux (float-vector (cos theta)
                                      (sin theta)
                                      0))
                    (uz (float-vector 0 0 1)))
                (let ((uy (v* uz ux)))
                  (let ((mat (make-matrix 3 3)))
                    (setf (matrix-column mat 0) ux)
                    (setf (matrix-column mat 1) uy)
                    (setf (matrix-column mat 2) uz)
                    (let ((goal-oriented-coords
                           (make-coords :pos (send from-body-coords :worldpos)
                                        :rot mat)))
                      (let ((offset-goal
                             (if (< (* *go-pos-offset* *go-pos-offset*)
                                    (+ (* (elt (send goal-oriented-coords :worldpos) 0)
                                          (elt (send goal-oriented-coords :worldpos) 0))
                                       (* (elt (send goal-oriented-coords :worldpos) 1)
                                          (elt (send goal-oriented-coords :worldpos) 1))))
                                 (send (send goal-oriented-coords :copy-worldcoords)
                                       :translate (float-vector (- *go-pos-offset*) 0 0))
                               (send goal-oriented-coords :copy-worldcoords) ;it's wrong probably...
                               )))
                        (ros::publish   ;for debug
                          "/go_pos_goal"
                          (ros::coords->tf-pose-stamped offset-goal "BODY"))
                        (setq go-pos-goal offset-goal)
                        (print offset-goal)
                        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
                          (send msg :type drc_com_common::FC2OCSSmall::*GO_POS_POSE*)
                          (setf (elt (send msg :params) 0) (elt (send go-pos-goal :worldpos) 0))
                          (setf (elt (send msg :params) 1) (elt (send go-pos-goal :worldpos) 1))
                          (setf (elt (send msg :params) 2) (elt (send go-pos-goal :worldpos) 2))
                          (setf (elt (send msg :params) 3) (elt (car (send go-pos-goal :rpy-angle)) 0))
                          (setf (elt (send msg :params) 4) (elt (car (send go-pos-goal :rpy-angle)) 1))
                          (setf (elt (send msg :params) 5) (elt (car (send go-pos-goal :rpy-angle)) 2))
                          (send self :send-to-ocs msg)
                          (setq state :initial)
                          ))))))))))))
  (:send-to-ocs (msg)
    (ros::publish "/fc_to_ocs_low_speed/input" msg))
  )


(load (robot-file "HRP2JSKNTS"))
(init-robot-from-name "HRP2JSKNTS")

(setq *go-pos-offset* 1000)
(ros::advertise "image_marker" image_view2::ImageMarker2)
(setq *app* (instance fc-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))

(ros::advertise (format nil "~A/output/go_pos_rect" (ros::get-name)) geometry_msgs::PolygonStamped)
(ros::advertise (format nil "~A/output/look_at_rect" (ros::get-name)) geometry_msgs::PolygonStamped)
(ros::advertise "/fc_to_ocs_low_speed/input" drc_com_common::FC2OCSSmall)
(ros::subscribe "/fc_from_ocs_low_speed/output" drc_com_common::OCS2FCSmall
  #'send (send *app* :ocs2fc-message-handler) :root-callback)
(ros::subscribe "/go_pos_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :go-pos-recog-callback)
(ros::subscribe "/look_at_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :look-at-recog-callback)
(ros::advertise "/go_pos_goal" geometry_msgs::PoseStamped)
(ros::rate 10)
(ros::spin)

