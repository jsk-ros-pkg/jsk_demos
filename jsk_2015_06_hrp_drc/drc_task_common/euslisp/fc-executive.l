#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "pcl_msgs")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/generate-valve-motion.l")

(defclass fc-executive-app
  :super propertied-object
  :slots (state go-pos-goal ocs2fc-message-handler latest-joint-angles))

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
                self :look-at-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*VALVE_REGION*
                self :valve-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*VALVE_EXEC*
                self :valve-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DOOR_REGION*
                self :door-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*DOOR_EXEC*
                self :door-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DEBRI_LINE*
                self :debri-line-command-callback)
          (list drc_com_common::OCS2FCSmall::*DEBRI_EXEC*
                self :debri-exec-command-callback))
    self)
  (:ocs2fc-message-handler () ocs2fc-message-handler)
  (:debri-line-command-callback (msg)
    (when (eq state :initial)
      (ros::publish "/debri_recognition/input_line"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/debri_recognition/snapshot/request")
      (setq state :recognizing-debri-pose)
      ))
  (:door-region-command-callback (msg)
    (when (eq state :initial)
      (multiple-value-bind
       (foreground background) (integer-vector-to-double-rect-polygon-stamped msg)
       (ros::publish "/door_recognition/input_support_plane_rect"
         foreground)
       (ros::publish "/door_recognition/input_target_rect"
         background)
       ;; enable snapshot
       (call-empty-service "/door_recognition/snapshot/request")
       (setq state :recognizing-door-pose)
       )))
  (:valve-region-command-callback (msg)
    (when (eq state :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/valve_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/valve_recognition/snapshot/request")
      (setq state :recognizing-valve-pose)
      ))
  (:go-pos-region-command-callback (msg)
    (when (eq state :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/go_pos_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/go_pos_recognition/snapshot/request")
      (setq state :recognizing-go-pos-location)
      ))
  (:debri-exec-command-callback (msg)
    (when (eq state :initial)
      (multiple-value-bind
       (coords r h)
       (float-vector-to-cylinder (send msg :float_params))
       (setq state :executing-debri)
       (ros::ros-info "removing debri at ~A ~A ~A" coords r h)
       ;; TODO: REMOVE DEBRI!
       (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
         (send msg :type drc_com_common::FC2OCSSmall::*DEBRI_FINISHED*)
         (send self :send-to-ocs msg))
       (setq state :initial)
       )))
  (:door-exec-command-callback (msg)
    (when (eq state :initial)
      (multiple-value-bind
       (coords size)
       (float-vector-to-box (send msg :float_params))
       (setq state :executing-door)
       (ros::ros-info "opening door at ~A ~A" coords size)
       ;; TODO: OPEN DOOR!
       (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
         (send msg :type drc_com_common::FC2OCSSmall::*DOOR_FINISHED*)
         (send self :send-to-ocs msg))
       (setq state :initial)
       )))
  (:valve-exec-command-callback (msg)
    (when (eq state :initial)
      (multiple-value-bind 
       (coords radius) (float-vector-to-torus (send msg :float_params))
       (setq state :executing-valve)
       (ros::ros-info "rotating valve at ~A,~A,~A~%"
         (send coords :worldpos)
         (car (send coords :rpy-angle))
         radius)
       ;; send motion to real robot
       (send-valve-motion radius coords)
       ;; send finish signal to ocs
       (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
         (send msg :type drc_com_common::FC2OCSSmall::*VALVE_FINISHED*)
         (send self :send-to-ocs msg))
       (setq state :initial)
       )))
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
      (ros::publish "/look_at_recognition/input_rect"
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
  (:lookup-from-body-transform (header)
    (let ((trans (send *tf* :lookup-transform "BODY" 
                       (send header :frame_id) (send header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "BODY" 
                            (send header :frame_id) (ros::time 0))))
        trans))
  (:debri-recog-callback (msg)
    (when (eq state :recognizing-debri-pose)
      (let ((trans (send self :lookup-from-body-transform (send msg :header)))
            (coefficients (send msg :values)))
        ;; make coordinates from coefficients
        ;; coefficients := [cx, cy, cz, dx, dy, dz, r, h]
        (let ((pos (scale 0.001 (float-vector (elt coefficients 0)
                                              (elt coefficients 1)
                                              (elt coefficients 2)))))
          (let ((coords (make-coords :pos pos)))
            (orient-coords-to-axis coords 
                                   (float-vector (elt coefficients 3)
                                                 (elt coefficients 4)
                                                 (elt coefficients 5)))
            (let ((from-body-coords (send trans :transform coords)))
              (let ((fc-msg (instance drc_com_common::FC2OCSSmall :init)))
                (send fc-msg :type drc_com_common::FC2OCSSmall::*DEBRI_POSE*)
                (send fc-msg :params (cylinder-to-float-vector
                                      from-body-coords (elt coefficients 6) (elt coefficients 7)
                                      (length (send fc-msg :params))))
                (send self :send-to-ocs fc-msg)
                (setq state :initial))))))))
  (:door-recog-callback (msg)
    (when (eq state :recognizing-door-pose)
      (let ((trans (send self :lookup-from-body-transform (send msg :header))))
        (let ((box (elt (send msg :boxes) 0)))
          (let ((coords (ros::tf-pose->coords (send box :pose))))
            (let ((from-body-coords (send trans :transform coords)))
              (let ((fc-msg (instance drc_com_common::FC2OCSSmall :init)))
                (send fc-msg :type drc_com_common::FC2OCSSmall::*DOOR_POSE*)
                (send fc-msg :params (box-to-float-vector
                                      from-body-coords (ros::tf-point->pos (send box :dimensions))
                                      (length (send fc-msg :params))))
                (send self :send-to-ocs fc-msg)
                (setq state :initial))))))))
  (:valve-recog-callback (msg)
    (when (eq state :recognizing-valve-pose)
      (let ((trans (send *tf* :lookup-transform "BODY" 
                         (send msg :header :frame_id) (send msg :header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "BODY" 
                            (send msg :header :frame_id) (ros::time 0))))
        (let ((coords (ros::tf-pose->coords (send msg :pose))))
          (let ((from-body-coords (send trans :transform coords)))
            (let ((fc-msg (instance drc_com_common::FC2OCSSmall :init)))
              (send fc-msg :params (torus-to-float-vector 
                                    from-body-coords (* (send msg :large_radius) 1000)
                                    (length (send fc-msg :params))))
              (send fc-msg :type drc_com_common::FC2OCSSmall::*VALVE_POSE*)
              (send self :send-to-ocs fc-msg)
              (setq state :initial)
              ))))
      ))
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
  (:joint-states-compressed-callback (msg)
    (setq latest-joint-angles (send msg :angles)))
  (:send-to-ocs (msg)
    (if latest-joint-angles
        (send msg :angles latest-joint-angles))
    (ros::publish "/fc_to_ocs_low_speed/input" msg))
  (:report ()
    (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
      (send msg :type drc_com_common::FC2OCSSmall::*REPORT*)
      (send self :send-to-ocs msg)))
  )


(load (robot-file "HRP2JSKNTS"))
(init-robot-from-name "HRP2JSKNTS")

(setq *go-pos-offset* 1000)
(ros::advertise "image_marker" image_view2::ImageMarker2)
(setq *app* (instance fc-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))

(ros::advertise "/go_pos_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/look_at_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/valve_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/door_recognition/input_support_plane_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/door_recognition/input_target_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/debri_recognition/input_line" geometry_msgs::PolygonStamped)
(ros::advertise "/fc_to_ocs_low_speed/input" drc_com_common::FC2OCSSmall)
(ros::subscribe "/fc_from_ocs_low_speed/output" drc_com_common::OCS2FCSmall
  #'send (send *app* :ocs2fc-message-handler) :root-callback)
(ros::subscribe "/go_pos_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :go-pos-recog-callback)
(ros::subscribe "/look_at_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :look-at-recog-callback)
(ros::subscribe "/valve_recognition/snapshot/output" jsk_recognition_msgs::Torus
  #'send *app* :valve-recog-callback)
(ros::subscribe "/door_recognition/snapshot/output" jsk_recognition_msgs::BoundingBoxArray
  #'send *app* :door-recog-callback)
(ros::subscribe "/debri_recognition/snapshot/output" pcl_msgs::ModelCoefficients
  #'send *app* :debri-recog-callback)
(ros::subscribe "/joint_states_compressed" drc_com_common::HumanoidJointAngles
  #'send *app* :joint-states-compressed-callback)
(ros::advertise "/go_pos_goal" geometry_msgs::PoseStamped)

(ros::rate 10)
(while (ros::ok)
  (ros::sleep)
  (send *app* :report)
  (ros::spin-once))

