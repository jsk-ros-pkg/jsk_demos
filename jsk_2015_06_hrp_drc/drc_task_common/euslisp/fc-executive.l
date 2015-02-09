#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "drc_com_common")
(load "package://drc_task_common/euslisp/util.l")
(defun get-ocs2fc-rect (msg)
  (let ((x1 (elt (send msg :image_roi_info) 0))
        (y1 (elt (send msg :image_roi_info) 1))
        (x2 (elt (send msg :image_roi_info) 2))
        (y2 (elt (send msg :image_roi_info) 3)))
    (let ((ret (instance geometry_msgs::PolygonStamped :init)))
      (send ret :header :stamp (ros::time-now))
      (let ((p1 (instance geometry_msgs::Point32 :init))
            (p2 (instance geometry_msgs::Point32 :init)))
        (send p1 :x x1)
        (send p1 :y y1)
        (send p2 :x x2) 
        (send p2 :y y2)
        (send ret :polygon :points (list p1 p2))
        ret))))

(defclass fc-executive-app
  :super propertied-object
  :slots (state go-pos-goal))

(defmethod fc-executive-app
  (:init ()
    (setq state :initial)
    self)
  (:narrow-message-callback (msg)
    (cond
     ((eq (send msg :action_type) drc_com_common::OCS2FCSmall::*GO_POS_REGION*)
      ;; enable snapshot
      (call-empty-service "/go_pos_recognition/snapshot/request")
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish (format nil "~A/output/go_pos_rect" (ros::get-name)) (get-ocs2fc-rect msg))
      (setq state :wait-for-go-pos-result)
      ))
    )
  (:go-pos-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (eq state :wait-for-go-pos-result)
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
                               (send goal-oriented-coords :copy-worldcoords))))
                        (ros::publish   ;for debug
                          "/go_pos_goal"
                          (ros::coords->tf-pose-stamped offset-goal "BODY"))
                        (setq go-pos-goal offset-goal)
                        (send self :return-go-pos-goal)
                        (print offset-goal))))))))))))
  (:return-go-pos-goal ()
    (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
      (send msg :type drc_com_common::FC2OCSSmall::*GO_POS_POSE*)
      (setf (elt (send msg :params) 0) (elt (send go-pos-goal :worldpos) 0))
      (setf (elt (send msg :params) 1) (elt (send go-pos-goal :worldpos) 1))
      (setf (elt (send msg :params) 2) (elt (send go-pos-goal :worldpos) 2))
      (setf (elt (send msg :params) 3) (elt (car (send go-pos-goal :rpy-angle)) 0))
      (setf (elt (send msg :params) 4) (elt (car (send go-pos-goal :rpy-angle)) 1))
      (setf (elt (send msg :params) 5) (elt (car (send go-pos-goal :rpy-angle)) 2))
      (ros::publish "/fc_to_ocs_low_speed/input" msg)
      )
    )
  )


(setq *go-pos-offset* 1000)
(setq *app* (instance fc-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))
(ros::advertise (format nil "~A/output/go_pos_rect" (ros::get-name)) geometry_msgs::PolygonStamped)
(ros::advertise "/fc_to_ocs_low_speed/input" drc_com_common::FC2OCSSmall)
(ros::subscribe "/fc_from_ocs_low_speed/output" drc_com_common::OCS2FCSmall
  #'send *app* :narrow-message-callback)
(ros::subscribe "/go_pos_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :go-pos-recog-callback)
(ros::advertise "/go_pos_goal" geometry_msgs::PoseStamped)
(ros::rate 10)
(ros::spin)

