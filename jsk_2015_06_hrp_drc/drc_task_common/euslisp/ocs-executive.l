#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "drc_com_common")
(load "package://drc_task_common/euslisp/util.l")
;; state
;;  :initial -> ...
;;  :go-pos-region-select -> :wait-for-exec-or-cancel [rectangle-callback]
;;  :wait-for-exec-or-cancel -> :initial [:exec-callback or :cancel-callback]

(defclass ocs-executive-app
  :super propertied-object
  :slots (state task rectangle-region go-pos-coords))

(defmethod ocs-executive-app
  (:init ()
    (setq state :initial)
    self)
  (:set-image-view2-mode (mode)
    (cond 
     ((eq mode :rectangle)
      (call-empty-service "/rqt_image_view2_engine/rectangle_mode"))
     ((eq mode :grabcut-rect)
      (call-empty-service "/rqt_image_view2_engine/grabcut_rect_mode"))
     ((eq mode :line)
      (call-empty-service "/rqt_image_view2_engine/line_mode"))
     (t
      (error "unknown mode: ~A" mode)
      ))
    )
  (:show-text-on-image (text)
    (let ((msg (instance image_view2::ImageMarker2 :init)))
      (send msg :type image_view2::ImageMarker2::*TEXT*)
      (send msg :position :x 300)
      (send msg :position :y 50)
      (send msg :scale 2)
      (send msg :text text)
      (ros::publish "/image_marker" msg)
      )
    )
  (:go-pos-button-callback ()
    ;; force to use rectangle mode
    (send self :set-image-view2-mode :rectangle)
    (send self :show-text-on-image "Select rectangle region")
    (setq state :go-pos-region-select)
    (setq task :go-pos)
    )
  (:send-to-fc (msg)
    (ros::publish "/ocs_to_fc_low_speed/input" msg)
    )
  (:rectangle-callback (msg)            ;msg = geometry_msgs/PolygonStamped
    (cond
     ((eq state :go-pos-region-select)
      (setq rectangle-region msg)
      ;; send to FC to recognize pos
      (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
        (send fc-msg :action_type drc_com_common::OCS2FCSmall::*GO_POS_REGION*)
        (let ((point-a (list (send (elt (send msg :polygon :points) 0) :x)
                             (send (elt (send msg :polygon :points) 0) :y)))
              (point-b (list (send (elt (send msg :polygon :points) 1) :x)
                             (send (elt (send msg :polygon :points) 1) :y))))
          (print (list (floor (elt point-a 0))
                                             (floor (elt point-a 1))
                                             (floor (elt point-b 0))
                                             (floor (elt point-b 1))))
          (send fc-msg :image_roi_info (list (floor (elt point-a 0))
                                             (floor (elt point-a 1))
                                             (floor (elt point-b 0))
                                             (floor (elt point-b 1))
                                             0 0 0 0))
          (send self :send-to-fc fc-msg)))
      (setq state :wait-for-exec-or-cancel))
     ))
  (:cancel-callback ()
    (when (eq state :wait-for-exec-or-cancel)
      (setq state :initial)
      (setq task nil)))
  (:exec-callback ()
    (when (eq state :wait-for-exec-or-cancel)
      (cond 
       ((eq task :go-pos)
        ;; send go-pos to fc
        (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
          (send msg :action_type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
          (setf (elt (send msg :float_params) 0)
                (elt (send go-pos-coords :worldpos) 0))
          (setf (elt (send msg :float_params) 1)
                (elt (send go-pos-coords :worldpos) 1))
          (setf (elt (send msg :float_params) 2)
                (elt (send go-pos-coords :worldpos) 2))
          (setf (elt (send msg :float_params) 3)
                (elt (car (send go-pos-coords :rpy-angle)) 0))
          (setf (elt (send msg :float_params) 4)
                (elt (car (send go-pos-coords :rpy-angle)) 1))
          (setf (elt (send msg :float_params) 5)
                (elt (car (send go-pos-coords :rpy-angle)) 2))
          (send self :send-to-fc msg)
          )
        (setq state :initial)
        (setq task nil)
        )                               ;end of :go-pos
       )))
  (:narrow-message-callback (msg)
    (cond
     ((eq (send msg :type) drc_com_common::FC2OCSSmall::*GO_POS_POSE*)
      (let ((params (send msg :params)))
        ;; show go pos pose in image
        (setq go-pos-coords (make-coords :pos (float-vector (elt params 0)
                                                            (elt params 1)
                                                            (elt params 2))
                                        :rpy (float-vector (elt params 3)
                                                           (elt params 4)
                                                           (elt params 5))))
        (send self :show-text-on-image 
              (format nil "go-pos ~f ~f ~f" 
                      (* 0.001 (elt (send msg :params) 0))
                      (* 0.001 (elt (send msg :params) 1))
                      (rad2deg (elt (send msg :params) 3))
                      )))))
    )
  )

(defun go-pos-callback (req)
  (send *app* :go-pos-button-callback)
  (send req :response))
(defun exec-callback (req)
  (send *app* :exec-callback)
  (send req :response))
(defun cancel-callback (req)
  (send *app* :cancel-callback)
  (send req :response))


(setq *app* (instance ocs-executive-app :init))

;; advertise
(ros::advertise "/image_marker" image_view2::ImageMarker2)
(ros::advertise "/ocs_to_fc_low_speed/input" drc_com_common::OCS2FCSmall)
;; registering services for button UIs
(ros::advertise-service "go_pos" std_srvs::Empty #'go-pos-callback)
(ros::advertise-service "exec" std_srvs::Empty #'exec-callback)
(ros::advertise-service "cancel" std_srvs::Empty #'cancel-callback)

;; subscribing image_view2 ROI stuff
(ros::subscribe "/multisense/left/image_rect_color/screenrectangle" geometry_msgs::PolygonStamped
  #'send *app* :rectangle-callback)
(ros::subscribe "/ocs_from_fc_low_speed/output" drc_com_common::FC2OCSSmall
  #'send *app* :narrow-message-callback)

(ros::rate 10)
(ros::spin)
