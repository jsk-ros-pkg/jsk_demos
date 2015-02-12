#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::load-ros-manifest "drc_task_common")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/state-machine.l")
;; state
;;  :initial -> ...
;;  :go-pos-region-select -> :wait-for-exec-or-cancel [rectangle-callback]
;;  :wait-for-exec-or-cancel -> :initial [:exec-callback or :cancel-callback]

(defclass ocs-executive-app
  :super propertied-object
  :slots (state task rectangle-region go-pos-coords smachine))

(defmethod ocs-executive-app
  (:init ()
    ;;(setq state :initial)
    (setq smachine (make-state-machine
                    '((:initial -> :selecting-region-for-go-pos) ; click go-pos
                      (:selecting-region-for-go-pos -> :initial) ; cancel
                      (:selecting-region-for-go-pos -> :recognizing-go-pos-goal)
                      ;;(:recognizing-go-pos-goal -> :initial) ; cancel
                      (:recognizing-go-pos-goal -> :selecting-exec-for-go-pos) ; recognition done
                      (:selecting-exec-for-go-pos -> :executing-go-pos) ; exec
                      (:selecting-exec-for-go-pos -> :initial) ; cancel
                      (:initial -> :executing-go-pos) ; from rviz interface
                      (:executing-go-pos -> :initial) ; go pos done
                      )
                    '((:initial :initial)
                      (:selecting-exec-for-go-pos :go-pos)
                      (:selecting-region-for-go-pos :go-pos)
                      (:recognizing-go-pos-goal :go-pos)
                      (:executing-go-pos :go-pos))
                    :initial))
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
      (send msg :position :x 0)
      (send msg :position :y 100)
      (send msg :scale 5)
      (send msg :filled 5)
      (send msg :left_up_origin t)
      (send msg :text text)
      (ros::publish "/image_marker" msg)
      )
    )
  (:go-pos-button-callback ()
    (when (send smachine :next? :selecting-region-for-go-pos)
      ;; force to use rectangle mode
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text-on-image "Select rectangle region")
      (send smachine :go-to :selecting-region-for-go-pos)
      )
    )
  (:send-to-fc (msg)
    (ros::publish "/ocs_to_fc_low_speed/input" msg)
    )
  (:rectangle-callback (msg)            ;msg = geometry_msgs/PolygonStamped
    (cond
     ((eq (send smachine :state) :selecting-region-for-go-pos)
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
      (send smachine :go-to :recognizing-go-pos-goal)
     )))
  (:cancel-callback ()
    (cond
     ((eq (send smachine :state) :selecting-region-for-go-pos)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send smachine :go-to :initial))
     )
    )
  (:exec-callback ()
    (cond
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
          (send msg :action_type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
          (send msg :float_params (coords-to-float-vector go-pos-coords))
          (send self :send-to-fc msg))
      (send smachine :go-to :executing-go-pos)
      )                               ;end of :go-pos
     ))
  (:narrow-message-callback (msg)
    (cond
     ((and (eq (send msg :type) drc_com_common::FC2OCSSmall::*GO_POS_POSE*)
           (eq (send smachine :state) :recognizing-go-pos-goal))
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
                      ))
        (send smachine :go-to :selecting-exec-for-go-pos)))
     ((and (eq (send msg :type) drc_com_common::FC2OCSSmall::*GO_POS_FINISHED*)
           (eq (send smachine :state) :executing-go-pos))
      (send smachine :go-to :initial))
     )
    )
  (:go-pos-command-from-rviz-interface (req)
    ;; go to :executing-go-pos from :initial directory
    (when (eq (send smachine :state) :initial)
      (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
        (send msg :action_type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
        (send msg :float_params (coords-to-float-vector (ros::tf-pose->coords (send req :goal))))
        (send self :send-to-fc msg))
      (send smachine :go-to :executing-go-pos))
    (send req :response)
    )
  )

(defun go-pos-callback (req)
  (print 'go-pos-button)
  (send *app* :go-pos-button-callback)
  (send req :response))
(defun exec-callback (req)
  (print 'exec-button)
  (send *app* :exec-callback)
  (send req :response))
(defun cancel-callback (req)
  (print 'cancel-button)
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
(ros::advertise-service "go_pos_command_from_rviz_interface" drc_task_common::GoPosCommand #'send *app* :go-pos-command-from-rviz-interface)

;; subscribing image_view2 ROI stuff
(ros::subscribe "/multisense/left/image_rect_color/screenrectangle" geometry_msgs::PolygonStamped
  #'send *app* :rectangle-callback)
(ros::subscribe "/ocs_from_fc_low_speed/output" drc_com_common::FC2OCSSmall
  #'send *app* :narrow-message-callback)

(ros::rate 10)
(ros::spin)
