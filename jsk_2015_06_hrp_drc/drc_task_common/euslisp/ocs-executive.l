#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
;;(ros::load-ros-manifest "drc_task_common")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/state-machine.l")
(load "package://drc_task_common/euslisp/generate-valve-motion.l")
(load "package://drc_task_common/euslisp/generate-debri-motion.l")
(load "package://drc_task_common/euslisp/generate-hose-motion.l")
(load "package://drc_task_common/euslisp/generate-drill-motion.l")
(load "package://drc_task_common/euslisp/generate-door-motion.l")
(load "package://drc_task_common/euslisp/primitive-marker-util.l")

;; state
;;  :initial -> ...
;;  :go-pos-region-select -> :wait-for-exec-or-cancel [rectangle-callback]
;;  :wait-for-exec-or-cancel -> :initial [:exec-callback or :cancel-callback]

(defclass ocs-executive-app
  :super propertied-object
  :slots (state task go-pos-coords smachine look-at-pos
                fc2ocs-message-handler
                valve-params ;; (list coords radius)
                door-params ;; (list coords size)
                debri-params ;; (list coords radius height)
                hose-grasp-params ;; (list coords)
                hose-connect-params ;; (list coords)
                drill-grasp-params
                drill-put-params
                drill-wall-target-coords ;; list of coords
                drill-button-params ;; (list coords)
                foreground-bacground-sync-obj
                ;; For driving task
                latest-handle-angle
                latest-accell-angle
                show-text-buffer
                ))

(defmethod ocs-executive-app
  (:init ()
    ;;(setq state :initial)
    (setq fc2ocs-message-handler (instance narrowband-message-handler :init))
    (send fc2ocs-message-handler :register-callbacks
          (list drc_com_common::FC2OCSSmall::*REPORT*
                self :report-callback)
          (list drc_com_common::FC2OCSSmall::*GO_POS_POSE*
                self :go-pos-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*GO_POS_FINISHED*
                self :go-pos-finished-callback)
          (list drc_com_common::FC2OCSSmall::*LOOK_AT_FINISHED*
                self :look-at-finished-callback)
          (list drc_com_common::FC2OCSSmall::*VALVE_POSE*
                self :valve-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*VALVE_FINISHED*
                self :valve-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DOOR_POSE*
                self :door-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DOOR_FINISHED*
                self :door-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DEBRI_POSE*
                self :debri-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DEBRI_FINISHED*
                self :debri-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*HOSE_GRASP_POSE*
                self :hose-grasp-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*HOSE_GRASP_FINISHED*
                self :hose-grasp-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*HOSE_CONNECT_POSE*
                self :hose-connect-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*HOSE_CONNECT_FINISHED*
                self :hose-connect-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_GRASP_POSE*
                self :drill-grasp-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_GRASP_FINISHED*
                self :drill-grasp-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_PUT_POSE*
                self :drill-put-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_PUT_FINISHED*
                self :drill-put-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_WALL_POSE*
                self :drill-wall-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_WALL_FINISHED*
                self :drill-wall-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_BUTTON_POSE*
                self :drill-button-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_BUTTON_FINISHED*
                self :drill-button-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*LOOK_AROUND_FINISHED*
                self :look-around-finished-callback)
          (list drc_com_common::FC2OCSSmall::*RECOGNITION_FAILED*
                self :recognition-failed-callback)
          (list drc_com_common::FC2OCSSmall::*RVIZ_MENU_FINISHED*
                self :rviz-finished-callback)
          (list drc_com_common::FC2OCSSmall::*ANGLE_VECTOR_FINISHED*
                self :angle-vector-finished-callback))
    (setq smachine (make-ocs-state-machine))
    (send smachine :register-initial-hook self :initial-callback)
    (send smachine :go-to :initial) ;; force to move to initial state
    (send self :show-text "initialized" :success)
    self)
  (:report-callback (msg)
    ;; do nothing
    )
  (:subscribe-foreground-backggrond-topics (topics)
    (setq foreground-bacground-sync-obj
          (instance foreground-background-synchronizer :init
                    topics
                    self
                    :foreground-background-callback)))
  (:fc2ocs-message-handler () fc2ocs-message-handler)
  (:initial-callback ()
    (send self :set-image-view2-mode :none)
    (send self :set-image-view2-mode :none :panorama-view)
    ;;(send self :show-text "")
    )
  (:set-image-view2-mode (mode &optional (type :image-rect))
    "type should be :image-rect or :panorama-view"
    (let ((node (cond 
                 ((eq type :image-rect)
                  "/image_rect/rqt_image_view2_engine")
                 ((eq type :panorama-view)
                  "/panorama_view/rqt_panorama_image_view2_engine"))))
      (cond
       ((eq mode :rectangle)
        (call-empty-service (format nil "~A/rectangle_mode" node)))
       ((eq mode :grabcut-rect)
        (call-empty-service (format nil "~A/grabcut_rect_mode"  node)))
       ((eq mode :line)
        (call-empty-service (format nil "~A/line_mode" node)))
       ((eq mode :polygon)
        (call-empty-service (format nil "~A/poly_mode" node)))
       ((eq mode :none)
        (call-empty-service (format nil "~A/none_mode" node)))
       (t
        (error "unknown mode: ~A" mode)
        ))
      ))
   (:show-text (text &optional (color :success))
    (setq show-text-buffer (cons text show-text-buffer))
    ;; buffer limit is 3
    (if (> (length show-text-buffer) 3)
        (setq show-text-buffer (subseq show-text-buffer 0 3)))
    (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
      ;; TODO: should use stream
      (let ((full-text ""))
        (dolist (tx (reverse show-text-buffer))
          (setq full-text (format nil "* ~A~%~A" tx full-text)))
        (send msg :text full-text)
        (cond
         ((eq color :select)
          (send msg :fg_color *select-color*))
         ((eq color :wait)
          (send msg :fg_color *wait-color*))
         ((eq color :recognize)
          (send msg :fg_color *recognize-color*))
         ((eq color :execute)
          (send msg :fg_color *execute-color*))
         ((eq color :success)
          (send msg :fg_color *success-color*))
         ((eq color :warn)
          (send msg :fg_color *warn-color*))
         ((eq color :fatal)
          (send msg :fg_color *fatal-color*))
         (t
          (send msg :fg_color *unknown-color*)))
        (send msg :bg_color (ros-rgba 110 100 210 0.2))
        (ros::publish "/ocs/rviz/visualization/ocs_executive_message" msg))
      ))
  (:send-to-fc (msg)
    (ros::publish "/ocs_to_fc_low_speed/input" msg))
  (:foreground-background-callback (foreground background)
    (cond
     ((eq (send smachine :state) :selecting-region-for-door)
      (send self :double-rectangles-callback-for-door-recognition foreground background))
     ))
  )

;; button callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:drive-button-callback (req)
    (when (send smachine :next? :driving)
      (send self :show-text "Driving mode" :select)
      (send smachine :go-to :driving))
    )
  (:valve-button-callback (req)
    (when (send smachine :next? :selecting-region-for-valve)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle region of valve" :select)
      (send smachine :go-to :selecting-region-for-valve))
    (send req :response))
  (:door-button-callback (req)
    (when (send smachine :next? :selecting-region-for-door)
      (send self :set-image-view2-mode :grabcut-rect)
      (send self :show-text "Select rectangle two region for door" :select)
      (send smachine :go-to :selecting-region-for-door))
    (send req :response))
  (:debri-button-callback (req)
    (when (send smachine :next? :selecting-region-for-debri)
      (send self :set-image-view2-mode :line)
      (send self :show-text "Select line for debri" :select)
      (send smachine :go-to :selecting-region-for-debri))
    (send req :response))
  (:hose-grasp-button-callback (req)
    (when (send smachine :next? :selecting-region-for-hose-grasp)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle for hose-grasp" :select)
      (send smachine :go-to :selecting-region-for-hose-grasp))
    (send req :response))
  (:hose-connect-button-callback (req)
    (when (send smachine :next? :selecting-region-for-hose-connect)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle for hose-connect" :select)
      (send smachine :go-to :selecting-region-for-hose-connect))
    (send req :response))
  (:drill-grasp-button-callback (req)
    (when (send smachine :next? :selecting-region-for-drill-grasp)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle region for drill-grasp" :select)
      (send smachine :go-to :selecting-region-for-drill-grasp))
    (send req :response))
  (:drill-put-button-callback (req)
    (when (send smachine :next? :selecting-region-for-drill-put)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle region for drill-put" :select)
      (send smachine :go-to :selecting-region-for-drill-put))
    (send req :response))
  (:drill-wall-button-callback (req)
    (when (send smachine :next? :selecting-region-for-drill-wall)
      (send self :set-image-view2-mode :polygon)
      (send self :show-text "Select points for drill-wall" :select)
      (send smachine :go-to :selecting-region-for-drill-wall))
    (send req :response))
  (:drill-button-button-callback (req)
    (when (send smachine :next? :selecting-region-for-drill-button)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle for drill-button ;; not implemented yet" :select)
      (send smachine :go-to :selecting-region-for-drill-button))
    (send req :response))
  (:go-pos-button-callback (req)
    (when (send smachine :next? :selecting-region-for-go-pos)
      ;; force to use rectangle mode
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle region to go-pos" :select)
      (send smachine :go-to :selecting-region-for-go-pos)
      )
    (send req :response))
  (:look-at-button-callback (req)
    (when (send smachine :next? :selecting-region-for-look-at)
      ;; force to use rectangle mode
      (send self :set-image-view2-mode :rectangle)
      (send self :set-image-view2-mode :rectangle :panorama-view) ;; we support panorama view
      (send self :show-text "Select rectangle region to look-at" :select)
      (send smachine :go-to :selecting-region-for-look-at)
      )
    (send req :response))
  (:look-around-button-callback (req)
    (when (send smachine :next? :executing-look-around)
      (send self :show-text "Execute look-around motion" :execute)
      (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
        (send fc-msg :type drc_com_common::OCS2FCSmall::*LOOK_AROUND_EXEC*) ;; go to EXEC state directory in look-around
        (send self :send-to-fc fc-msg)
        (send smachine :go-to :executing-look-around)))
    (send req :response))
  )

;; rectangle / line callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:line-callback (msg) ;; msg = geometry_msgs/PolygonStamped
    (cond
     ((eq (send smachine :state) :selecting-region-for-debri)
      (send self :line-callback-for-debri-line msg))))
  (:polygon-callback (msg)
    (cond
     ((eq (send smachine :state) :selecting-region-for-drill-wall)
      (send self :polygon-callback-for-drill-wall msg))))
  (:rectangle-callback (msg) ;; msg = geometry_msgs/PolygonStamped
    (cond
     ((eq (send smachine :state) :selecting-region-for-go-pos)
      (send self :rectangle-callback-for-go-pos-region msg))
     ((eq (send smachine :state) :selecting-region-for-look-at)
      (send self :rectangle-callback-for-look-at-region msg))
     ((eq (send smachine :state) :selecting-region-for-valve)
      (send self :rectangle-callback-for-valve-region msg))
     ((eq (send smachine :state) :selecting-region-for-hose-grasp)
      (send self :rectangle-callback-for-hose-grasp-region msg))
     ((eq (send smachine :state) :selecting-region-for-hose-connect)
      (send self :rectangle-callback-for-hose-connect-region msg))
     ((eq (send smachine :state) :selecting-region-for-drill-grasp)
      (send self :rectangle-callback-for-drill-grasp-region msg))
     ((eq (send smachine :state) :selecting-region-for-drill-put)
      (send self :rectangle-callback-for-drill-put-region msg))
     ((eq (send smachine :state) :selecting-region-for-drill-button)
      (send self :rectangle-callback-for-drill-button-region msg))
     ))
  (:rectangle-callback-for-valve-region (msg)
    (send self :show-text "Wait for valve recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*VALVE_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-valve-pose)))
  (:double-rectangles-callback-for-door-recognition (foreground background)
    (send self :show-text "Wait for door recognition" :wait)
    ;;(print (list foreground background))
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DOOR_REGION*)
      (send fc-msg :image_roi_info (double-rect-polygon-stamped-to-integer-vector
                                    foreground background (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-door-pose)))
  (:line-callback-for-debri-line (msg)
    (send self :show-text "Wait for debri recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DEBRI_LINE*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-debri-pose)))
  (:polygon-callback-for-drill-wall (msg)
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_WALL_POINTS*)
      (let ((poly-integer-vector (polygon-stamped-to-integer-vector
                                  msg (length (send fc-msg :image_roi_info)))))
        (if poly-integer-vector
            (progn
              (send self :show-text "Wait for wall recognition" :wait)
              (send fc-msg :image_roi_info poly-integer-vector)
              (send self :send-to-fc fc-msg)
              (send smachine :go-to :recognizing-drill-wall))
          (progn
            (send self :show-text "Failed to convert to integer vector, too many points" :fatal)
            nil)
          ))))
  (:rectangle-callback-for-hose-grasp-region (msg)
    (send self :show-text "Wait for hose-grasp recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*HOSE_GRASP_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-hose-grasp-pose)))
 (:rectangle-callback-for-hose-connect-region (msg)
    (send self :show-text "Wait for hose-connect recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*HOSE_CONNECT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-hose-connect-pose)))
  (:rectangle-callback-for-drill-grasp-region (msg)
    (send self :show-text "Wait for drill-grasp recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_GRASP_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-drill-grasp-pose)))
  (:rectangle-callback-for-drill-put-region (msg)
    (send self :show-text "Wait for drill-put recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_PUT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-drill-put-pose)))
  (:rectangle-callback-for-drill-button-region (msg)
    (send self :show-text "Wait for drill-button recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_BUTTON_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-drill-button-pose)))
  (:rectangle-callback-for-go-pos-region (msg)
    (send self :show-text "Wait for go-pos recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*GO_POS_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-go-pos-goal)))
  (:rectangle-callback-for-look-at-region (msg)
    (send self :show-text "Wait for look-at recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :executing-look-at))) ;; go to executing state directory in look-at
  (:panorama-rectangle-callback (msg)
    (send self :show-text "Wait for look-at recognition" :wait)
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send fc-msg :image_type drc_com_common::OCS2FCSmall::*PANORAMA_VIEW*)
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :executing-look-at))) ;; go to executing state directory in look-at
  )

;; cancel callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:cancel-button-callback (req)
    (send self :show-text "Canceled" :warn)
    (cond
     ;; from selecting-region state to initial state ;;;;
     ((eq (send smachine :state) :selecting-region-for-valve)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-door)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-debri)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-hose-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-hose-connect)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-drill-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-drill-put)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-drill-wall)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-drill-button)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-go-pos)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-region-for-look-at)
      (send smachine :go-to :initial))
     ;; there is no selecting-region state for look-around
     ;; from recgnizing state to initial state ;;;;
     ;; TODO : cancel recognition from button
     ((eq (send smachine :state) :recognizing-valve-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-door-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-debri-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-hose-grasp-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-hose-connect-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-drill-grasp-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-drill-put-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-drill-wall)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-drill-button-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-go-pos-goal)
      (send smachine :go-to :initial))
     ;; there is no recognition state for look-at and look-around
     ;; from selecting-exec state to initial state ;;;;
     ((eq (send smachine :state) :selecting-exec-for-valve)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-door)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-debri)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-hose-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-hose-connect)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-drill-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-drill-put)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-drill-wall)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-drill-button)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send smachine :go-to :initial))
     ;; there is no selecting-region state for look-at and look-around
     )
    ;; clear text message
    (send req :response))
  )

;; exec callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:exec-valve-button-callback ()
    (send self :show-text "Execute valve motion" :execute)
    ;; update latest valve params
    (setq valve-params
          (list (get-primitive-marker-pose :frame-id "BODY")
                (elt (get-primitive-marker-dimensions) 3)))
    ;; send valve params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*VALVE_EXEC*)
      (send msg :float_params
            (torus-to-float-vector 
             (car valve-params) (cadr valve-params)
             (length (send msg :float_params)))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-valve)))
  (:exec-door-button-callback ()
    (send self :show-text "Execute door motion" :execute)
    ;; TODO : update latest door params
    ;; TODO : send door params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DOOR_EXEC*)
      (send msg :float_params (box-to-float-vector 
                               (car door-params) (cadr door-params)
                               (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-door)))
  (:exec-debri-button-callback ()
    (send self :show-text "Execute debri motion" :execute)
    ;; update latest debri params
    (let* ((dim (get-primitive-marker-dimensions)))
      (setq debri-params
            (list (get-primitive-marker-pose :frame-id "BODY") (elt dim 3) (elt dim 2))))
    ;; send debri params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DEBRI_EXEC*)
      (send msg :float_params (cylinder-to-float-vector
                               (elt debri-params 0) (elt debri-params 1) (elt debri-params 2)
                               (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-debri)))
  (:exec-hose-grasp-button-callback ()
    (send self :show-text "Execute hose-grasp motion" :execute)
    ;; update latest hose-grasp params
    (setq hose-grasp-params
          (list (get-primitive-marker-pose :frame-id "BODY")))
    ;; send hose-grasp params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*HOSE_GRASP_EXEC*)
      (let ((pos (send (elt hose-grasp-params 0) :worldpos)))
        (setf (elt (send msg :float_params) 0) (elt pos 0))
        (setf (elt (send msg :float_params) 1) (elt pos 1))
        (setf (elt (send msg :float_params) 2) (elt pos 2)))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-hose-grasp)))
  (:exec-hose-connect-button-callback ()
    (send self :show-text "Execute hose-connect motion" :execute)
    ;; update latest hose-connect params
    (setq hose-connect-params
          (list (get-primitive-marker-pose :frame-id "BODY")))
    ;; send hose-connect params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*HOSE_CONNECT_EXEC*)
      (let ((pos (send (elt hose-connect-params 0) :worldpos)))
        (setf (elt (send msg :float_params) 0) (elt pos 0))
        (setf (elt (send msg :float_params) 1) (elt pos 1))
        (setf (elt (send msg :float_params) 2) (elt pos 2)))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-hose-connect)))
  (:exec-drill-grasp-button-callback ()
    (send self :show-text "Execute drill-grasp motion" :execute)
    ;; update latest drill-grasp params
    (setq drill-grasp-params
          (list (get-primitive-marker-pose :frame-id "BODY")))
    ;; send drill-grasp params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_GRASP_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-grasp-params 0) (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-grasp)))
  (:exec-drill-put-button-callback ()
    (send self :show-text "Execute drill-put motion" :execute)
    ;; update latest drill-put params
    (setq drill-put-params
          (list (get-primitive-marker-pose :frame-id "BODY")))
    ;; send drill-put params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_PUT_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-put-params 0) (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-put)))
  (:exec-drill-wall-button-callback ()
    (send self :show-text "Execute drill-wall motion" :execute)
    ;; do something magical!!
    ;; send drill-wall params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_WALL_EXEC*)
      (send msg :float_params (coords-set-to-float-vector drill-wall-target-coords (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-wall)))
  (:exec-drill-button-button-callback ()
    (send self :show-text "Execute drill-button motion" :execute)
    ;; update latest drill-button params
    (setq drill-button-params
          (list (get-primitive-marker-pose :frame-id "BODY")))
    ;; do something magical!!
    ;; send drill-button params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_BUTTON_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-button-params 0) (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-button)))
  (:exec-go-pos-button-callback ()
    ;; update latest go-pos coords
    (setq go-pos-coords (send *tf* :lookup-transform "hrp2_marker_root" "ground" (ros::time 0)))
    (setq go-pos-coords (send go-pos-coords :transformation (make-coords) :local))
    ;; send go-pos coords to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
      (send msg :float_params 
            (coords-to-float-vector 
             go-pos-coords (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-go-pos)))
  (:exec-button-callback (req)
    (cond
     ((eq (send smachine :state) :selecting-exec-for-valve)
      (send self :exec-valve-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-door)
      (send self :exec-door-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-debri)
      (send self :exec-debri-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-hose-grasp)
      (send self :exec-hose-grasp-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-hose-connect)
      (send self :exec-hose-connect-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-drill-grasp)
      (send self :exec-drill-grasp-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-drill-put)
      (send self :exec-drill-put-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-drill-wall)
      (send self :exec-drill-wall-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-drill-button)
      (send self :exec-drill-button-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send self :exec-go-pos-button-callback))
     )
    (send req :response))
  )

;; recognition finised callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:valve-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-valve-pose)
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords radius) (float-vector-to-torus params)
         (send self :show-text 
               (format nil "Recognize valve. pos:~A rpy:~A r:~1f"
                       (send coords :worldpos) (reverse (car (send coords :rpy-angle))) radius) :recognize)
         ;; publish object marker
         (erase-all-primitive-marker)
         (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*TORUS* :name "torus1" :description "")
         (set-primitive-marker-dimensions :radius radius :small-radius (* 0.1 radius))
         (set-primitive-marker-pose coords "BODY")
         (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
         (apply-primitive-dimensions-to-midi-device)
         (move-robot-model-to-current-pos)
         ;; store result
         (setq valve-params (list coords radius))
         ;; publish motion to robot model on rviz
         (publish-valve-motion :radius (cadr valve-params) :coords (car valve-params) :update? t)
         ))
      (send smachine :go-to :selecting-exec-for-valve)))
  (:door-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-door-pose)
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords size) (float-vector-to-box params)
         (send self :show-text (format nil "Recognize door. param:~A" params) :recognize)
         ;; publish object marker
         (erase-all-primitive-marker)
         (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX* :name "box1" :description "")
         (set-primitive-marker-dimensions :x (elt size 0) :y (elt size 1) :z (elt size 2))
         (set-primitive-marker-pose coords "BODY")
         (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
         (apply-primitive-dimensions-to-midi-device)
         (move-robot-model-to-current-pos)
         ;; store result
         (setq door-params (list coords size))
         ;; publish motion to robot model on rviz
         (publish-door-motion :coords (elt door-params 0) :knob-size (elt door-params 1) :update? t)
         (send smachine :go-to :selecting-exec-for-door)))
      ))
  (:debri-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-debri-pose)
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords r h) (float-vector-to-cylinder params)
         (send self :show-text
               (format nil
                       "Recognize debri. pos:~A rpy:~A r:~1f h:~1f" (send coords :worldpos) (reverse (car (send coords :rpy-angle))) r h) :recognize)
         ;; publish object marker
         (erase-all-primitive-marker)
         (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
         (set-primitive-marker-dimensions :radius r :z h)
         (set-primitive-marker-pose coords "BODY")
         (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
         (apply-primitive-dimensions-to-midi-device)
         (move-robot-model-to-current-pos)
         ;; store result
         (setq debri-params (list coords r h))
         ;; publish motion to robot model on rviz
         (publish-debri-motion :coords coords :radius r :height h :handle-ratio 0.9 :update? t)
         (send smachine :go-to :selecting-exec-for-debri)))))
  (:hose-grasp-recognition-finished-callback (msg)
    ;; TODO : receive all hose information
    (when (eq (send smachine :state) :recognizing-hose-grasp-pose)
      (let* ((params (send msg :params))
             (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rpy (list 0 pi 0))))
        (send self :show-text 
              (format nil "Recognize hose. pos:~A" (send coords :worldpos)) :recognize)
        ;; store result
        (setq hose-grasp-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
        (set-primitive-marker-dimensions :radius 30 :z 120)
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
        (publish-hose-grasp-motion :coords (elt hose-grasp-params 0) :update? t)
        (send smachine :go-to :selecting-exec-for-hose-grasp))
      ))
  (:hose-connect-recognition-finished-callback (msg)
    ;; TODO : receive all hose information
    (when (eq (send smachine :state) :recognizing-hose-connect-pose)
      (let* ((params (send msg :params))
             (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rpy (list 0 pi/2 0))))
        (send self :show-text 
              (format nil "Recognize hose. pos:~A" (send coords :worldpos)) :recognize)
        ;; store result
        (setq hose-connect-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
        (set-primitive-marker-dimensions :radius 30 :z 50)
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
        (publish-hose-connect-motion :coords (elt hose-connect-params 0) :update? t)
        (send smachine :go-to :selecting-exec-for-hose-connect))
      ))
  (:drill-grasp-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-grasp-pose)
      (let* ((params (send msg :params))
             (coords (float-vector-to-coords params)))
        (send self :show-text 
              (format nil "Recognize drill. pos:~A" (send coords :worldpos)) :recognize)
        ;; store result
        (setq drill-grasp-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (cond 
         (nil ;(equal *drill-type* :gun) ;;todo
          (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
          )
         (t ;(equal *drill-type* :takenoko) ;;todo
          (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
          (set-primitive-marker-control-relative-pose (make-coords :pos (float-vector -22 30 97) :rpy (float-vector 0 0 pi/2))) ;;Todo more precise pose for button
          ))
        ;(set-primitive-marker-dimensions :radius 30 :z 300)
        (set-primitive-marker-pose coords "BODY")
        ;; (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        ;; (apply-primitive-dimensions-to-midi-device)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
	(publish-drill-grasp-motion :coords (elt drill-grasp-params 0) :update? t) ;todo drill-type
        (send smachine :go-to :selecting-exec-for-drill-grasp))
      ))
  (:drill-put-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-put-pose)
      (let* ((params (send msg :params))
             (coords (float-vector-to-coords params)))
        (send self :show-text 
              (format nil "Recognize drill. pos:~A" (send coords :worldpos)) :recognize)
        ;; store result
        (setq drill-put-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
        (set-primitive-marker-dimensions :radius 300 :z 30) ;; show plane as circle
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
	(publish-drill-put-motion :coords (elt drill-put-params 0) :update? t)
        (send smachine :go-to :selecting-exec-for-drill-put))
      ))
  (:drill-wall-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-wall)
      (let ((params (send msg :params)))
        (let ((coords-set (float-vector-to-coords-set params)))
	  (send self :show-text 
		(format nil "Recognize wall coords") :recognize)
          (publish-coords-list coords-set "BODY")
          (setq drill-wall-target-coords coords-set)
          (publish-drill-wall-motion :coords-list coords-set :update? t)
          (send smachine :go-to :selecting-exec-for-drill-wall)))))
  (:drill-button-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-button-pose)
      (let* ((params (send msg :params))
             (coords (float-vector-to-coords params)))
        (send self :show-text 
              (format nil "Recognize drill-button. pos:~A rpy:~A" (send coords :worldpos) (reverse (car (send coords :rpy-angle)))) :recognize)
        ;; store result
        (setq drill-button-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://hrpsys_gazebo_tutorials/environment_models/dewalt_dc550ka/meshes/dewalt_dc550ka.dae" :use-material t)
	(set-primitive-marker-control-relative-pose (make-coords :pos (float-vector -25 80 95) :rpy (float-vector 0 0 pi/2))) ;;Todo more precise pose for button
        ;;(set-primitive-marker-pose coords "BODY") ;; comment out because pose is not correct
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
	;;(publish-drill-button-motion :coords (elt drill-button-params 0) :update? t) ;; comment out because pose is not correct
        (send smachine :go-to :selecting-exec-for-drill-button))))
  (:go-pos-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-go-pos-goal)
      (let ((params (send msg :params)))
        (setq go-pos-coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2))
                                         :rpy (float-vector (elt params 3) (elt params 4) (elt params 5))))
        (send self :show-text 
              (format nil "Recognize go-pos ~1f ~1f ~1f" (* 0.001 (elt params 0)) (* 0.001 (elt params 1)) (rad2deg (elt params 3))) :recognize)
        ;; move robot model on rviz and send potentio-vector
        (move-robot-model go-pos-coords "ground")
        (publish-model-joint-states :av (get-potentio-vector-from-joint-states))
        (send smachine :go-to :selecting-exec-for-go-pos))))
  (:recognition-failed-callback (msg)
    (when (send smachine :next? :initial)
      (send self :show-text "Recognition failed, go to :initial state" :fatal)
      (send smachine :go-to :initial)))
  (:rviz-finished-callback (msg)
    (send self :show-text "rviz menu succeeded!" :success)
    )
  )

;; exec finised callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:valve-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-valve :success)
      (send self :show-text "Finished valve motion")
      (send smachine :go-to :initial)))
  (:door-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-door)
      (send self :show-text "Finished door motion" :success)
      (send smachine :go-to :initial)))
  (:debri-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-debri)
      (send self :show-text "Finished debri motion" :success)
      (send smachine :go-to :initial)))
  (:hose-grasp-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-hose-grasp)
      (send self :show-text "Finished hose-grasp motion" :success)
      (send smachine :go-to :initial)))
  (:hose-connect-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-hose-connect)
      (send self :show-text "Finished hose-connect motion" :success)
      (send smachine :go-to :initial)))
  (:drill-grasp-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-drill-grasp)
      (send self :show-text "Finished drill-grasp motion" :success)
      (send smachine :go-to :initial)))
  (:drill-put-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-drill-put)
      (send self :show-text "Finished drill-put motion" :success)
      (send smachine :go-to :initial)))
  (:drill-wall-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-drill-wall)
      (send self :show-text "Finished drill-wall motion" :success)
      (send smachine :go-to :initial)))
  (:drill-button-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-drill-button)
      (send self :show-text "Finished drill-button motion" :success)
      (send smachine :go-to :initial)))
  (:go-pos-finished-callback (msg)
    (when (eq (send smachine :state) :executing-go-pos)
      (send self :show-text "Finished go-pos motion" :success)
      (send smachine :go-to :initial)))
  (:look-at-finished-callback (msg)
    (when (eq (send smachine :state) :executing-look-at)
      (send self :show-text "Finished look-at motion" :success)
      (send smachine :go-to :initial)))
  (:look-around-finished-callback (msg)
    (when (eq (send smachine :state) :executing-look-around)
      (send self :show-text "Finished look-around motion" :success)
      (send smachine :go-to :initial)
      ))
  (:angle-vector-finished-callback (msg)
      (send-angle-vector-finished-service)
   )
  )

;; reflesh callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:reflesh-button-callback (req)
    (cond ((eq (send smachine :state) :selecting-exec-for-valve)
           ;; update latest valve params
           (setq valve-params
                 (list (get-primitive-marker-pose :frame-id "BODY")
                       (elt (get-primitive-marker-dimensions) 3)))
           ;; publish valve motion to robot model on rviz
           (publish-valve-motion :radius (cadr valve-params) :coords (car valve-params) :update? t))
          ((eq (send smachine :state) :selecting-exec-for-debri)
           ;; update latest debri params
           (let* ((dim (get-primitive-marker-dimensions)))
             (setq debri-params
                   (list (get-primitive-marker-pose :frame-id "BODY") (elt dim 3) (elt dim 2))))
           ;; publish valve motion to robot model on rviz
           (publish-debri-motion :coords (elt debri-params 0) :radius (elt debri-params 1) :height (elt debri-params 2) :handle-ratio 0.9 :update? t))
          ((eq (send smachine :state) :selecting-exec-for-hose-grasp)
           ;; update latest hose-grasp params
           (setq hose-grasp-params
                 (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish valve motion to robot model on rviz
           (publish-hose-grasp-motion :coords (elt hose-grasp-params 0) :update? t)
           )
          ((eq (send smachine :state) :selecting-exec-for-hose-connect)
           ;; update latest hose-connect params
           (setq hose-connect-params
                 (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish valve motion to robot model on rviz
           (publish-hose-connect-motion :coords (elt hose-connect-params 0) :update? t)
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-grasp)
           ;; update latest drill-grasp params
           (setq drill-grasp-params
                 (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish dirll motion to robot model on rviz
           (publish-drill-grasp-motion :coords (elt drill-grasp-params 0) :update? t) ;;drill-type
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-put)
           ;; update latest drill-put params
           (setq drill-put-params
                 (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish dirll motion to robot model on rviz
           (publish-drill-put-motion :coords (elt drill-put-params 0) :update? t)
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-button)
           ;; update latest drill-button params
           (setq drill-button-params
                 (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish dirll motion to robot model on rviz
           (publish-drill-button-motion :coords (elt drill-button-params 0) :update? t)
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-wall)
           ;; update latest drill-button params
           ;; (setq drill-wall-params
           ;;       (list (get-primitive-marker-pose :frame-id "BODY")))
           ;; publish dirll motion to robot model on rviz
           (publish-drill-wall-motion :coords-list drill-wall-target-coords :update? t)
           )
          ((eq (send smachine :state) :selecting-exec-for-door)
           ;; update latest door params
           (let* ((dim (get-primitive-marker-dimensions)))
             (setq door-params
                   (list (get-primitive-marker-pose :frame-id "BODY") (float-vector (elt dim 0) (elt dim 1) (elt dim 2)))))
           ;; publish door motion to robot model on rviz
           (publish-door-motion :coords (elt door-params 0) :knob-size (elt door-params 1) :update? t)
           (send smachine :go-to :selecting-exec-for-door))
          )
    (send req :response)
    )
  )

;; rviz interface ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:go-pos-command-from-rviz-interface
   (req)
   ;; go to :executing-go-pos from :initial directory
   (when (eq (send smachine :state) :initial)
     (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
       (send msg :type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
       (send msg :float_params 
             (coords-to-float-vector (ros::tf-pose->coords (send req :goal)) (length (send msg :float_params))))
       (send self :send-to-fc msg))
     (send smachine :go-to :executing-go-pos))
   (send req :response)
   )
  (:angle-vector-command-from-ocs
   (req)
   (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*ANGLE_VECTOR_SEND*)
     (send msg :float_params 
           (angle-vector-to-float-vector (send (send req :angle_vector) :data) (send (send req :interpolation_time) :data) (length (send msg :float_params))))
     (send self :send-to-fc msg))
   (send req :response)
   )
  (:cancel-motion-now
    (req)
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*CANCEL_MOTION_NOW*)
     (send self :show-text "cancel command send")
     (send self :change-state-before-exec)
     (send self :send-to-fc msg))
    (send req :response)
    )
  (:skip-recog
   (req)
   ;todo something more precise
    (cond
     ((eq (send smachine :state) :selecting-region-for-door)
      (send smachine :go-to! :selecting-exec-for-door))
     ((eq (send smachine :state) :selecting-region-for-valve)
      (send smachine :go-to! :selecting-exec-for-valve))
     ((eq (send smachine :state) :selecting-region-for-debri)
      (send smachine :go-to! :selecting-exec-for-debri))
     ((eq (send smachine :state) :selecting-region-for-hose-grasp)
      (send smachine :go-to! :selecting-exec-for-hose-grasp))
     ((eq (send smachine :state) :selecting-region-for-hose-connect)
      (send smachine :go-to! :selecting-exec-for-hose-connect))
     ((eq (send smachine :state) :selecting-region-for-drill-grasp)
      (send smachine :go-to! :selecting-exec-for-drill-grasp)
      (erase-all-primitive-marker)
      (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t))
     ((eq (send smachine :state) :selecting-region-for-drill-put)
      (send smachine :go-to! :selecting-exec-for-drill-put))
     ;; ((eq (send smachine :state) :selecting-region-for-drill-wall)
     ;;  (send smachine :go-to! :selecting-exec-for-drill-wall)) ;; because of not using interactive-marker
     ((eq (send smachine :state) :selecting-region-for-drill-button)
      (send *robot* :angle-vector (get-potentio-vector-from-joint-states))
      (init-takenoko-drill)
      (set-drill-default-coords)
      (let ((coords (send (send (send (send *robot* :body_lk) :copy-worldcoords) :inverse-transformation) :transform (send *drill* :copy-worldcoords))))
              (send self :show-text 
              (format nil "Recognize drill-button. pos:~A rpy:~A" (send coords :worldpos) (reverse (car (send coords :rpy-angle)))) :recognize)
        ;; store result
        (setq drill-button-params (list coords))
        ;; publish object marker
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
        (set-primitive-marker-pose coords "BODY")
        (move-robot-model-to-current-pos)
        ;(send smachine :go-to :selecting-exec-for-drill-button)
	)
      (send smachine :go-to! :selecting-exec-for-drill-button))
     )
    (send self :show-text "SKIP RECOG! SET AND MOVE MARKER MANUALLY")
    (send req :response)
   )
  (:enable-head-overwrite
   (req)
   (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*ENABLE_OVERWRITE_HEAD_JOINT*)
     (send self :show-text "enable head overwrite")
     (send self :send-to-fc msg))
   (send req :response)
   )
  (:disable-head-overwrite
   (req)
   (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*DISABLE_OVERWRITE_HEAD_JOINT*)
     (send self :show-text "disable head overwrite")
     (send self :send-to-fc msg))
   (send req :response)
   )
  (:change-state-before-exec
    ()
    (cond
     ((eq (send smachine :state) :executing-door) (send smachine :go-to :selecting-exec-for-door))
     ((eq (send smachine :state) :executing-valve) (send smachine :go-to :selecting-exec-for-valve))
     ((eq (send smachine :state) :executing-debri) (send smachine :go-to :selecting-exec-for-debri))
     ((eq (send smachine :state) :executing-hose-grasp) (send smachine :go-to :selecting-exec-for-hose-grasp))
     ((eq (send smachine :state) :executing-hose-connect) (send smachine :go-to :selecting-exec-for-hose-connect))
     ((eq (send smachine :state) :executing-drill-grasp) (send smachine :go-to :selecting-exec-for-drill-grasp))
     ((eq (send smachine :state) :executing-drill-put) (send smachine :go-to :selecting-exec-for-drill-put))
     ((eq (send smachine :state) :executing-drill-wall) (send smachine :go-to :selecting-exec-for-drill-wall))
     ((eq (send smachine :state) :executing-drill-button) (send smachine :go-to :selecting-exec-for-drill-button))
     )
    )
  )

(defmethod ocs-executive-app
  (:uint8-request-callback (req)
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type (send req :type))
      (send self :send-to-fc msg))
    (send self :show-text "uint8 command send" :success)
    (send req :response)
    )
  )

;; Driving callback
(defmethod ocs-executive-app
  (:drive-handle-angle-callback (msg)
    (when (send smachine :state? :vehicle)
      (setq latest-handle-angle (send msg :data))
      (send self :relay-drive-message-to-fc)))
  (:drive-accell-angle-callback (msg)
    (when (send smachine :state? :vehicle)
      (setq latest-accell-angle (send msg :data))
      (send self :relay-drive-message-to-fc)))
  (:drive-brake-angle-callback (msg)
    (when (send smachine :state? :vehicle)
      (setq latest-brake-angle (send msg :data))
      (send self :relay-drive-message-to-fc)))
  (:relay-drive-message-to-fc ()
    (let ((msg (instance OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRIVE_COMMAND*)
      (when (and latest-handle-angle latest-accell-angle latest-brake-angle)
        (setf (elt (send msg :float_params) 0) latest-handle-angle)
        (setf (elt (send msg :float_params) 1) latest-accell-angle)
        (setf (elt (send msg :float_params) 2) latest-brake-angle)
        (setq latest-handle-angle nil)
        (setq latest-accell-angle nil)
        (setq latest-brake-angle nil))
      (send self :send-to-fc msg)))
  )

;; generate *robot*
(generate-robot "HRP2JSKNT" :generate-ri? nil)

;; advertise
(ros::advertise "/ocs/rviz/visualization/ocs_executive_message"
  jsk_rviz_plugins::OverlayText)
;; TODO
(let ((drill-type-string (ros::get-param "~drill_type" "takenoko")))
  (cond
   ((equal drill-type-string "gun") (setq *drill-type* :gun))
   ((equal drill-type-string "takenoko") (setq *drill-type* :takenoko))
   )
  )
(setq *app* (instance ocs-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))

(ros::advertise "/ocs_to_fc_low_speed/input" drc_com_common::OCS2FCSmall)
(ros::advertise "/jsk_model_marker_interface/hrp2/reset_joint_states_and_root" sensor_msgs::JointState 1)
(ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
(ros::advertise "/midi_config_player/set_feedback" sensor_msgs::JoyFeedbackArray)
(ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)
(ros::advertise "/transformable_interactive_server/set_control_relative_pose" geometry_msgs::Pose 1)
(ros::advertise "/drill_pose_list" geometry_msgs::PoseArray 1)

;; registering services for button UIs
(ros::advertise-service "drive" std_srvs::Empty #'send *app* :drive-button-callback)
(ros::advertise-service "valve" std_srvs::Empty #'send *app* :valve-button-callback)
(ros::advertise-service "door" std_srvs::Empty #'send *app* :door-button-callback)
(ros::advertise-service "debri" std_srvs::Empty #'send *app* :debri-button-callback)
(ros::advertise-service "hose_grasp" std_srvs::Empty #'send *app* :hose-grasp-button-callback)
(ros::advertise-service "hose_connect" std_srvs::Empty #'send *app* :hose-connect-button-callback)
(ros::advertise-service "drill_grasp" std_srvs::Empty #'send *app* :drill-grasp-button-callback)
(ros::advertise-service "drill_put" std_srvs::Empty #'send *app* :drill-put-button-callback)
(ros::advertise-service "drill_wall" std_srvs::Empty #'send *app* :drill-wall-button-callback)
(ros::advertise-service "drill_button" std_srvs::Empty #'send *app* :drill-button-button-callback)
(ros::advertise-service "go_pos" std_srvs::Empty #'send *app* :go-pos-button-callback)
(ros::advertise-service "look_at" std_srvs::Empty #'send *app* :look-at-button-callback)
(ros::advertise-service "look_around" std_srvs::Empty #'send *app* :look-around-button-callback)
(ros::advertise-service "exec" std_srvs::Empty #'send *app* :exec-button-callback)
(ros::advertise-service "cancel" std_srvs::Empty #'send *app* :cancel-button-callback)
(ros::advertise-service "reflesh" std_srvs::Empty #'send *app* :reflesh-button-callback)
(ros::advertise-service "go_pos_command_from_rviz_interface" drc_task_common::GoPosCommand #'send *app* :go-pos-command-from-rviz-interface)
(ros::advertise-service "uint8_command" drc_task_common::Uint8Request #'send *app* :uint8-request-callback)
(ros::advertise-service "angle_vector_command_from_ocs" drc_task_common::AngleVectorCommand #'send *app* :angle-vector-command-from-ocs)
(ros::advertise-service "cancel_motion_now" std_srvs::Empty #'send *app* :cancel-motion-now)
(ros::advertise-service "skip_recog" std_srvs::Empty #'send *app* :skip-recog)
(ros::advertise-service "enable_head_overwrite" std_srvs::Empty #'send *app* :enable-head-overwrite)
(ros::advertise-service "disable_head_overwrite" std_srvs::Empty #'send *app* :disable-head-overwrite)

;; Drive joystick controller topics
(ros::subscribe "/ocs/drive/operation/handle_cmd" std_msgs::Float64 #'send *app* :drive-handle-angle-callback)
(ros::subscribe "/ocs/drive/operation/accel_cmd" std_msgs::Float64 #'send *app* :drive-accell-angle-callback)
(ros::subscribe "/ocs/drive/operation/brake_cmd" std_msgs::Float64 #'send *app* :drive-brake-angle-callback)
;; subscribing image_view2 ROI stuff
(ros::subscribe "/ocs/communication/image_rect_color/screenrectangle/scaled" geometry_msgs::PolygonStamped
  #'send *app* :rectangle-callback)
(ros::subscribe "/ocs/communication/panorama_image/screenrectangle/scaled" geometry_msgs::PolygonStamped
  #'send *app* :panorama-rectangle-callback)
(ros::subscribe "/ocs/communication/image_rect_color/line/scaled" geometry_msgs::PolygonStamped
  #'send *app* :line-callback)
(ros::subscribe "/ocs/communication/image_rect_color/poly/scaled" geometry_msgs::PolygonStamped
  #'send *app* :polygon-callback)
;; (ros::subscribe "/ocs/communication/image_rect_color/foreground_rect/scaled" geometry_msgs::PolygonStamped
;;   #'send *app* :foreground-rect-callback)
;; (ros::subscribe "/ocs/communication/image_rect_color/background_rect/scaled" geometry_msgs::PolygonStamped
;;   #'send *app* :background-rect-callback)
(ros::subscribe "/ocs_from_fc_low_speed/output" drc_com_common::FC2OCSSmall
  #'send (send *app* :fc2ocs-message-handler) :root-callback)
(send *app* :subscribe-foreground-backggrond-topics 
      (list (list "/ocs/communication/image_rect_color/foreground_rect/scaled" geometry_msgs::PolygonStamped)
            (list "/ocs/communication/image_rect_color/background_rect/scaled" geometry_msgs::PolygonStamped)))
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-states-callback)

(ros::rate 10)
(ros::spin)
