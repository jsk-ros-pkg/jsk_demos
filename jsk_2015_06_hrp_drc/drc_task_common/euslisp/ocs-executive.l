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
(ros::roseus-add-srvs "drc_task_common")
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
                drill-wall-params
                drill-wall-target-coords ;; list of coords
                drill-stand-coords-list ;; list of (list symbol coords), base is odom_on_ground
                drill-stand-coords-index ;; referenced stand coords
                drill-button-params ;; (list coords)
		drill-rotate-params
		drill-grasp-relative-coords
                foreground-bacground-sync-obj
                arm-mode ;; one of (:rarm-only :rarm-preferred :larm-preferred :larm-only)
                valve-grasp-mode ;; one of (:edge-grasp-only :edge-grasp-preferred :center-grasp-preferrned :center-grasp-only)
                overwrite-stand-coords? ;; one of (:force :auto nil)
                drill-rotate-num
                door-name
                force-overwrite-door-arm ;; one of (:force :auto)
                ;; For driving taskot
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
          (list drc_com_common::FC2OCSSmall::*DRILL_ROTATE_POSE*
                self :drill-rotate-recognition-finished-callback)
          (list drc_com_common::FC2OCSSmall::*DRILL_ROTATE_FINISHED*
                self :drill-rotate-exec-finished-callback)
          (list drc_com_common::FC2OCSSmall::*LOOK_AROUND_FINISHED*
                self :look-around-finished-callback)
          (list drc_com_common::FC2OCSSmall::*RECOGNITION_FAILED*
                self :recognition-failed-callback)
          (list drc_com_common::FC2OCSSmall::*RVIZ_MENU_FINISHED*
                self :rviz-finished-callback)
          (list drc_com_common::FC2OCSSmall::*ANGLE_VECTOR_FINISHED*
                self :angle-vector-finished-callback)
          (list drc_com_common::FC2OCSSmall::*REACH_UNTIL_TOUCH_FINISHED*
                self :reach-until-touch-finished-callback))
    (setq smachine (make-ocs-state-machine))
    (send smachine :register-initial-hook self :initial-callback)
    (send smachine :go-to :initial) ;; force to move to initial state
    (setq arm-mode :rarm-preferred)
    (setq overwrite-stand-coords? :auto)
    (setq drill-wall-target-coords nil)
    (setq valve-grasp-mode :edge-grasp-preferred)
    (setq door-push/pull :push)
    (setq door-name :drc-lasvegas)
    (setq force-overwrite-door-arm :force)
    (send self :show-text "initialized" :success)
    (setq drill-rotate-num 2)
    (setq drill-stand-coords-index 0)
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
    (when (send smachine :next? :recognizing-drill-grasp-pose)
      (send self :set-image-view2-mode :none)
      (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
        (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_GRASP_REGION*) ;Should have another constant
        (send self :show-text "recognizing drill grasp coords" :select)
        (send self :send-to-fc fc-msg))
      (send smachine :go-to :recognizing-drill-grasp-pose))
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
  (:drill-rotate-button-callback (req)
    (when (send smachine :next? :selecting-region-for-drill-rotate)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text "Select rectangle region for drill-rotate" :select)
      (send smachine :go-to :selecting-region-for-drill-rotate))
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
     ((eq (send smachine :state) :selecting-region-for-drill-rotate)
      (send self :rectangle-callback-for-drill-rotate-region msg))
     (t
       (send self :show-text "rectangle selected but no mean" :warn))
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
  ;; Deprecated
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
  (:rectangle-callback-for-drill-rotate-region (msg)
    (send self :show-text "Wait for drill-rotate recognition" :wait)
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DRILL_ROTATE_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-drill-rotate-pose)))
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
     ((eq (send smachine :state) :selecting-region-for-drill-rotate)
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
     ((eq (send smachine :state) :recognizing-drill-rotate-pose)
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
     ((eq (send smachine :state) :selecting-exec-for-drill-rotate)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send smachine :go-to :initial))
     ;; there is no selecting-region state for look-at and look-around
     ;; from selecting-region state to initial state ;;;;
     ((eq (send smachine :state) :executing-valve)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-door)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-debri)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-hose-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-hose-connect)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-drill-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-drill-put)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-drill-wall)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-drill-button)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-drill-rotate)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-go-pos)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-look-at)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :executing-look-around)
      (send smachine :go-to :initial))
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
      (let (valve-grasp-mode-idx)
        (cond ((not (boundp '*valve-grasp-mode*))
               (setq valve-grasp-mode-idx drc_com_common::OCS2FCSmall::*UNDECIDED*))
              ((equal *valve-grasp-mode* :center-grasp)
               (setq valve-grasp-mode-idx drc_com_common::OCS2FCSmall::*VALVE_CENTER_GRASP*))
              ((equal *valve-grasp-mode* :edge-grasp)
               (setq valve-grasp-mode-idx drc_com_common::OCS2FCSmall::*VALVE_EDGE_GRASP*))
              (t
               (setq valve-grasp-mode-idx drc_com_common::OCS2FCSmall::*UNDECIDED*))
              )
        (send msg :float_params
              (valve-params-to-float-vector valve-params valve-grasp-mode-idx (length (send msg :float_params)))))
      (cond ((not (boundp '*valve-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *valve-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *valve-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send msg :stand_coords
            (coords-to-float-vector (get-robot-model-current-pos) (length (send msg :stand_coords))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-valve)))
  (:exec-door-button-callback ()
    (send self :show-text "Execute door motion" :execute)
    ;; update latest door params
    (let* ((dim (get-primitive-marker-dimensions)))
      (setq door-params
            (list (get-primitive-marker-pose :frame-id "BODY") (float-vector (elt dim 0) (elt dim 1) (elt dim 2)))))
    ;; send door params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DOOR_EXEC*)
      (let ((door-push/pull-idx
             (cond ((equal door-push/pull :push) drc_com_common::OCS2FCSmall::*DOOR_PUSH*)
                   ((equal door-push/pull :pull) drc_com_common::OCS2FCSmall::*DOOR_PULL*)
                   (t drc_com_common::OCS2FCSmall::*UNDECIDED*)))
            (door-name-idx
             (cond ((equal door-name :drc-box2) drc_com_common::OCS2FCSmall::*DOOR_DRC_BOX2*)
                   ((equal door-name :drc-test) drc_com_common::OCS2FCSmall::*DOOR_DRC_TEST*)
                   ((equal door-name :drc-test-mirror) drc_com_common::OCS2FCSmall::*DOOR_DRC_TEST_MIRROR*)
                   ((equal door-name :drc-sagami) drc_com_common::OCS2FCSmall::*DOOR_DRC_SAGAMI*)
                   ((equal door-name :drc-sagami-mirror) drc_com_common::OCS2FCSmall::*DOOR_DRC_SAGAMI_MIRROR*)
                   ((equal door-name :drc-lasvegas) drc_com_common::OCS2FCSmall::*DOOR_DRC_LASVEGAS*)
                   ((equal door-name :drc-lasvegas-mirror) drc_com_common::OCS2FCSmall::*DOOR_DRC_LASVEGAS_MIRROR*)
                   (t drc_com_common::OCS2FCSmall::*UNDECIDED*)))
            )
        (send msg :float_params
              (door-params-to-float-vector door-params door-push/pull-idx door-name-idx (length (send msg :float_params)))))
      (cond ((not (boundp '*door-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *door-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *door-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send msg :stand_coords
            (coords-to-float-vector (get-robot-model-current-pos) (length (send msg :stand_coords))))
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
          (list (get-primitive-marker-pose :frame-id "BODY" :name "drill")))
    (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
    (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
    ;; (apply-primitive-dimensions-to-midi-device)
    (set-primitive-marker-pose (elt drill-grasp-params 0) "BODY" :name "cylinder1")
    (set-primitive-marker-dimensions :radius 400 :z 1 :name "cylinder1")
    (set-focus-marker-name "drill")
    ;; send drill-grasp params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_GRASP_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-grasp-params 0) (length (send msg :float_params))))
      (send msg :image_type drill-rotate-num)
      (send msg :stand_coords
            (coords-to-float-vector (get-robot-model-current-pos) (length (send msg :stand_coords))))
      (cond ((not (boundp '*drill-main-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *drill-main-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *drill-main-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-grasp)))
  (:exec-drill-put-button-callback ()
    (send self :show-text "Execute drill-put motion" :execute)
    ;; update latest drill-put params
    (setq drill-put-params
          (list (get-primitive-marker-pose :frame-id "BODY" :name "cylinder1")))
    ;; send drill-put params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_PUT_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-put-params 0) (length (send msg :float_params))))
      (send msg :stand_coords
            (coords-to-float-vector (get-robot-model-current-pos) (length (send msg :stand_coords))))
      (cond ((not (boundp '*drill-main-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *drill-main-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *drill-main-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-put)))
  (:exec-drill-wall-button-callback ()
    (send self :show-text "Execute drill-wall motion" :execute)
    ;; do something magical!!
    ;; send drill-wall params to fc
    (cond ((get-marker-existence "drill_wall")
           (setq drill-wall-params
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "drill_wall"))))
          ((get-marker-existence "box1")
           (setq drill-wall-params
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "box1")))
           (let* ((dims (get-primitive-marker-dimensions :name "box1"))
                  (x (/ (elt dims 0) 2))
                  (y (/ (elt dims 1) 2))
                  )
             (setq drill-wall-target-coords (list
                                             (make-coords :pos (float-vector (+ x 30) y 0))
                                             (make-coords :pos (float-vector (- x) y 0))
                                             (make-coords :pos (float-vector (- x) (- y) 0))
                                             (make-coords :pos (float-vector x (- y) 0))
                                             (make-coords :pos (float-vector x (+ y 30) 0))
                                             ))
             )
           )
          (t (send self :show-text "no marker set! insert box or recog wakk" :warn))
          )
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_WALL_EXEC*)
      (send msg :float_params (coords-set-to-float-vector (get-coords-list-with-center (elt drill-wall-params 0) drill-wall-target-coords) (length (send msg :float_params))))
      (send msg :stand_coords
            (coords-to-float-vector (get-robot-model-current-pos) (length (send msg :stand_coords))))
      (cond ((not (boundp '*drill-main-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *drill-main-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *drill-main-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-wall)))
  (:exec-drill-button-button-callback ()
    (send self :show-text "Execute drill-button motion" :execute)
    ;; update latest drill-button params
    (setq drill-button-params
          (list (get-primitive-marker-pose :frame-id "BODY" :name "drill")))
    ;; do something magical!!
    ;; send drill-button params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_BUTTON_EXEC*)
      (send msg :float_params (coords-to-float-vector (elt drill-button-params 0) (length (send msg :float_params))))
      (cond ((not (boundp '*drill-main-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *drill-main-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *drill-main-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-button)))
  (:exec-drill-rotate-button-callback ()
    (send self :show-text "Execute drill-rotate motion" :execute)
    ;; update latest drill-rotate params
    (setq drill-rotate-params
          (list (get-primitive-marker-pose :frame-id "BODY" :name "drill") (get-primitive-marker-pose :frame-id "BODY" :name "cylinder1")))
    ;; do something magical!!
    ;; send drill-rotate params to fc
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DRILL_ROTATE_EXEC*)
      (send msg :float_params (coords-set-to-float-vector drill-rotate-params (length (send msg :float_params))))
      (send msg :image_type (min 1 drill-rotate-num))
      (cond ((not (boundp '*drill-main-arm*))
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            ((equal *drill-main-arm* :larm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
            ((equal *drill-main-arm* :rarm)
             (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
            (t
             (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
            )
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-drill-rotate)))

  (:exec-go-pos-button-callback ()
    ;; update latest go-pos coords
    (send self :show-text "Execute GO POS" :execute)
    (setq go-pos-coords (send *tf* :lookup-transform "robot_marker_root" "ground" (ros::time 0)))
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
     ((eq (send smachine :state) :selecting-exec-for-drill-rotate)
      (send self :exec-drill-rotate-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send self :exec-go-pos-button-callback))
     (t (send self :show-text "exec selected but no mean" :warn))
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
         (publish-valve-motion (cadr valve-params) (car valve-params)
                               :arm (send self :get-arm-side) :overwrite-arm-side? (send self :is-arm-overwrite)
                               :stand-coords (make-coords) :overwrite-stand-coords? :force ;; overwrite stand-coords forcibly in valve motion
                               :grasp-center? (send self :is-valve-grasp-center) :overwrite-grasp-mode? (send self :is-valve-grasp-overwrite)
                               :update? t)
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
         (publish-door-motion
          :door-name door-name :door-coords (elt door-params 0)
          :arm (send self :get-arm-side) :overwrite-arm-side? (send self :is-door-arm-overwrite)
          :stand-coords (make-coords) :overwrite-stand-coords? :force ;; overwrite stand-coords forcibly in door motion
          :push/pull door-push/pull
          :update? t)
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
          (set-primitive-marker-control-relative-pose (make-coords :pos (float-vector 0 0 97) :rpy (float-vector 0 0 0))) ;;Todo more precise pose
          ))
        ;(set-primitive-marker-dimensions :radius 30 :z 300)
        (set-primitive-marker-pose coords "BODY")
        ;; (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (move-robot-model-to-current-pos)
        (set-focus-marker-name "drill")
        ;; publish motion to robot model on rviz
        (setq *drill-main-arm* (send self :get-arm-side))
	(publish-drill-grasp-motion :coords (elt drill-grasp-params 0) :update? t :stand-coords (make-coords) :overwrite-stand-coords? :force :drill-rotate-num drill-rotate-num)
        (let ((index (position-if #'(lambda (x) (equal (elt x 0) :grasp)) drill-stand-coords-list)))
          (if index
              (progn
                (setf (elt drill-stand-coords-list index) (list :grasp (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))))
                (setq drill-stand-coords-index index))
            (progn
              (push-back (list :grasp (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))) drill-stand-coords-list)
              (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
              )))
	(if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	    (send self :show-text 
		  "IK FAIL")
          (send self :show-text (format nil "IK SUCCESS diff-deg ~Adeg" *drill-diff-angle*))
          )
 ;todo drill-type
        
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
        (set-primitive-marker-dimensions :radius 400 :z 1 :name "cylinder1")
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
        (setq *drill-main-arm* (send self :get-arm-side))
        (publish-drill-put-motion :coords (elt drill-put-params 0) :update? t :stand-coords (make-coords) :overwrite-stand-coords? t)
	(if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	    (send self :show-text 
		  "IK FAIL")
          (send self :show-text "IK SUCCESS")
	  )
        (send smachine :go-to :selecting-exec-for-drill-put))
      ))
  (:drill-wall-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-wall)
      (let ((params (send msg :params)))
        (let ((coords-set (float-vector-to-coords-set params)) center-coords)
          (send self :show-text 
                (format nil "Recognize wall coords") :recognize)
          (publish-coords-list coords-set "BODY")
          ;; marker 
          ;; (erase-all-primitive-marker)
          (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "drill_wall" :description "")
          (set-primitive-marker-dimensions :radius 300 :z 30) ;; show plane as circle
          (setq center-coords (get-center-coords coords-set))
          (set-primitive-marker-pose center-coords "BODY" :name "drill_wall")
          (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
          (apply-primitive-dimensions-to-midi-device)
          (move-robot-model-to-current-pos)
          ;; end marker
          (setq drill-wall-target-coords (get-relative-coords-list center-coords coords-set))
	  (setq *drill-main-arm* (send self :get-arm-side))
          (publish-drill-wall-motion :coords-list coords-set :update? t :stand-coords (make-coords) :overwrite-stand-coords? :force :draw? nil)
          (let ((index (position-if #'(lambda (x) (equal (elt x 0) :wall)) drill-stand-coords-list)))
            (if index
                (progn
                  (setf (elt drill-stand-coords-list index) (list :wall (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))))
                  (setq drill-stand-coords-index index))
              (progn
                (push-back (list :wall (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))) drill-stand-coords-list)
                (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
                )))
	  (if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	      (send self :show-text 
		  "IK FAIL")
	    )
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
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
	(set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-control-relative-pose (make-coords :pos (float-vector 0 0 95) :rpy (float-vector 0 0 0))) ;;Todo more precise pose for button
        ;;(set-primitive-marker-pose coords "BODY") ;; comment out because pose is not correct
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
	(setq *drill-main-arm* (send self :get-arm-side))
        (publish-drill-button-motion :coords (elt drill-button-params 0) :update? t)
	(if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	    (send self :show-text 
		  "IK FAIL")
          (send self :show-text "IK SUCCESS")
	  )
        (send smachine :go-to :selecting-exec-for-drill-button))))
 (:drill-rotate-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-drill-rotate-pose)
      (let* ((params (send msg :params))
             (coords (float-vector-to-coords params)))
        (send self :show-text 
              (format nil "Recognize drill-rotate. pos:~A rpy:~A" (send coords :worldpos) (reverse (car (send coords :rpy-angle)))) :recognize)
        ;; store result
        (setq drill-rotate-params (list coords (get-primitive-marker-pose :frame-id "BODY" :name "drill")))
        ;; publish object marker
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
        (move-robot-model-to-current-pos)
        ;; publish motion to robot model on rviz
	(setq *drill-main-arm* (send self :get-arm-side))
	(publish-drill-rotate-motion :drill-coords (elt drill-rotate-params 0) :plane-coords (elt drill-rotate-params 1) :update? t)
	(if (or (not (boundp '*drill-motion-in*)) (not *drill-motion-in*))
	    (send self :show-text 
		  "IK FAIL")
          (send self :show-text "IK SUCCESS")
          )
        (send smachine :go-to :selecting-exec-for-drill-rotate))))
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
    (when (eq (send smachine :state) :selecting-exec-for-drill-grasp) (return-from :recognition-failed-callback nil)) ;; skip selected!
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
    (when (eq (send smachine :state) :executing-valve)
      (send self :show-text "Finished valve motion" :success)
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
      (send smachine :go-to :initial)
      (let* ((params (send msg :params))
             (coords-set (float-vector-to-coords-set params)))
        (send self :show-text 
              (format nil "drill grasp succeeded. Left Rotation :~A" (elt (send (elt coords-set 2) :worldpos) 0)))
        ;; store result
	(set-primitive-marker-pose (elt coords-set 0) "BODY" :name "drill")
	(setq drill-grasp-relative-coords (elt coords-set 1))
        (when (< (abs (elt (send (elt coords-set 2) :worldpos) 0)) 20)
          (push-back (list :button (convert-coords-frame (make-coords :pos (float-vector -500 0 0)))) drill-stand-coords-list)
          (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
          (send self :show-text "GOPOS AUTO TO BUTTON FAISE")
          )
        )
      )
    )
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
      (if (send msg :success)
          (progn (send self :show-text "Finished drill-button motion with success" :success)
                 (send self :show-text "PUSH AUTO GO-POS BUTTON!")
                 (let ((index (position-if #'(lambda (x) (equal (elt x 0) :wall)) drill-stand-coords-list)))
                   (if index
                       (progn
                         (setf (elt drill-stand-coords-list index) (list :wall (convert-coords-frame (make-coords :rpy (list (deg2rad -90) 0 0)))))
                         (setq drill-stand-coords-index index))
                     (progn
                       (push-back (list :wall (convert-coords-frame (make-coords :rpy (list (deg2rad -90) 0 0)))) drill-stand-coords-list)
                       (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
                       )))
                 )
        (send self :show-text "Finished drill-button motion but failed!":fatal))
      (send smachine :go-to :initial)))
  (:drill-rotate-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-drill-rotate)
      (send self :show-text "Finished drill-rotate motion" :success)
      (send smachine :go-to :initial)
      (let* ((params (send msg :params))
             (coords-set (float-vector-to-coords-set params)))
        (send self :show-text 
              (format nil "drill grasp succeeded. Left Rotation :~A" (elt (send (elt coords-set 2) :worldpos) 0)))
        ;; store result
        (when (< (abs (elt (send (elt coords-set 2) :worldpos) 0)) 20)
          (push-back (list :button (convert-coords-frame (make-coords :pos (float-vector -500 0 0)))) drill-stand-coords-list)
          (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
          (send self :show-text "GOPOS AUTO TO BUTTON FAISE")
          )
        (setq drill-grasp-relative-coords (elt coords-set 1))
	(set-primitive-marker-pose (elt coords-set 0) "BODY" :name "drill"))
      )
    )
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
      (send self :show-text "Finished send-angle-vector" :success)
      (send-angle-vector-finished-service)
   )
  (:reach-until-touch-finished-callback (msg)
      (send self :show-text "Finished reach-until-touch" :success)
   )
  )

;; reflesh callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod ocs-executive-app
  (:reflesh-button-callback (req)
    (send self :show-text "Start to replay robot motion")
    (cond ((eq (send smachine :state) :selecting-exec-for-valve)
           ;; update latest valve params
           (setq valve-params
                 (list (get-primitive-marker-pose :frame-id "BODY")
                       (elt (get-primitive-marker-dimensions) 3)))
           ;; publish valve motion to robot model on rviz
           (publish-valve-motion (cadr valve-params) (car valve-params)
                                 :arm (send self :get-arm-side) :overwrite-arm-side? (send self :is-arm-overwrite)
                                 :stand-coords (get-robot-model-current-pos) :overwrite-stand-coords? overwrite-stand-coords?
                                 :grasp-center? (send self :is-valve-grasp-center) :overwrite-grasp-mode? (send self :is-valve-grasp-overwrite)
                                 :update? t))
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
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "drill")))
           ;; publish drill motion to robot model on rviz
	   (setq *drill-main-arm* (send self :get-arm-side))
           (publish-drill-grasp-motion :coords (elt drill-grasp-params 0) :update? t :stand-coords (get-robot-model-current-pos) :overwrite-stand-coords? overwrite-stand-coords? :drill-rotate-num drill-rotate-num) ;;drill-type
           (let ((index (position-if #'(lambda (x) (equal (elt x 0) :grasp)) drill-stand-coords-list)))
             (if index
                 (progn
                   (setf (elt drill-stand-coords-list index) (list :grasp (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))))
                   (setq drill-stand-coords-index index))
               (progn
                 (push-back (list :grasp (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))) drill-stand-coords-list)
                 (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
                 )))
           
	   (if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	       (send self :show-text 
		  "IK FAIL")
             (send self :show-text (format nil "IK SUCCESS diff-deg ~Adeg" *drill-diff-angle*))
             )
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-put)
           ;; update latest drill-put params
           (setq drill-put-params
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "cylinder1")))
           ;; publish drill motion to robot model on rviz
	   (setq *drill-main-arm* (send self :get-arm-side))
           (publish-drill-put-motion :coords (elt drill-put-params 0) :update? t :stand-coords (get-robot-model-current-pos) :overwrite-stand-coords? overwrite-stand-coords?)
	   (if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	       (send self :show-text 
		     "IK FAIL")
	     )
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-button)
           ;; update latest drill-button params
           (setq drill-button-params
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "drill")))
           ;; publish drill motion to robot model on rviz
	   (setq *drill-main-arm* (send self :get-arm-side))
           (publish-drill-button-motion :coords (elt drill-button-params 0) :update? t)
	   (if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	       (send self :show-text 
		     "IK FAIL")
	     )
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-rotate)
           ;; update latest drill-rotate params
           (setq drill-rotate-params
                 (list (get-primitive-marker-pose :frame-id "BODY" :name "drill") (get-primitive-marker-pose :frame-id "BODY" :name "cylinder1")))
           ;; publish drill motion to robot model on rviz
	   (setq *drill-main-arm* (send self :get-arm-side))
           (publish-drill-rotate-motion :drill-coords (elt drill-rotate-params 0) :plane-coords (elt drill-rotate-params 1) :update? t)
           
	   (if (or (not (boundp '*drill-motion-in*)) (not *drill-motion-in*))
	       (send self :show-text 
		     "IK FAIL")
	     )
           )
          ((eq (send smachine :state) :selecting-exec-for-drill-wall)
           ;; update latest drill-button params
	   (setq *drill-main-arm* (send self :get-arm-side))
           (cond ((get-marker-existence "drill_wall")
                  (setq drill-wall-params
                        (list (get-primitive-marker-pose :frame-id "BODY" :name "drill_wall"))))
                 ((get-marker-existence "box1")
                  (setq drill-wall-params
                        (list (get-primitive-marker-pose :frame-id "BODY" :name "box1")))
                  (let* ((dims (get-primitive-marker-dimensions :name "box1"))
                         (x (/ (elt dims 0) 2))
                         (y (/ (elt dims 1) 2))
                         )
                    (setq drill-wall-target-coords (list
                                                    (make-coords :pos (float-vector (+ x 30) y 0))
                                                    (make-coords :pos (float-vector (- x) y 0))
                                                    (make-coords :pos (float-vector (- x) (- y) 0))
                                                    (make-coords :pos (float-vector x (- y) 0))
                                                    (make-coords :pos (float-vector x (+ y 30) 0))
                                                    ))
                    )
                  )
                 (t (send self :show-text "no marker set! insert box or recog wakk" :warn))
                 )
           ;; publish drill motion to robot model on rviz
           (publish-drill-wall-motion :coords-list (get-coords-list-with-center (elt drill-wall-params 0) drill-wall-target-coords) :update? t :stand-coords (get-robot-model-current-pos) :overwrite-stand-coords? overwrite-stand-coords?)
           (let ((index (position-if #'(lambda (x) (equal (elt x 0) :wall)) drill-stand-coords-list)))
             (if index
                 (progn
                   (setf (elt drill-stand-coords-list index) (list :wall (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))))
                   (setq drill-stand-coords-index index))
              (progn
                (push-back (list :wall (convert-coords-frame (if (boundp '*drill-stand-coords-forced*) *drill-stand-coords-forced* (make-coords)))) drill-stand-coords-list)
                (setq drill-stand-coords-index (- (length drill-stand-coords-list) 1))
                )))
	   (if (or (not (boundp '*drill-motion*)) (not *drill-motion*))
	       (send self :show-text 
		     "IK FAIL")
	    )
           )
          ((eq (send smachine :state) :selecting-exec-for-door)
           ;; update latest door params
           (let* ((dim (get-primitive-marker-dimensions)))
             (setq door-params
                   (list (get-primitive-marker-pose :frame-id "BODY") (float-vector (elt dim 0) (elt dim 1) (elt dim 2)))))
           ;; publish door motion to robot model on rviz
           (publish-door-motion
            :door-name door-name :door-coords (elt door-params 0)
            :arm (send self :get-arm-side) :overwrite-arm-side? (send self :is-door-arm-overwrite)
            :stand-coords (get-robot-model-current-pos) :overwrite-stand-coords? overwrite-stand-coords?
            :push/pull door-push/pull
            :update? t)
           (send smachine :go-to :selecting-exec-for-door))
          (t (send self :show-text "refresh selected but no mean" :warn)
              (send req :response))
          )
    (send self :show-text "Finish to replay robot motion")
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
  (:angle-vector-command-from-ocs ;; no state check nor transition
   (req)
   (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*ANGLE_VECTOR_SEND*)
     (send msg :float_params
           (angle-vector-to-float-vector (send (send req :angle_vector) :data) (send (send req :interpolation_time) :data) (length (send msg :float_params))))
     (send self :send-to-fc msg))
   (send req :response)
   )
  (:reach-until-touch-from-ocs ;; no state check nor transition
   (req)
   (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
     (send msg :type drc_com_common::OCS2FCSmall::*REACH_UNTIL_TOUCH_EXEC*)
     (cond ((equal (send req :arm_side) drc_task_common::ReachUntilTouchCommandRequest::*LARM*)
            (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*))
           ((equal (send req :arm_side) drc_task_common::ReachUntilTouchCommandRequest::*RARM*)
            (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*))
           (t
            (send msg :arm_side drc_com_common::OCS2FCSmall::*UNDECIDED*))
           )
     (send msg :float_params
           (reach-until-touch-params-to-float-vector
            (ros::tf-point->pos (send req :direction))
            (send req :thre) (send req :max_dist)
            (send req :dist_step) (send req :time_step) (send req :rotation_axis)
            (length (send msg :float_params))))
     (send self :send-to-fc msg))
   (send req :response)
   )
  (:set-drill-coords
   (req)
   (when (not drill-grasp-relative-coords)
     (set-drill-environment :draw? t)
     (set-drill-default-coords)
     (setq *drill-main-arm* (send self :get-arm-side))
     (setq drill-grasp-relative-coords (send (send *robot* *drill-main-arm* :end-coords :copy-worldcoords) :transformation (send *drill* :copy-coords)))
     )
   (when (and drill-grasp-relative-coords (boundp '*drill-main-arm*))
     (when (not (get-marker-existence "drill"))
       (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
       )
     (if (equal *drill-main-arm* :rarm) (set-primitive-marker-pose drill-grasp-relative-coords "rarm_end_coords" :name "drill") (set-primitive-marker-pose drill-grasp-relative-coords "larm_end_coords" :name "drill")
         )
     )
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
     ((or (eq (send smachine :state) :selecting-region-for-drill-grasp)
          (eq (send smachine :state) :recognizing-drill-grasp-pose))
      (send smachine :go-to! :selecting-exec-for-drill-grasp)
      )
     ((eq (send smachine :state) :selecting-region-for-drill-put)
      (send smachine :go-to! :selecting-exec-for-drill-put))
     ;; ((eq (send smachine :state) :selecting-region-for-drill-wall)
     ;;  (send smachine :go-to! :selecting-exec-for-drill-wall)) ;; because of not using interactive-marker
     ((eq (send smachine :state) :selecting-region-for-drill-button)
      ;; todo drill-type
      (when (not (boundp '*drill*))
        (if t
            (set-drill-environment :drill-type :takenoko)
          (set-drill-environment :drill-type :gun)
          )
        (send *robot* :angle-vector (get-potentio-vector-from-joint-states))
        (send *robot* :fix-leg-to-coords (make-coords))
        (set-drill-default-coords)
        (let ((coords (send (send (send (send *robot* :body_lk) :copy-worldcoords) :inverse-transformation) :transform (send *drill* :copy-worldcoords))))
          (send self :show-text
                (format nil "Recognize drill-button. pos:~A rpy:~A" (send coords :worldpos) (reverse (car (send coords :rpy-angle)))) :recognize)
          ;; store result
          (setq drill-button-params (list coords))
          ;; publish object marker
          (erase-all-primitive-marker)
          (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
          (set-primitive-marker-control-relative-pose (make-coords :pos (float-vector 0 0 97) :rpy (float-vector 0 0 0))) ;;Todo more precise pose for button
          (move-robot-model-to-current-pos)
          (set-primitive-marker-pose coords "BODY")
                                        ;(send smachine :go-to :selecting-exec-for-drill-button)
          )
        )
      (send smachine :go-to! :selecting-exec-for-drill-button))
     ((eq (send smachine :state) :selecting-region-for-drill-wall)
      (send smachine :go-to! :selecting-exec-for-drill-wall))
     ((eq (send smachine :state) :selecting-region-for-drill-rotate)
      (send smachine :go-to! :selecting-exec-for-drill-rotate))
     (t (send self :show-text "skip recog selected but no mean" :warn))
     )
    (send self :show-text "skip recog. set and move marker manually.")
    (send req :response)
    )
  (:set-pitch-head-angle-callback (req)
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_PITCH_ANGLE*)
      (setf (elt (send msg :float_params) 0) (rad2deg (send req :value)))
      (send self :show-text "Moving head pitch angle")
      (send self :send-to-fc msg)
      (send smachine :go-to! :executing-look-at)
      (send req :response)))
  (:set-yaw-head-angle-callback (req)
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_YAW_ANGLE*)
      (setf (elt (send msg :float_params) 0) (rad2deg (send req :value)))
      (send self :show-text "Moving head yaw angle")
      (send self :send-to-fc msg)
      (send smachine :go-to! :executing-look-at)
      (send req :response)))
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
     ((eq (send smachine :state) :executing-drill-rotate) (send smachine :go-to :selecting-exec-for-drill-rotate))
     )
    )
  )

;; select arm
(defmethod ocs-executive-app
  (:larm-only-checked-callback
   (req)
   (send self :show-text "larm only is selected" :success)
   (setq arm-mode :larm-only)
   (send req :response)
   )
  (:larm-preferred-checked-callback
   (req)
   (send self :show-text "larm preferred is selected" :success)
   (setq arm-mode :larm-preferred)
   (send req :response)
   )
  (:rarm-preferred-checked-callback
   (req)
   (send self :show-text "rarm preferred is selected" :success)
   (setq arm-mode :rarm-preferred)
   (send req :response)
   )
  (:rarm-only-checked-callback
   (req)
   (send self :show-text "rarm only is selected" :success)
   (setq arm-mode :rarm-only)
   (send req :response)
   )
  (:is-arm-overwrite
   ()
   (or (equal arm-mode :larm-preferred) (equal arm-mode :rarm-preferred)))
  (:is-door-arm-overwrite
   ()
   (cond ((equal force-overwrite-door-arm :force)
          :force)
         ((or (equal arm-mode :larm-preferred) (equal arm-mode :rarm-preferred))
          t)
         (t
          nil))
   )
  (:get-arm-side
   ()
   (cond ((or (equal arm-mode :larm-preferred) (equal arm-mode :larm-only)) :larm)
         ((or (equal arm-mode :rarm-preferred) (equal arm-mode :rarm-only)) :rarm)))
  )

;; optional button
(defmethod ocs-executive-app
  ;; stand-coords overwrite
  (:force-overwrite-stand-coords-callback
   (req)
   (setq overwrite-stand-coords? :force)
   (send self :show-text "force overwrite stand-coords overwrite" :success)
   (send req :response)
   )
  (:auto-overwrite-stand-coords-callback
   (req)
   (setq overwrite-stand-coords? :auto)
   (send self :show-text "auto overwrite stand-coords overwrite" :success)
   (send req :response)
   )
  (:no-overwrite-stand-coords-callback
   (req)
   (setq overwrite-stand-coords? nil)
   (send self :show-text "no overwrite  stand-coords overwrite" :success)
   (send req :response)
   )
  ;; head overwrite
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
  ;; valve grasp
  (:valve-edge-grasp-only-callback
   (req)
   (setq valve-grasp-mode :edge-grasp-only)
   (send self :show-text "valve edge grasp only" :success)
   (send req :response)
   )
  (:valve-edge-grasp-preferred-callback
   (req)
   (setq valve-grasp-mode :edge-grasp-preferred)
   (send self :show-text "valve edge grasp preferred" :success)
   (send req :response)
   )
  (:valve-center-grasp-preferred-callback
   (req)
   (setq valve-grasp-mode :center-grasp-preferred)
   (send self :show-text "valve center grasp preferred" :success)
   (send req :response)
   )
  (:valve-center-grasp-only-callback
   (req)
   (setq valve-grasp-mode :center-grasp-only)
   (send self :show-text "valve center grasp only" :success)
   (send req :response)
   )
  (:is-valve-grasp-center
   ()
   (or (equal valve-grasp-mode :center-grasp-only) (equal valve-grasp-mode :center-grasp-preferred)))
  (:is-valve-grasp-overwrite
   ()
   (or (equal valve-grasp-mode :edge-grasp-preferred) (equal valve-grasp-mode :center-grasp-preferred)))
  ;; door direction
  (:door-direction-push-callback
   (req)
   (setq door-push/pull :push)
   (send self :show-text "door direction push" :success)
   (send req :response)
   )
  (:door-direction-pull-callback
   (req)
   (setq door-push/pull :pull)
   (send self :show-text "door direction pull" :success)
   (send req :response)
   )
  ;; door name
  (:door-name-drc-box2-callback
   (req)
   (setq door-name :drc-box2)
   (send self :show-text "door name drc-box2" :success)
   (send req :response)
   )
  (:door-name-drc-test-callback
   (req)
   (setq door-name :drc-test)
   (send self :show-text "door name drc-test" :success)
   (send req :response)
   )
  (:door-name-drc-test-mirror-callback
   (req)
   (setq door-name :drc-test-mirror)
   (send self :show-text "door name drc-test-mirror" :success)
   (send req :response)
   )
  (:door-name-drc-sagami-callback
   (req)
   (setq door-name :drc-sagami)
   (send self :show-text "door name drc-sagami" :success)
   (send req :response)
   )
  (:door-name-drc-sagami-mirror-callback
   (req)
   (setq door-name :drc-sagami-mirror)
   (send self :show-text "door name drc-sagami-mirror" :success)
   (send req :response)
   )
  (:door-name-drc-lasvegas-callback
   (req)
   (setq door-name :drc-lasvegas)
   (send self :show-text "door name drc-lasvegas" :success)
   (send req :response)
   )
  (:door-name-drc-lasvegas-mirror-callback
   (req)
   (setq door-name :drc-lasvegas-mirror)
   (send self :show-text "door name drc-lasvegas-mirror" :success)
   (send req :response)
   )
  (:drill-rotate-zero-callback
   (req)
   (setq drill-rotate-num 0)
   (send req :response)
   )
  (:drill-rotate-one-callback
   (req)
   (setq drill-rotate-num 1)
   (send req :response)
   )
  (:drill-rotate-two-callback
   (req)
   (setq drill-rotate-num 2)
   (send req :response)
   )
  (:drill-rotate-three-callback
   (req)
   (setq drill-rotate-num 3)
   (send req :response)
   )
  (:drill-auto-go-pos-callback
   (req)
   (send smachine :go-to! :selecting-exec-for-go-pos)
   (if drill-stand-coords-list
       (progn
         (move-robot-model (elt (elt drill-stand-coords-list drill-stand-coords-index) 1) "/odom_on_ground")
         (send self :show-text (format nil "GOPOS for ~A Push EXEC" (elt (elt drill-stand-coords-list drill-stand-coords-index) 0)))
         (setq drill-stand-coords-index (mod (+ drill-stand-coords-index 1) (length drill-stand-coords-list)))
         )
     (send self :show-text "Please firstly find drill and recog")
     )
   (ros::ros-info (format nil "drill auto ~A" drill-stand-coords-list))
   (send req :response)
   )
  ;; door arm overwrite
  (:force-overwrite-door-arm-callback
   (req)
   (setq force-overwrite-door-arm :force)
   (send self :show-text "force overwrite door arm" :success)
   (send req :response)
   )
  (:auto-overwrite-door-arm-callback
   (req)
   (setq force-overwrite-door-arm :auto)
   (send self :show-text "auto overwrite door arm" :success)
   (send req :response)
   )
  )

(defmethod ocs-executive-app
  (:uint8-request-callback (req)
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type (send req :type))
      (if (equal (send self :get-arm-side) :rarm)
          (send msg :arm_side drc_com_common::OCS2FCSmall::*RARM*)
        (send msg :arm_side drc_com_common::OCS2FCSmall::*LARM*)
        )
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

(defmethod ocs-executive-app
  (:set-door-detect-auto-callback
   (req)
   (send req :response)
   )
  (:set-door-detect-semi-auto-callback
   (req)
   (send req :response)
   )
  (:set-valve-detect-auto-callback
   (req)
   (send req :response)
   )
  (:set-valve-detect-semi-auto-callback
   (req)
   (send req :response)
   )
  (:set-drill-detect-auto-callback
   (req)
   (send req :response)
   )
  (:set-drill-detect-semi-auto-callback
   (req)
   (send req :response)
   )
  )


;; generate *robot*
(setq *robot-name* (string-upcase (unix::getenv "ROBOT")))
(generate-robot *robot-name* :generate-ri? nil)
(set-drill-environment :draw? nil :generate-ri? nil) ;; for init param

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
(ros::advertise "/jsk_model_marker_interface/robot/reset_joint_states_and_root" sensor_msgs::JointState 1)
(ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
(ros::advertise "/midi_config_player/set_feedback" sensor_msgs::JoyFeedbackArray)
(ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)
(ros::advertise "/transformable_interactive_server/set_control_relative_pose" geometry_msgs::Pose 1)
(ros::advertise "/drill_pose_list" geometry_msgs::PoseArray 1)
(ros::advertise "/robotsound" sound_play::SoundRequest 5)
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
(ros::advertise-service "drill_rotate" std_srvs::Empty #'send *app* :drill-rotate-button-callback)
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
(ros::advertise-service "set_pitch_angle" drc_task_common::SetValue #'send *app* :set-pitch-head-angle-callback)
(ros::advertise-service "set_yaw_angle" drc_task_common::SetValue #'send *app* :set-yaw-head-angle-callback)
(ros::advertise-service "larm_only_checked" std_srvs::Empty #'send *app* :larm-only-checked-callback)
(ros::advertise-service "larm_preferred_checked" std_srvs::Empty #'send *app* :larm-preferred-checked-callback)
(ros::advertise-service "rarm_preferred_checked" std_srvs::Empty #'send *app* :rarm-preferred-checked-callback)
(ros::advertise-service "rarm_only_checked" std_srvs::Empty #'send *app* :rarm-only-checked-callback)
(ros::advertise-service "valve_edge_grasp_only" std_srvs::Empty #'send *app* :valve-edge-grasp-only-callback)
(ros::advertise-service "valve_edge_grasp_preferred" std_srvs::Empty #'send *app* :valve-edge-grasp-preferred-callback)
(ros::advertise-service "valve_center_grasp_preferred" std_srvs::Empty #'send *app* :valve-center-grasp-preferred-callback)
(ros::advertise-service "valve_center_grasp_only" std_srvs::Empty #'send *app* :valve-center-grasp-only-callback)
(ros::advertise-service "force_overwrite_stand_coords" std_srvs::Empty #'send *app* :force-overwrite-stand-coords-callback)
(ros::advertise-service "auto_overwrite_stand_coords" std_srvs::Empty #'send *app* :auto-overwrite-stand-coords-callback)
(ros::advertise-service "no_overwrite_stand_coords" std_srvs::Empty #'send *app* :no-overwrite-stand-coords-callback)
(ros::advertise-service "door_direction_push" std_srvs::Empty #'send *app* :door-direction-push-callback)
(ros::advertise-service "door_direction_pull" std_srvs::Empty #'send *app* :door-direction-pull-callback)
(ros::advertise-service "door_name_drc_box2" std_srvs::Empty #'send *app* :door-name-drc-box2-callback)
(ros::advertise-service "door_name_drc_test" std_srvs::Empty #'send *app* :door-name-drc-test-callback)
(ros::advertise-service "door_name_drc_test_mirror" std_srvs::Empty #'send *app* :door-name-drc-test-mirror-callback)
(ros::advertise-service "door_name_drc_sagami" std_srvs::Empty #'send *app* :door-name-drc-sagami-callback)
(ros::advertise-service "door_name_drc_sagami_mirror" std_srvs::Empty #'send *app* :door-name-drc-sagami-mirror-callback)
(ros::advertise-service "door_name_drc_lasvegas" std_srvs::Empty #'send *app* :door-name-drc-lasvegas-callback)
(ros::advertise-service "door_name_drc_lasvegas_mirror" std_srvs::Empty #'send *app* :door-name-drc-lasvegas-mirror-callback)
(ros::advertise-service "drill_rotate_zero" std_srvs::Empty #'send *app* :drill-rotate-zero-callback)
(ros::advertise-service "drill_rotate_one" std_srvs::Empty #'send *app* :drill-rotate-one-callback)
(ros::advertise-service "drill_rotate_two" std_srvs::Empty #'send *app* :drill-rotate-two-callback)
(ros::advertise-service "drill_rotate_three" std_srvs::Empty #'send *app* :drill-rotate-three-callback)
(ros::advertise-service "drill_auto_go_pos" std_srvs::Empty #'send *app* :drill-auto-go-pos-callback)

(ros::advertise-service "reach_until_touch_command_from_ocs" drc_task_common::ReachUntilTouchCommand #'send *app* :reach-until-touch-from-ocs)
(ros::advertise-service "set_drill_coords" std_srvs::Empty #'send *app* :set-drill-coords)
(ros::advertise-service "auto_door_detect" std_srvs::Empty #'send *app* :set-door-detect-auto-callback)
(ros::advertise-service "semi_auto_door_detect" std_srvs::Empty #'send *app* :set-door-detect-semi-auto-callback)
(ros::advertise-service "auto_valve_detect" std_srvs::Empty #'send *app* :set-valve-detect-auto-callback)
(ros::advertise-service "semi_auto_valve_detect" std_srvs::Empty #'send *app* :set-valve-detect-semi-auto-callback)
(ros::advertise-service "auto_drill_detect" std_srvs::Empty #'send *app* :set-drill-detect-auto-callback)
(ros::advertise-service "semi_auto_drill_detect" std_srvs::Empty #'send *app* :set-drill-detect-semi-auto-callback)
(ros::advertise-service "force_overwrite_door_arm" std_srvs::Empty #'send *app* :force-overwrite-door-arm-callback)
(ros::advertise-service "auto_overwrite_door_arm" std_srvs::Empty #'send *app* :auto-overwrite-door-arm-callback)

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
