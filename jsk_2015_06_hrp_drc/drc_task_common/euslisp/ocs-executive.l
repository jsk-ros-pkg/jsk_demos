#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::load-ros-manifest "drc_task_common")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/state-machine.l")
(load "package://drc_task_common/euslisp/generate-valve-motion.l")
(load "package://drc_task_common/euslisp/generate-debri-motion.l")
(load "package://drc_task_common/euslisp/generate-hose-motion.l")
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
                door-params
                debri-params ;; (list coords radius height)
                hose-grasp-params ;; (list coords)
                hose-connect-params ;; (list coords)
                foreground-bacground-sync-obj
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
          (list drc_com_common::FC2OCSSmall::*LOOK_AT_POS*
                self :look-at-recognition-finished-callback)
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
                self :hose-connect-exec-finished-callback))
    (setq smachine (make-ocs-state-machine))
    (send smachine :register-initial-hook self :initial-callback)
    (send smachine :go-to :initial)     ;force to move to initial state
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
    (send self :show-text-on-image "")
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
       ((eq mode :none)
        (call-empty-service (format nil "~A/none_mode" node)))
       (t
        (error "unknown mode: ~A" mode)
        ))
      ))
  (:show-text-on-image (text)
    (let ((msg (instance image_view2::ImageMarker2 :init)))
      (send msg :type image_view2::ImageMarker2::*TEXT*)
      (send msg :position :x 0)
      (send msg :position :y 0.1)
      (send msg :scale 0.02)
      (send msg :filled 2)
      (send msg :left_up_origin t)
      (send msg :ratio_scale t)
      (send msg :text text)
      (ros::publish "/image_rect/image_marker" msg)
      )
    )
  (:go-pos-button-callback (req)
    (when (send smachine :next? :selecting-region-for-go-pos)
      ;; force to use rectangle mode
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text-on-image "Select rectangle region to go")
      (send smachine :go-to :selecting-region-for-go-pos)
      )
    (send req :response))
  (:door-button-callback (req)
    (when (send smachine :next? :selecting-region-for-door)
      (send self :set-image-view2-mode :grabcut-rect)
      (send self :show-text-on-image "Select rectangle two region for door")
      (send smachine :go-to :selecting-region-for-door))
    (send req :response))
  (:debri-button-callback (req)
    (when (send smachine :next? :selecting-region-for-debri)
      (send self :set-image-view2-mode :line)
      (send self :show-text-on-image "Select line for debri")
      (send smachine :go-to :selecting-region-for-debri))
    (send req :response))
  (:hose-grasp-button-callback (req)
    (when (send smachine :next? :selecting-region-for-hose-grasp)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text-on-image "Select rectangle for hose")
      (send smachine :go-to :selecting-region-for-hose-grasp))
    (send req :response))
  (:hose-connect-button-callback (req)
    (when (send smachine :next? :selecting-region-for-hose-connect)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text-on-image "Select rectangle for hose")
      (send smachine :go-to :selecting-region-for-hose-connect))
    (send req :response))
  (:valve-button-callback (req)
    (when (send smachine :next? :selecting-region-for-valve)
      (send self :set-image-view2-mode :rectangle)
      (send self :show-text-on-image "Select rectangle region of valve")
      (send smachine :go-to :selecting-region-for-valve))
    (send req :response))
  (:look-at-button-callback (req)
    (when (send smachine :next? :selecting-region-for-look-at)
      ;; force to use rectangle mode
      (send self :set-image-view2-mode :rectangle)
      (send self :set-image-view2-mode :rectangle :panorama-view) ;we support panorama view
      (send self :show-text-on-image "Select rectangle region to look")
      (send smachine :go-to :selecting-region-for-look-at)
      )
    (send req :response))
  (:send-to-fc (msg)
    (ros::publish "/ocs_to_fc_low_speed/input" msg))
  (:rectangle-callback-for-go-pos-region (msg)
    (send self :show-text-on-image "Wait for go-pos recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*GO_POS_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-go-pos-goal)))
  (:panorama-rectangle-callback (msg)
    (send self :show-text-on-image "Wait for go-pos recognition")
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send fc-msg :image_type drc_com_common::OCS2FCSmall::*PANORAMA_VIEW*)
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-look-at-point)))
  (:rectangle-callback-for-look-at-region (msg)
    (send self :show-text-on-image "Wait for look-at recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-look-at-point)))
  (:rectangle-callback-for-valve-region (msg)
    (send self :show-text-on-image "Wait for valve recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*VALVE_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-valve-pose)))
  (:double-rectangles-callback-for-door-recognition (foreground background)
    ;;(print (list foreground background))
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DOOR_REGION*)
      (send fc-msg :image_roi_info (double-rect-polygon-stamped-to-integer-vector
                                    foreground background (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-door-pose)))
  (:rectangle-callback-for-hose-grasp-region (msg)
    (send self :show-text-on-image "Wait for hose-grasp recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*HOSE_GRASP_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-hose-grasp-pose)))
  (:rectangle-callback-for-hose-connect-region (msg)
    (send self :show-text-on-image "Wait for hose-connect recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*HOSE_CONNECT_REGION*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-hose-connect-pose)))
  (:foreground-background-callback (foreground background)
    (cond
     ((eq (send smachine :state) :selecting-region-for-door)
      (send self :double-rectangles-callback-for-door-recognition foreground background))
     )
    )
  (:line-callback-for-debri-line (msg)
    (send self :show-text-on-image "Wait for debri recognition")
    ;; send to FC to recognize pos
    (let ((fc-msg (instance drc_com_common::OCS2FCSmall :init)))
      (send fc-msg :type drc_com_common::OCS2FCSmall::*DEBRI_LINE*)
      (send fc-msg :image_roi_info (rect-polygon-stamped-to-integer-vector
                                    msg (length (send fc-msg :image_roi_info))))
      (send self :send-to-fc fc-msg)
      (send smachine :go-to :recognizing-debri-pose)))
  (:line-callback (msg)            ;msg = geometry_msgs/PolygonStamped
    (cond
     ((eq (send smachine :state) :selecting-region-for-debri)
      (send self :line-callback-for-debri-line msg))))
  (:rectangle-callback (msg)            ;msg = geometry_msgs/PolygonStamped
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
     ))
  (:cancel-button-callback (req)
    (cond
     ((eq (send smachine :state) :selecting-region-for-go-pos)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-look-at)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-door)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-valve)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-debri)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-hose-grasp)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :selecting-exec-for-hose-connect)
      (send smachine :go-to :initial))
     ;; TODO : cancel recognition from button
     ((eq (send smachine :state) :recognizing-look-at-point)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-valve-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-door-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-debri-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-go-pos-goal)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-hose-grasp-pose)
      (send smachine :go-to :initial))
     ((eq (send smachine :state) :recognizing-hose-connect-pose)
      (send smachine :go-to :initial))
     )
    ;; clear text message
    (send req :response))
  (:exec-go-pos-button-callback ()
    ;; update latest go-pos coords
    (setq go-pos-coords (send *tf* :lookup-transform "hrp2_marker_root" "ground" (ros::time 0)))
    (setq go-pos-coords (send go-pos-coords :transformation (make-coords) :local))
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*GO_POS_EXEC*)
      (send msg :float_params 
            (coords-to-float-vector 
             go-pos-coords (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-go-pos)))
  (:exec-look-at-button-callback ()
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*LOOK_AT_EXEC*)
      (send msg :float_params (pos-to-float-vector look-at-pos (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-look-at)))
  (:exec-valve-button-callback ()
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
    (let ((msg (instance drc_com_common::OCS2FCSmall :init)))
      (send msg :type drc_com_common::OCS2FCSmall::*DOOR_EXEC*)
      (send msg :float_params (box-to-float-vector 
                               (car door-params) (cadr door-params)
                               (length (send msg :float_params))))
      (send self :send-to-fc msg)
      (send smachine :go-to :executing-door)))
  (:exec-debri-button-callback ()
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
  (:exec-button-callback (req)
    (cond
     ((eq (send smachine :state) :selecting-exec-for-go-pos)
      (send self :exec-go-pos-button-callback))
     ((eq (send smachine :state) :selecting-exec-for-look-at)
      (send self :exec-look-at-button-callback))                          
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
     )
    (send req :response))
  (:debri-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-debri-pose)
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords r h) (float-vector-to-cylinder params)
         (send self :show-text-on-image (format nil "debri: ~A" params))
         ;; publish torus pose
         (erase-all-primitive-marker)
         (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
         (set-primitive-marker-dimensions :radius r :z h)
         (set-primitive-marker-pose coords "BODY")
         (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
         (apply-primitive-dimensions-to-midi-device)
         (send self :move-robot-model-to-current-pos)
         ;; store result
         (setq debri-params (list coords r h))
         ;; publish valve motion to robot model on rviz
         (publish-debri-motion :coords coords :radius r :height h :handle-ratio 0.9 :update? t)
         (send smachine :go-to :selecting-exec-for-debri)))))
  (:door-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-door-pose)
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords size) (float-vector-to-box params)
         (send self :show-text-on-image (format nil "door: ~A" params))
         ;; TODO: publish BoundingBox to visualize
         (setq door-params (list coords size))
         (send smachine :go-to :selecting-exec-for-door)))
      ))
  (:look-at-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-look-at-point)
      (let ((params (send msg :params)))
        (send self :show-text-on-image 
              (format nil "look-at ~f ~f ~f" 
                      (* 0.001 (elt params 0))
                      (* 0.001 (elt params 1))
                      (* 0.001 (elt params 2))))
        (setq look-at-pos (float-vector (elt params 0)
                                        (elt params 1)
                                        (elt params 2)))
        (send smachine :go-to :selecting-exec-for-look-at))))
  (:go-pos-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-go-pos-goal)
      (let ((params (send msg :params)))
        ;; show go pos pose in image
        (setq go-pos-coords (make-coords :pos (float-vector (elt params 0)
                                                            (elt params 1)
                                                            (elt params 2))
                                         :rpy (float-vector (elt params 3)
                                                            (elt params 4)
                                                            (elt params 5))))
        ;; move robot model on rviz
        (send self :move-robot-model go-pos-coords "ground")
        (send self :show-text-on-image 
              (format nil "go-pos ~f ~f ~f" 
                      (* 0.001 (elt params 0))
                      (* 0.001 (elt params 1))
                      (rad2deg (elt params 3))
                      ))
        (send smachine :go-to :selecting-exec-for-go-pos))))
  (:debri-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-debri)
      (send smachine :go-to :initial)))
  (:valve-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-valve)
      (send smachine :go-to :initial)))
  (:door-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-door)
      (send smachine :go-to :initial)))
  (:go-pos-finished-callback (msg)
    (when (eq (send smachine :state) :executing-go-pos)
      (send smachine :go-to :initial)))
  (:hose-grasp-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-hose-grasp)
      (send smachine :go-to :initial)))
  (:hose-connect-exec-finished-callback (msg)
    (when (eq (send smachine :state) :executing-hose-connect)
      (send smachine :go-to :initial)))
  (:valve-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-valve-pose)
      ;; show it
      (let ((params (send msg :params)))
        (multiple-value-bind
         (coords radius) (float-vector-to-torus params)
         (send self :show-text-on-image 
               (format nil "valve ~A ~A ~f" 
                       (send coords :worldpos)
                       (reverse (car (send coords :rpy-angle)))
                       radius))
         ;; publish torus pose
         (erase-all-primitive-marker)
         (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*TORUS* :name "torus1" :description "")
         (set-primitive-marker-dimensions :radius radius :small-radius (* 0.1 radius))
         (set-primitive-marker-pose coords "BODY")
         (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
         (apply-primitive-dimensions-to-midi-device)
         (send self :move-robot-model-to-current-pos)
         ;; store result
         (setq valve-params (list coords radius))
         ;; publish valve motion to robot model on rviz
         (publish-valve-motion :radius (cadr valve-params) :coords (car valve-params) :update? t)
         ))
      (send smachine :go-to :selecting-exec-for-valve)))
  (:hose-grasp-recognition-finished-callback (msg)
    (when (eq (send smachine :state) :recognizing-hose-grasp-pose)
      (let* ((params (send msg :params))
             ;; TODO : receive all hose information
            (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rpy (list 0 pi 0))))
        (send self :show-text-on-image 
              (format nil "hose pos ~f ~f ~f" 
                      (* 0.001 (elt params 0))
                      (* 0.001 (elt params 1))
                      (* 0.001 (elt params 2))))
        ;; store result
        (setq hose-grasp-params (list coords))
        ;; publish torus pose
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
        (set-primitive-marker-dimensions :radius 30 :z 120)
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (send self :move-robot-model-to-current-pos)
        ;; publish hose motion to robot model on rviz
        (publish-hose-grasp-motion :coords (elt hose-grasp-params 0) :update? t)
        (send smachine :go-to :selecting-exec-for-hose-grasp))
      ))
  (:hose-connect-recognition-finished-callback (msg)
    ;; TODO : receive all hose information
    (when (eq (send smachine :state) :recognizing-hose-connect-pose)
      (let* ((params (send msg :params))
             (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rot (list 0 pi/2 0))))
        (send self :show-text-on-image 
              (format nil "hose pos ~f ~f ~f" 
                      (* 0.001 (elt params 0))
                      (* 0.001 (elt params 1))
                      (* 0.001 (elt params 2))))
        ;; store result
        (setq hose-connect-params (list coords))
        ;; publish torus pose
        (erase-all-primitive-marker)
        (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER* :name "cylinder1" :description "")
        (set-primitive-marker-dimensions :radius 30 :z 120)
        (set-primitive-marker-pose coords "BODY")
        (set-primitive-marker-color :r 1.0 :g 1.0 :b 0.0 :a 0.6)
        (apply-primitive-dimensions-to-midi-device)
        (send self :move-robot-model-to-current-pos)
        ;; publish hose motion to robot model on rviz
        (publish-hose-connect-motion :coords (elt hose-connect-params 0) :update? t)
        (send smachine :go-to :selecting-exec-for-hose-connect))
      ))
  (:look-at-finished-callback (msg)
    (when (eq (send smachine :state) :executing-look-at)
      (send smachine :go-to :initial)))
  (:go-pos-command-from-rviz-interface (req)
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
           ))
    (send req :response)
    )
  (:move-robot-model
    (coords frame-id)
    (let* ((header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id frame-id))
           (msg (instance geometry_msgs::PoseStamped :init :header header :pose (ros::coords->tf-pose coords))))
      (ros::publish "/urdf_control_marker/set_pose" msg)
      ))
  (:move-robot-model-to-current-pos
    ()
    (let* ((base-frame "/odom_on_ground") (target-frame "/ground")
           (header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id base-frame))
           (msg (instance geometry_msgs::PoseStamped :init :header header))
           coords)
      (setq coords (send (send *tf* :lookup-transform target-frame base-frame (ros::time 0)) :inverse-transformation))
      (send msg :pose (ros::coords->tf-pose coords))
      (ros::publish "/urdf_control_marker/set_pose" msg)
      ))
  )


;; advertise
(ros::advertise "/image_rect/image_marker" image_view2::ImageMarker2)
(setq *app* (instance ocs-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))

(ros::advertise "/ocs_to_fc_low_speed/input" drc_com_common::OCS2FCSmall)
(ros::advertise "/jsk_model_marker_interface/hrp2/reset_joint_states_and_root" sensor_msgs::JointState 1)
(ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
(ros::advertise "/midi_config_player/set_feedback" sensor_msgs::JoyFeedbackArray)
(ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)

;; registering services for button UIs
(ros::advertise-service "go_pos" std_srvs::Empty #'send *app* :go-pos-button-callback)
(ros::advertise-service "valve" std_srvs::Empty #'send *app* :valve-button-callback)
(ros::advertise-service "door" std_srvs::Empty #'send *app* :door-button-callback)
(ros::advertise-service "debri" std_srvs::Empty #'send *app* :debri-button-callback)
(ros::advertise-service "hose_grasp" std_srvs::Empty #'send *app* :hose-grasp-button-callback)
(ros::advertise-service "hose_connect" std_srvs::Empty #'send *app* :hose-connect-button-callback)
(ros::advertise-service "look_at" std_srvs::Empty #'send *app* :look-at-button-callback)
(ros::advertise-service "exec" std_srvs::Empty #'send *app* :exec-button-callback)
(ros::advertise-service "cancel" std_srvs::Empty #'send *app* :cancel-button-callback)
(ros::advertise-service "reflesh" std_srvs::Empty #'send *app* :reflesh-button-callback)
(ros::advertise-service "go_pos_command_from_rviz_interface" drc_task_common::GoPosCommand #'send *app* :go-pos-command-from-rviz-interface)

;; subscribing image_view2 ROI stuff
(ros::subscribe "/ocs/communication/image_rect_color/screenrectangle/scaled" geometry_msgs::PolygonStamped
  #'send *app* :rectangle-callback)
(ros::subscribe "/ocs/communication/panorama_image/screenrectangle/scaled" geometry_msgs::PolygonStamped
  #'send *app* :panorama-rectangle-callback)
(ros::subscribe "/ocs/communication/image_rect_color/line/scaled" geometry_msgs::PolygonStamped
  #'send *app* :line-callback)
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
