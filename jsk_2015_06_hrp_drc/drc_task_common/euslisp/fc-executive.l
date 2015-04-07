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
(load "package://drc_task_common/euslisp/state-machine.l")
(load "package://drc_task_common/euslisp/generate-valve-motion.l")
(load "package://drc_task_common/euslisp/generate-debri-motion.l")
(load "package://drc_task_common/euslisp/generate-drill-motion.l")
(load "package://drc_task_common/euslisp/generate-hose-motion.l")
(load "package://drc_task_common/euslisp/generate-door-motion.l")

(defclass fc-executive-app
  :super propertied-object
  :slots (state go-pos-goal ocs2fc-message-handler smachine))

(defmethod fc-executive-app
  (:init ()
    ;; prepare state machine
    (setq smachine (make-fc-state-machine))
    (send self :register-timelimits smachine)
    (setq ocs2fc-message-handler (instance narrowband-message-handler :init))
    ;; register callbacks
    (send ocs2fc-message-handler :register-callbacks
          (list drc_com_common::OCS2FCSmall::*GO_POS_REGION*
                self :go-pos-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*GO_POS_EXEC*
                self :go-pos-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*
                self :look-at-region-command-callback)
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
                self :debri-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*HOSE_GRASP_REGION*
                self :hose-grasp-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*HOSE_GRASP_EXEC*
                self :hose-grasp-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*HOSE_CONNECT_REGION*
                self :hose-connect-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*HOSE_CONNECT_EXEC*
                self :hose-connect-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AROUND_EXEC*
                self :look-around-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_GRASP_REGION*
                self :drill-grasp-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_GRASP_EXEC*
                self :drill-grasp-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_PUT_REGION*
                self :drill-put-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_PUT_EXEC*
                self :drill-put-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_WALL_POINTS*
                self :drill-wall-points-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_WALL_EXEC*
                self :drill-wall-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_BUTTON_REGION*
                self :drill-button-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_BUTTON_EXEC*
                self :drill-button-exec-command-callback)
          ;; rviz plugins
          (list drc_com_common::OCS2FCSmall::*RESET_POSE*
                self :reset-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*RESET_MANIP_POSE*
                self :reset-manip-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*HAND_RESET_POSE*
                self :hand-reset-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*HAND_HOOK_POSE*
                self :hand-hook-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*HAND_HOOK_AFTER_POSE*
                self :hand-hook-after-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*HAND_GRASP_POSE*
                self :hand-grasp-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_START_ABC*
                self :hrpsys-start-abc-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_START_ST*
                self :hrpsys-start-st-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_START_IMP*
                self :hrpsys-start-imp-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ABC*
                self :hrpsys-stop-abc-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ST*
                self :hrpsys-stop-st-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_IMP*
                self :hrpsys-stop-imp-command-cb)
          (list drc_com_common::OCS2FCSmall::*ANGLE_VECTOR_SEND*
                self :angle-vector-send-command-callback)
          (list drc_com_common::OCS2FCSmall::*CANCEL_MOTION_NOW*
                self :cancel-motion-now-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRIVE_COMMAND*
                self :drive-command-callback))
    self)
  (:register-timelimits (sm)
    ;; sm is a instance of state machine
    (send (send smachine :node :recognizing-debri-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-hose-grasp-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-hose-connect-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-grasp-pose)
          :add-timelimit 15 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-put-pose)
          :add-timelimit 25 :initial ;; at present, too long!
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-button-pose)
          :add-timelimit 15 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-door-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-go-pos-location)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-valve-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-look-at-point)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-look-at-point-panorama)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-wall)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-button-pose)
          :add-timelimit 10 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    )
  (:report-recognition-failure-to-ocs ()
    (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
      (send msg :type drc_com_common::FC2OCSSmall::*RECOGNITION_FAILED*)
      (send self :send-to-ocs msg))
    )
  (:state-machine ()
    smachine)
  (:ocs2fc-message-handler () ocs2fc-message-handler)
  (:debri-line-command-callback (msg)
    ;;(when (send smachine :state? :initial)
      ;; enable snapshot
      (call-empty-service "/debri_recognition/snapshot/request")
      (ros::publish "/debri_recognition/input_line"
        (integer-vector-to-rect-polygon-stamped msg))
      (send smachine :go-to :recognizing-debri-pose)
      ;;)
  )
  (:hose-grasp-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/hose_grasp_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/hose_grasp_recognition/snapshot/request")
      (send smachine :go-to :recognizing-hose-grasp-pose)
      ))
  (:hose-connect-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/hose_connect_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/hose_connect_recognition/snapshot/request")
      (send smachine :go-to :recognizing-hose-connect-pose)
      ))
  (:drill-grasp-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/drill_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition/snapshot/request")
      (send smachine :go-to :recognizing-drill-grasp-pose)
      ))
  (:drill-put-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/drill_recognition_for_put/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition_for_put/snapshot/request")
      (send smachine :go-to :recognizing-drill-put-pose)
      ))
  (:drill-wall-points-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/drill_recognition_for_wall/input_poly"
        (integer-vector-to-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition_for_wall/snapshot/request")
      (send smachine :go-to :recognizing-drill-wall)
      ))
  (:drill-button-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/drill_recognition_for_button/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition_for_button/snapshot/request")
      (send smachine :go-to :recognizing-drill-button-pose)
      ))
  (:panorama-clear ()
    (call-empty-service "/panorama/panorama_processing/clear"))
  (:panorama-shutter ()
    (call-empty-service "/panorama/panorama_processing/shutter"))
  (:look-around-exec-command-callback (msg)
    (when (send smachine :state? :initial)           ;no need to check it?
      (let ((tm 1000))
        (send self :panorama-clear)
        (send smachine :go-to :executing-look-around)
        (send *robot* :angle-vector (send *ri* :state :reference-vector))
        (let ((initial-av (send *robot* :angle-vector)))
          (send *robot* :head :neck-p :joint-angle 0)
          (send *robot* :head :neck-y :joint-angle (- (send *robot* :head :neck-y :max-angle) 1))
          (send *ri* :angle-vector (send *robot* :angle-vector) tm)
          (send *ri* :wait-interpolation)
          (send self :panorama-shutter)
          
          (send *robot* :head :neck-y :joint-angle 0)
          (send *ri* :angle-vector (send *robot* :angle-vector) tm)
          (send *ri* :wait-interpolation)
          (send self :panorama-shutter)
          
          (send *robot* :head :neck-y :joint-angle (+ 1 (send *robot* :head :neck-y :min-angle)))
          (send *ri* :angle-vector (send *robot* :angle-vector) tm)
          (send *ri* :wait-interpolation)
          (send self :panorama-shutter)

          (send *robot* :angle-vector initial-av)
          (send *ri* :angle-vector (send *robot* :angle-vector) tm)
          (send *ri* :wait-interpolation))
        ;; send back done information
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*LOOK_AROUND_FINISHED*)
          (send self :send-to-ocs msg))
        (send smachine :go-to :initial)
        )                               ;done
      ))
  (:door-region-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind
       (foreground background) (integer-vector-to-double-rect-polygon-stamped msg)
       (ros::publish "/door_recognition/input_support_plane_rect"
         foreground)
       (ros::publish "/door_recognition/input_target_rect"
         background)
       ;; enable snapshot
       (call-empty-service "/door_recognition/snapshot/request")
       (send smachine :go-to :recognizing-door-pose)
       )))
  (:valve-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/valve_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/valve_recognition/snapshot/request")
      (send smachine :go-to :recognizing-valve-pose)
      ))
  (:go-pos-region-command-callback (msg)
    (when (send smachine :state? :initial)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      (ros::publish "/go_pos_recognition/input_rect"
        (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/go_pos_recognition/snapshot/request")
      (send smachine :go-to :recognizing-go-pos-location)
      ))
  (:debri-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind
       (coords r h) (float-vector-to-cylinder (send msg :float_params))
       (send smachine :go-to :executing-debri)
       (setq *cancel-motion-flag* nil)
       (catch 'exit
       (ros::ros-info "removing debri at ~A ~A ~A" coords r h)
       ;; send motion to real robot
       (send-debri-motion coords r h 0.9)
       ;; send finish signal to ocs
       (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
         (send msg :type drc_com_common::FC2OCSSmall::*DEBRI_FINISHED*)
         (send self :send-to-ocs msg))
       )
       (send smachine :go-to :initial)
       )))
  (:hose-grasp-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             ;; TODO : receive all hose information
             (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rpy (list 0 pi 0))))
        (ros::ros-info "grasping hose at ~A" coords)
        (send smachine :go-to :executing-hose-grasp)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (send-hose-grasp-motion coords)
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*HOSE_GRASP_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)
        ))
    )
  (:hose-connect-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             (coords (make-coords :pos (float-vector (elt params 0) (elt params 1) (elt params 2)) :rpy (list 0 pi/2 0))))
        (ros::ros-info "connecting hose at ~A" coords)
        (send smachine :go-to :executing-hose-connect)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (send-hose-connect-motion coords)
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*HOSE_CONNECT_FINISHED*)
          (send self :send-to-ocs msg))
        (send smachine :go-to :initial)
        )
        ))
    )
  (:drill-grasp-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             (coords (float-vector-to-coords params)))
        (ros::ros-info "grasp drill at ~A" coords)
        (send smachine :go-to :executing-drill-grasp)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (send-drill-grasp-motion coords)
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_GRASP_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)
        ))
    )
  (:drill-put-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             (coords (float-vector-to-coords params)))
        (ros::ros-info "put drill at ~A" coords)
        (send smachine :go-to :executing-drill-put)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (send-drill-put-motion coords)
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_PUT_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)        
        ))
    )
  (:drill-wall-exec-command-callback (msg)
    (when (send smachine :next? :executing-drill-wall)
      (send smachine :go-to :executing-drill-wall)
      (setq *cancel-motion-flag* nil)
      (catch 'exit
      (let ((coords-set (float-vector-to-coords-set (send msg :float_params))))
        (ros::ros-info "Target coords are: ~A" coords-set)
        (send-drill-wall-motion coords-set)
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_WALL_FINISHED*)
          (send self :send-to-ocs msg))
        )
      )
      (send smachine :go-to :initial)
      ))
  (:drill-button-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             (coords (float-vector-to-coords params)))
        (ros::ros-info "button drill at ~A" coords)
        (send smachine :go-to :executing-drill-button)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (send-drill-button-motion coords)
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_BUTTON_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)
        ))
    )
  (:door-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind
       (coords size)
       (float-vector-to-box (send msg :float_params))
       (send smachine :go-to :executing-door)
       (setq *cancel-motion-flag* nil)
       (catch 'exit
       (ros::ros-info "opening door at ~A ~A~%" coords size)
       ;; send motion to real robot
       (send-door-motion size coords)
       ;; send finish signal to ocs
       (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
         (send msg :type drc_com_common::FC2OCSSmall::*DOOR_FINISHED*)
         (send self :send-to-ocs msg))
       )
       (send smachine :go-to :initial)
       )))
  (:valve-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind 
       (coords radius) (float-vector-to-torus (send msg :float_params))
       (send smachine :go-to :executing-valve)
       (setq *cancel-motion-flag* nil)
       (catch 'exit
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
       )
       (send smachine :go-to :initial)       
       )))
  (:go-pos-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let ((goal-coords (float-vector-to-coords (send msg :float_params))))
        (send smachine :go-to :executing-go-pos)
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
      (send smachine :go-to :initial)))
  (:look-at-region-command-callback (msg)
    (when (send smachine :state? :initial)
      (cond
       ((eq (send msg :image_type) drc_com_common::OCS2FCSmall::*IMAGE_RECT*)
        ;; convert OCS2FCSmall into geometry_msgs/Polygon
        (ros::publish "/look_at_recognition/input_rect"
          (integer-vector-to-rect-polygon-stamped msg))
        ;; enable snapshot
        (call-empty-service "/look_at_recognition/snapshot/request")
        (send smachine :go-to :recognizing-look-at-point))
       ((eq (send msg :image_type) drc_com_common::OCS2FCSmall::*PANORAMA_VIEW*)
        (ros::publish "/panorama/panorama_processing/output/screenrectangle"
          (integer-vector-to-rect-polygon-stamped msg))
        ;; enable snapshot
        (call-empty-service "/panorama/snapshot/request")
        (send smachine :go-to :recognizing-look-at-point-panorama))
       (t
        (error "unknown image type: ~A" (send msg :image_type)))
       )))
  (:lookup-from-body-transform (header)
    (let ((trans (send *tf* :lookup-transform "BODY" 
                       (send header :frame_id) (send header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "BODY" 
                            (send header :frame_id) (ros::time 0))))
        trans))
  (:lookup-from-ground-transform (header)
    (let ((trans (send *tf* :lookup-transform "ground" 
                       (send header :frame_id) (send header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform "ground" 
                            (send header :frame_id) (ros::time 0))))
        trans))
  (:debri-recog-callback (msg)
    (when (send smachine :state? :recognizing-debri-pose)
      (let ((trans (send self :lookup-from-body-transform (send msg :header)))
            (coefficients (send msg :values)))
        ;; make coordinates from coefficients
        ;; coefficients := [cx, cy, cz, dx, dy, dz, r, h]
        (let ((pos (scale 1000 (float-vector (elt coefficients 0)
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
                                      from-body-coords (* (elt coefficients 6) 1000) (* (elt coefficients 7) 1000)
                                      (length (send fc-msg :params))))
                (send self :send-to-ocs fc-msg)
                (send smachine :go-to :initial))))))))
  (:door-recog-callback (msg)
    (when (send smachine :state? :recognizing-door-pose)
      (let ((trans (send self :lookup-from-body-transform (send msg :header))))
        (let ((box (elt (send msg :boxes) 0)))
          (let ((coords (ros::tf-pose->coords (send box :pose))))
            (let ((from-body-coords (send trans :transform coords)))
              (ros::ros-info "coords ~A" from-body-coords)
              (let ((fc-msg (instance drc_com_common::FC2OCSSmall :init)))
                (send fc-msg :type drc_com_common::FC2OCSSmall::*DOOR_POSE*)
                (send fc-msg :params (box-to-float-vector
                                      from-body-coords (ros::tf-point->pos (send box :dimensions))
                                      (length (send fc-msg :params))))
                (send self :send-to-ocs fc-msg)
                (send smachine :go-to :initial))))))))
  (:valve-recog-callback (msg)
    (when (send smachine :state? :recognizing-valve-pose)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose->coords (send msg :pose)))
             (from-body-coords (send trans :transform coords)))
        (let ((fc-msg (instance drc_com_common::FC2OCSSmall :init)))
          (send fc-msg :params (torus-to-float-vector 
                                from-body-coords (* (send msg :large_radius) 1000)
                                (length (send fc-msg :params))))
          (send fc-msg :type drc_com_common::FC2OCSSmall::*VALVE_POSE*)
          (send self :send-to-ocs fc-msg)
          (send smachine :go-to :initial)
          ))
      ))
  (:hose-grasp-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-hose-grasp-pose)
      ;; TODO : recognize hose
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
        ;; only position is important in the temporary recognition
        (let ((pos (float-vector (elt (send from-body-coords :worldpos) 0)
                                 (elt (send from-body-coords :worldpos) 1)
                                 (elt (send from-body-coords :worldpos) 2))))
          (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
            (send msg :type drc_com_common::FC2OCSSmall::*HOSE_GRASP_POSE*)
            (setf (elt (send msg :params) 0) (elt pos 0))
            (setf (elt (send msg :params) 1) (elt pos 1))
            (setf (elt (send msg :params) 2) (elt pos 2))
            (send self :send-to-ocs msg)
            (send smachine :go-to :initial)
            )))))
  (:hose-connect-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-hose-connect-pose)
      ;; TODO : recognize hose
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
        ;; only position is important in the temporary recognition
        (let ((pos (float-vector (elt (send from-body-coords :worldpos) 0)
                                 (elt (send from-body-coords :worldpos) 1)
                                 (elt (send from-body-coords :worldpos) 2))))
          (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
            (send msg :type drc_com_common::FC2OCSSmall::*HOSE_CONNECT_POSE*)
            (setf (elt (send msg :params) 0) (elt pos 0))
            (setf (elt (send msg :params) 1) (elt pos 1))
            (setf (elt (send msg :params) 2) (elt pos 2))
            (send self :send-to-ocs msg)
            (send smachine :go-to :initial)
            )))))
  (:drill-grasp-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-drill-grasp-pose)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
	
	(let ((msg (instance drc_com_common::FC2OCSSmall :init)))
	  (send msg :type drc_com_common::FC2OCSSmall::*DRILL_GRASP_POSE*)
	  (send msg :params (coords-to-float-vector from-body-coords (length (send msg :params))))
	  (send self :send-to-ocs msg)
	  (send smachine :go-to :initial)
	  ))))
  (:drill-put-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-drill-put-pose)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
	(let ((msg (instance drc_com_common::FC2OCSSmall :init)))
	  (send msg :type drc_com_common::FC2OCSSmall::*DRILL_PUT_POSE*)
	  (send msg :params (coords-to-float-vector from-body-coords (length (send msg :params))))
	  (send self :send-to-ocs msg)
	  (send smachine :go-to :initial)
	  ))))
  (:drill-wall-recog-callback (msg)
    (ros::ros-info ":drill-wall-recog-callback")
    (when (send smachine :state? :recognizing-drill-wall)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords-set (mapcar #'(lambda (ros-pose)
                                     (ros::tf-pose->coords ros-pose))
                                 (send msg :poses)))
             (from-body-coords-set
              (mapcar #'(lambda (c)
                          (send (send trans :copy-worldcoords) :transform c))
                      coords-set)))
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_WALL_POSE*)
          (send msg :params (coords-set-to-float-vector
                             from-body-coords-set (length (send msg :params))))
          (if (send msg :params)
              (progn
                (send self :send-to-ocs msg))
            (progn
              ;; failed because of too many poses
              (send self :report-recognition-failure-to-ocs)))
          (send smachine :go-to :initial)))))
  (:drill-button-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-drill-button-pose)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
	(let ((msg (instance drc_com_common::FC2OCSSmall :init)))
	  (send msg :type drc_com_common::FC2OCSSmall::*DRILL_BUTTON_POSE*)
	  (send msg :params (coords-to-float-vector from-body-coords (length (send msg :params))))
	  (send self :send-to-ocs msg)
	  (send smachine :go-to :initial)
	  ))))
  (:look-at-panorama-recog-callback (msg)
    (ros::ros-warn "panorama callback!")
    (send smachine :go-to :recognizing-look-at-point)
    (send self :look-at-recog-callback msg))
  (:look-at-recog-callback (msg)
    (when (send smachine :state? :recognizing-look-at-point)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
        ;; only position is important
        (let ((pos (float-vector (elt (send from-body-coords :worldpos) 0)
                                 (elt (send from-body-coords :worldpos) 1)
                                 (elt (send from-body-coords :worldpos) 2))))
          (send *robot* :angle-vector (send *ri* :state :reference-vector))
          (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
          (let ((origin-to-body
                 (send (send *robot* :body_lk) :copy-worldcoords)))
            (let ((pos-from-origin (send origin-to-body :transform-vector pos)))
              (send *robot* :head :look-at pos-from-origin)
              (send *ri* :angle-vector (send *robot* :angle-vector) 1000) ;; 1 sec
              (send *ri* :wait-interpolation)))))
      (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
        (send msg :type drc_com_common::FC2OCSSmall::*LOOK_AT_FINISHED*)
          (send self :send-to-ocs msg))
      (send smachine :go-to :initial)))
  (:go-pos-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-go-pos-location)
      (let ((trans (send self :lookup-from-ground-transform (send msg :header))))
        (let ((coords (ros::tf-pose-stamped->coords msg)))
          (let ((from-ground-coords (send trans :transform coords)))
            (let ((theta (atan2 (elt (send from-ground-coords :pos) 1)
                                (elt (send from-ground-coords :pos) 0))))
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
                           (make-coords :pos (send from-ground-coords :worldpos)
                                        :rot mat)))
                      (let ((offset-goal
                             (send (send goal-oriented-coords :copy-worldcoords)
                                   :translate (float-vector (- *go-pos-offset*) 0 0))))
                        (ros::publish   ;for debug
                          "/go_pos_goal"
                          (ros::coords->tf-pose-stamped offset-goal "ground"))
                        (setq go-pos-goal offset-goal)
                        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
                          (send msg :type drc_com_common::FC2OCSSmall::*GO_POS_POSE*)
                          ;; x, y and yaw is active
                          (setf (elt (send msg :params) 0) (elt (send go-pos-goal :worldpos) 0))
                          (setf (elt (send msg :params) 1) (elt (send go-pos-goal :worldpos) 1))
                          (setf (elt (send msg :params) 2) 0) ;force to set zero
                          (setf (elt (send msg :params) 3) (elt (car (send go-pos-goal :rpy-angle)) 0))
                          (setf (elt (send msg :params) 4) 0) ;force to set zero
                          (setf (elt (send msg :params) 5) 0) ;force to set zero
                          (send self :send-to-ocs msg)
                          (send smachine :go-to :initial)))))))))))))
  (:reset-pose-command-callback (msg)
    (send *ri* :angle-vector (send *robot* :reset-pose) 10000)
    (send *ri* :wait-interpolation)
    (send self :send-rviz-button-finished-to-ocs))
  (:reset-manip-pose-command-callback (msg)
    (send *ri* :angle-vector (send *robot* :reset-manip-pose) 10000)
    (send *ri* :wait-interpolation)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-reset-pose-command-callback (msg)
    (send *robot* :hand :arms :reset-pose)
    (hand-model2real)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-hook-pose-command-callback (msg)
    (send *robot* :hand :arms :hook-pose)
    (hand-model2real)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-hook-after-pose-command-callback (msg)
    (unix::sleep 5)
    (send *robot* :hand :arms :hook-pose)
    (hand-model2real)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-grasp-pose-command-callback (msg)
    (send *robot* :hand :arms :angle-vector (float-vector 90.0 0.0 0.0 0.0 90.0 90.0))
    ;(send *robot* :hand :arms :grasp-pose)
    (hand-model2real)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-abc-command-cb (msg)
    (send *ri* :start-auto-balancer)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-st-command-cb (msg)
    (send *ri* :start-st)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-imp-command-cb (msg)
    (send *ri* :start-impedance :arms :moment-gain #f(0 0 0) :k-p 1000 :d-p 400)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-abc-command-cb (msg)
    (send *ri* :stop-auto-balancer)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-st-command-cb (msg)
    (send *ri* :stop-st)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-imp-command-cb (msg)
    (send *ri* :stop-impedance :arms)
    (send self :send-rviz-button-finished-to-ocs))
  (:send-rviz-button-finished-to-ocs ()
    (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
      (send msg :type drc_com_common::FC2OCSSmall::*RVIZ_MENU_FINISHED*)
      (send self :send-to-ocs msg)
      )
    )
  (:angle-vector-send-command-callback
   (msg)
   (when (send smachine :state? :initial)
     (multiple-value-bind 
         (av time) (float-vector-to-angle-vector (send msg :float_params))
       (send smachine :go-to :executing-angle-vector)
       (format t "angle-vector ~A ~A~%" av time)
       (send *ri* :angle-vector av time)
       (send *ri* :wait-interpolation)
       )
     (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
       (send msg :type drc_com_common::FC2OCSSmall::*ANGLE_VECTOR_FINISHED*)
       (send self :send-to-ocs msg))
     (send smachine :go-to :initial))
   )
  (:drive-command-callback (msg)
    (when (send smachine :state? :initial) ;initial??, should it change to :executing-vehicle?
      (let ((handle-angle (elt (send msg :float_params) 0))
            (accell-angle (elt (send msg :float_params) 1))
            (brake-angle (elt (send msg :float_params) 2)))
        (ros::publish "/drive/operation/handle_cmd"
          (instance std_msgs::Float64 :init :data handle-angle))
        (ros::publish "/drive/operation/accell_cmd"
          (instance std_msgs::Float64 :init :data accell-angle))
        (ros::publish "/drive/operation/brake_cmd"
          (instance std_msgs::Float64 :init :data brake-angle)))))
  (:cancel-motion-now-command-callback
    (msg)
    (setq *cancel-motion-flag* t)
    )
  (:send-to-ocs (msg)
    (ros::publish "/fc_to_ocs_low_speed/input" msg))
  )


(load (robot-file "HRP2JSKNT"))
(init-robot-from-name "HRP2JSKNT")

(setq *go-pos-offset* 700)
(let ((drill-type-string (ros::get-param "~drill_type" "takenoko")))
  (cond
   ((equal drill-type-string "gun") (setq *drill-type* :gun))
   ((equal drill-type-string "takenoko") (setq *drill-type* :takenoko))
   )
  )
(ros::advertise "image_marker" image_view2::ImageMarker2)
(setq *app* (instance fc-executive-app :init))
(setq *tf* (instance ros::transform-listener :init))

(ros::advertise "/go_pos_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/hose_grasp_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/hose_connect_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/look_at_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/panorama/panorama_processing/output/screenrectangle" geometry_msgs::PolygonStamped)
(ros::advertise "/valve_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/door_recognition/input_support_plane_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/door_recognition/input_target_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/debri_recognition/input_line" geometry_msgs::PolygonStamped)
(ros::advertise "/drill_recognition/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/drill_recognition_for_button/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/drill_recognition_for_put/input_rect" geometry_msgs::PolygonStamped)
(ros::advertise "/drill_recognition_for_wall/input_poly" geometry_msgs::PolygonStamped)
(ros::advertise "/fc_to_ocs_low_speed/input" drc_com_common::FC2OCSSmall)
(ros::advertise "/drive/operation/handle_cmd" std_msgs::Float64)
(ros::advertise "/drive/operation/accel_cmd" std_msgs::Float64)
(ros::advertise "/drive/operation/brake_cmd" std_msgs::Float64)

(ros::subscribe "/fc_from_ocs_low_speed/output" drc_com_common::OCS2FCSmall
  #'send (send *app* :ocs2fc-message-handler) :root-callback)
(ros::subscribe "/go_pos_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :go-pos-recog-callback)
(ros::subscribe "/look_at_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :look-at-recog-callback)
(ros::subscribe "/panorama/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :look-at-panorama-recog-callback)
(ros::subscribe "/valve_recognition/snapshot/output" jsk_recognition_msgs::Torus
  #'send *app* :valve-recog-callback)
(ros::subscribe "/door_recognition/snapshot/output" jsk_recognition_msgs::BoundingBoxArray
  #'send *app* :door-recog-callback)
(ros::subscribe "/debri_recognition/snapshot/output" pcl_msgs::ModelCoefficients
  #'send *app* :debri-recog-callback)
(ros::subscribe "/hose_grasp_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :hose-grasp-recog-callback)
(ros::subscribe "/hose_connect_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :hose-connect-recog-callback)
(ros::subscribe "/drill_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :drill-grasp-recog-callback)
(ros::subscribe "/drill_recognition_for_button/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :drill-button-recog-callback)
(ros::subscribe "/drill_recognition_for_put/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :drill-put-recog-callback)
(ros::subscribe "/drill_recognition_for_wall/snapshot/output" geometry_msgs::PoseArray
  #'send *app* :drill-wall-recog-callback)
(ros::advertise "/go_pos_goal" geometry_msgs::PoseStamped)

(ros::rate 10)

;;(ros::spin)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (send (send *app* :state-machine) :proc)
  )
  
