#!/usr/bin/env roseus

(ros::roseus "ocs_executive")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "jsk_topic_tools")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "drc_task_common")
(ros::roseus-add-msgs "pcl_msgs")

(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/state-machine.l")
(load "package://drc_task_common/euslisp/generate-valve-motion.l")
(load "package://drc_task_common/euslisp/generate-debri-motion.l")
(load "package://drc_task_common/euslisp/generate-drill-motion.l")
(load "package://drc_task_common/euslisp/generate-hose-motion.l")
(load "package://drc_task_common/euslisp/generate-door-motion.l")
(load "package://drc_task_common/euslisp/test-drc-terrain-walk.l")

(defun footstep-array-from-two-coordinates (goal-coords int-vector)
  (let ((goal-footstep (instance jsk_footstep_msgs::FootstepArray :init)))
    (send goal-footstep :header :frame_id "ground")
    (send goal-footstep :header :stamp (ros::time-now))
    (send goal-footstep :footsteps
          (mapcar #'(lambda (c)
                      (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                        (send f :pose (ros::coords->tf-pose c))
                        f))
                  goal-coords))
    (send (car (send goal-footstep :footsteps)) :leg (elt int-vector 0))
    (send (cadr (send goal-footstep :footsteps)) :leg (elt int-vector 1))
    goal-footstep))

(defclass fc-executive-app
  :super propertied-object
  :slots (state go-pos-goal ocs2fc-message-handler smachine head-overwrite-mode footstep-planner-client footstep-controller-client))

(defmethod fc-executive-app
  (:init ()
    ;; prepare state machine
    (setq smachine (make-fc-state-machine))
    (send self :register-timelimits smachine)
    (setq footstep-planner-client (instance ros::simple-action-client :init
                                            "footstep_planner"
                                            jsk_footstep_msgs::PlanFootstepsAction))
    (setq footstep-controller-client (instance ros::simple-action-client :init
                                               "footstep_controller"
                                               jsk_footstep_msgs::ExecFootstepsAction))
    (setq ocs2fc-message-handler (instance narrowband-message-handler :init))
    ;; register callbacks
    (send ocs2fc-message-handler :register-callbacks
          (list drc_com_common::OCS2FCSmall::*GO_POS_REGION*
                self :go-pos-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*GO_POS_EXEC*
                self :go-pos-exec-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_REGION*
                self :look-at-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_PITCH_ANGLE*
                self :look-at-pitch-command-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_AT_YAW_ANGLE*
                self :look-at-yaw-command-callback)
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
          (list drc_com_common::OCS2FCSmall::*DRILL_ROTATE_REGION*
                self :drill-rotate-region-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_ROTATE_EXEC*
                self :drill-rotate-exec-command-callback)
          ;; rviz plugins
          (list drc_com_common::OCS2FCSmall::*RESET_POSE*
                self :reset-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*RESET_MANIP_POSE*
                self :reset-manip-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRILL_RESET_POSE*
                self :drill-reset-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*WATCH_DRILL_POSE*
                self :watch-drill-pose-command-callback)
          (list drc_com_common::OCS2FCSmall::*DOOR_THROUGH_POSE*
                self :door-through-pose-command-callback)
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
          (list drc_com_common::OCS2FCSmall::*HRPSYS_START_IMP_SOFT*
                self :hrpsys-start-imp-command-soft-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_START_IMP_HARD*
                self :hrpsys-start-imp-command-hard-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_RESET_FORCE_OFFSET*
                self :hrpsys-reset-force-offset-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ABC*
                self :hrpsys-stop-abc-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ST*
                self :hrpsys-stop-st-command-cb)
          (list drc_com_common::OCS2FCSmall::*HRPSYS_STOP_IMP*
                self :hrpsys-stop-imp-command-cb)
          (list drc_com_common::OCS2FCSmall::*GO_POS_ZERO*
                self :go-pos-zero-cb)
          (list drc_com_common::OCS2FCSmall::*REF_FORCE_ZERO*
                self :ref-force-zero-cb)
          (list drc_com_common::OCS2FCSmall::*CALIB_HAND*
                self :calib-hand-command-cb)
          (list drc_com_common::OCS2FCSmall::*ANGLE_VECTOR_SEND*
                self :angle-vector-send-command-callback)
          (list drc_com_common::OCS2FCSmall::*REACH_UNTIL_TOUCH_EXEC*
                self :reach-until-touch-command-callback)
          (list drc_com_common::OCS2FCSmall::*CANCEL_MOTION_NOW*
                self :cancel-motion-now-command-callback)
          (list drc_com_common::OCS2FCSmall::*DRIVE_COMMAND*
                self :drive-command-callback)
          (list drc_com_common::OCS2FCSmall::*ENABLE_OVERWRITE_HEAD_JOINT*
                self :enable-overwrite-head-joint-command-callback)
          (list drc_com_common::OCS2FCSmall::*DISABLE_OVERWRITE_HEAD_JOINT*
                self :disable-overwrite-head-joint-command-callback)
          (list drc_com_common::OCS2FCSmall::*MOVE_TO_STAIR_POSE*
                self :move-to-stair-pose-callback)
          (list drc_com_common::OCS2FCSmall::*EXECUTE_STAIR_FOOTSTEPS*
                self :execute-stair-footsteps-callback)
          ;; grasp
          (list drc_com_common::OCS2FCSmall::*START_GRASP*
                self :start-grasp-callback)
          (list drc_com_common::OCS2FCSmall::*STOP_GRASP*
                self :stop-grasp-callback)
          ;; impedance
          (list drc_com_common::OCS2FCSmall::*START_IMP*
                self :start-imp-callback)
          (list drc_com_common::OCS2FCSmall::*STOP_IMP*
                self :stop-imp-callback)
          (list drc_com_common::OCS2FCSmall::*START_IMP_WITH_PARAM*
                self :start-imp-with-param-callback)
          ;; terrain
          (list drc_com_common::OCS2FCSmall::*PROJECT_FOOTPRINT*
                self :project-footprint-callback)
          (list drc_com_common::OCS2FCSmall::*LOOK_GROUND*
                self :lookground-callback)
          (list drc_com_common::OCS2FCSmall::*EXECUTE_FOOTSTEPS*
                self :execute-footstep-callback)
          (list drc_com_common::OCS2FCSmall::*REQUEST_FOOTSTEP_PLANNING*
                self :request-footstep-planning-callback)
          )
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
    (send (send smachine :node :recognizing-drill-rotate-pose)
          :add-timelimit 15 :initial
          :deligate-object self
          :deligate-method :report-recognition-failure-to-ocs)
    (send (send smachine :node :recognizing-drill-wall)
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
      ;; We do not specify ROI region anymore
      ;; ;; Enable passthrough for 10seconds
      ;; (call-request-duration "/drill_recognition/passthrough/request_duration" 10)
      ;; ;; convert OCS2FCSmall into geometry_msgs/Polygon
      ;; (ros::publish "/drill_recognition/input_rect"
      ;;   (integer-vector-to-rect-polygon-stamped msg))
      ;; ;; enable snapshot
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
      (call-request-duration "/drill_recognition_for_button/passthrough/request_duration" 10)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      ;; (ros::publish "/drill_recognition_for_button/input_rect"
      ;;   (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition_for_button/snapshot/request")
      (send smachine :go-to :recognizing-drill-button-pose)
      ))
  (:drill-rotate-region-command-callback (msg)
    (when (send smachine :state? :initial)
      (call-request-duration "/drill_recognition_for_button/passthrough/request_duration" 10)
      ;; convert OCS2FCSmall into geometry_msgs/Polygon
      ;; (ros::publish "/drill_recognition_for_button/input_rect"
      ;;   (integer-vector-to-rect-polygon-stamped msg))
      ;; enable snapshot
      (call-empty-service "/drill_recognition_for_button/rotate_snapshot/request")
      (send smachine :go-to :recognizing-drill-rotate-pose)
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
          (cond
           (head-overwrite-mode
            (send *ri* :set-joint-angles-of-group "head_for_teleop" (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) (* 0.001 tm))
            (send *ri* :wait-interpolation-of-group "head_for_teleop")
            )
           (t
            (send *ri* :angle-vector (send *robot* :angle-vector) tm)
            (send *ri* :wait-interpolation)))
          (send self :panorama-shutter)

          (send *robot* :head :neck-y :joint-angle 0)
          (cond
           (head-overwrite-mode
            (send *ri* :set-joint-angles-of-group "head_for_teleop" (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) (* 0.001 tm))
            (send *ri* :wait-interpolation-of-group "head_for_teleop")
            )
           (t
            (send *ri* :angle-vector (send *robot* :angle-vector) tm)
            (send *ri* :wait-interpolation)))
          (send self :panorama-shutter)

          (send *robot* :head :neck-y :joint-angle (+ 1 (send *robot* :head :neck-y :min-angle)))
          (cond
           (head-overwrite-mode
            (send *ri* :set-joint-angles-of-group "head_for_teleop" (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) (* 0.001 tm))
            (send *ri* :wait-interpolation-of-group "head_for_teleop")
            )
           (t
            (send *ri* :angle-vector (send *robot* :angle-vector) tm)
            (send *ri* :wait-interpolation)))
          (send self :panorama-shutter)

          (send *robot* :angle-vector initial-av)
          (cond
           (head-overwrite-mode
            (send *ri* :set-joint-angles-of-group "head_for_teleop" (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) (* 0.001 tm))
            (send *ri* :wait-interpolation-of-group "head_for_teleop")
            )
           (t
            (send *ri* :angle-vector (send *robot* :angle-vector) tm)
            (send *ri* :wait-interpolation))))
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
             (stand-coords (float-vector-to-coords (send msg :stand_coords)))
             (coords (float-vector-to-coords params)))
        (ros::ros-info "grasp drill at ~A" coords)
        (send smachine :go-to :executing-drill-grasp)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (let* ((arm-side (send msg :arm_side))
               (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
	       (if arm
                   (progn
                     (setq *drill-main-arm* arm)
                     (send-drill-grasp-motion coords :stand-coords stand-coords :drill-rotate-num (send msg :image_type))
                     )
                 (ros::ros-info "arm side empty")
                 )
               )
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
	  (set-drill-default-coords)
	  (send msg :params (coords-set-to-float-vector 
			     (list
			      (send (send (send *robot* :body_lk) :copy-worldcoords) :transformation (send *drill* :copy-worldcoords)) 
                              (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :transformation (send *drill* :copy-worldcoords))
                              (make-coords :pos (float-vector (check-diff-angle) 1 0))
                              ) (length (send msg :params))
                              ))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_GRASP_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)
        ))
    )
  (:drill-put-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((params (send msg :float_params))
             (coords (float-vector-to-coords params))
             (stand-coords (float-vector-to-coords (send msg :stand_coords))))
        (ros::ros-info "put drill at ~A" coords)
        (send smachine :go-to :executing-drill-put)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (let* ((arm-side (send msg :arm_side))
               (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
	       (if arm
                   (progn
                     (setq *drill-main-arm* arm)
		     (send-drill-put-motion coords :stand-coords stand-coords)
                     )
                 (ros::ros-info "arm side empty")
		 )
	       )
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
      (let ((coords-set (float-vector-to-coords-set (send msg :float_params)))
            (stand-coords (float-vector-to-coords (send msg :stand_coords))))
        (ros::ros-info "Target coords are: ~A" coords-set)
        (let* ((arm-side (send msg :arm_side))
               (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
	       (if arm
                   (progn
                     (setq *drill-main-arm* arm)
		     (send-drill-wall-motion coords-set :stand-coords stand-coords)
                     )
                 (ros::ros-info "arm side empty")
		 )
	       )

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
          (let (ret)
            (let* ((arm-side (send msg :arm_side))
                   (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
              (if arm
                  (progn
                    (setq *drill-main-arm* arm)
                    (setq ret (send-drill-button-motion coords))
                    )
                (ros::ros-info "arm side empty")
                )
              )
            ;; send finish signal to ocs
            (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
              (send msg :type drc_com_common::FC2OCSSmall::*DRILL_BUTTON_FINISHED*)
              (if (equal ret :success)
                  (send msg :success t)
                  )
              (send self :send-to-ocs msg))
            )
          )
        (send smachine :go-to :initial)
        ))
    )
  (:drill-rotate-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (let* ((coords-set (float-vector-to-coords-set (send msg :float_params))))
        (ros::ros-info (format nil "coords set: ~A" coords-set))
        (send smachine :go-to :executing-drill-button)
        (setq *cancel-motion-flag* nil)
        (catch 'exit
        ;; send motion to real robot
        (let* ((arm-side (send msg :arm_side))
               (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
	       (if arm
                   (progn
                     (setq *drill-main-arm* arm)
		     (send-drill-rotate-motion (elt coords-set 0) (elt coords-set 1) :drill-rotate-num (send msg :image_type))
                     )
                 (ros::ros-info "arm side empty")
		 )
	       )
        ;; send finish signal to ocs
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (set-drill-default-coords)
	  (send msg :params (coords-set-to-float-vector
			     (list
                              (send (send (send *robot* :body_lk) :copy-worldcoords) :transformation (send *drill* :copy-worldcoords))
                              (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :transformation (send *drill* :copy-worldcoords))
                              (make-coords :pos (float-vector (check-diff-angle) 1 0))
                              ) (length (send msg :params))))
	  (send msg :type drc_com_common::FC2OCSSmall::*DRILL_ROTATE_FINISHED*)
          (send self :send-to-ocs msg))
        )
        (send smachine :go-to :initial)
        ))
    )
  (:door-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind
          (coords size door-push/pull-idx door-name-idx) (float-vector-to-door-params (send msg :float_params))
        (let* ((stand-coords (float-vector-to-coords (send msg :stand_coords)))
               (arm-side (send msg :arm_side))
               (arm
                (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm)
                      ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                      (t :rarm)))
               (door-push/pull
                (cond ((eps= (float door-push/pull-idx) (float drc_com_common::OCS2FCSmall::*DOOR_PUSH*)) :push)
                      ((eps= (float door-push/pull-idx) (float drc_com_common::OCS2FCSmall::*DOOR_PULL*)) :pull)
                      (t :push)))
               (door-name
                (cond ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_BOX2*)) :drc-box2)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_TEST*)) :drc-test)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_TEST_MIRROR*)) :drc-test-mirror)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_SAGAMI*)) :drc-sagami)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_SAGAMI_MIRROR*)) :drc-sagami-mirror)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_LASVEGAS*)) :drc-lasvegas)
                      ((eps= (float door-name-idx) (float drc_com_common::OCS2FCSmall::*DOOR_DRC_LASVEGAS_MIRROR*)) :drc-lasvegas-mirror)
                      (t :drc-box2)))
               )
          (send smachine :go-to :executing-door)
          (setq *cancel-motion-flag* nil)
          (catch 'exit
            (ros::ros-info "opening door at ~A ~A~%" coords size)
            ;; send motion to real robot
            (send-door-motion :door-name door-name :door-coords coords :arm arm :stand-coords stand-coords :push/pull door-push/pull)
            ;; send finish signal to ocs
            (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
              (send msg :type drc_com_common::FC2OCSSmall::*DOOR_FINISHED*)
              (send self :send-to-ocs msg))
            )
          )
        (send smachine :go-to :initial)
        )))
  (:valve-exec-command-callback (msg)
    (when (send smachine :state? :initial)
      (multiple-value-bind
          (coords radius grasp-mode-idx) (float-vector-to-valve-params (send msg :float_params))
        (let* ((stand-coords (float-vector-to-coords (send msg :stand_coords)))
               (arm-side (send msg :arm_side))
               (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t :rarm)))
               (grasp-center? (eps= (float grasp-mode-idx) (float drc_com_common::OCS2FCSmall::*VALVE_CENTER_GRASP*))))
          (send smachine :go-to :executing-valve)
          (setq *cancel-motion-flag* nil)
          (catch 'exit
            (progn
              (ros::ros-info "rotating valve at ~A,~A,~A~%"
                             (send coords :worldpos)
                             (car (send coords :rpy-angle))
                             radius)
              ;; send motion to real robot
              (send-valve-motion radius coords :arm arm :stand-coords stand-coords :grasp-center? grasp-center?)
              ;; send finish signal to ocs
              (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
                (send msg :type drc_com_common::FC2OCSSmall::*VALVE_FINISHED*)
                (send self :send-to-ocs msg))
              ))
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
        ;; enable snapshot
        (call-request-duration "/lookat/output_passthrough/request_duration" 5)
        (send smachine :go-to :recognizing-look-at-point)
        (ros::publish "/lookat/input"
          (compute-centroid-of-polygon-stamped
           (integer-vector-to-rect-polygon-stamped msg)))
        )
       ((eq (send msg :image_type) drc_com_common::OCS2FCSmall::*PANORAMA_VIEW*)
        (ros::publish "/fisheye/input"
          (compute-centroid-of-polygon-stamped
           (integer-vector-to-rect-polygon-stamped msg)))
        ;; enable snapshot: no need?
        ;; (call-empty-service "/panorama/snapshot/request")
        ;; /fisheye/fisheye_click_to_pose/output_point
        (send smachine :go-to :recognizing-look-at-point))
        ;;(send smachine :go-to :recognizing-look-at-point-panorama))
       (t
        (error "unknown image type: ~A" (send msg :image_type)))
       )))
  (:look-at-pitch-command-callback (msg)
    (when (send smachine :state? :initial)
      (send *robot* :angle-vector (send *ri* :state :reference-vector))
      (send *robot* :head :neck-p :joint-angle (elt (send msg :float_params) 0))
      (send self :look-at-angle-common)))
  (:look-at-yaw-command-callback (msg)
    (when (send smachine :state? :initial)
      (send *robot* :angle-vector (send *ri* :state :reference-vector))
      (send *robot* :head :neck-y :joint-angle (elt (send msg :float_params) 0))
      (send self :look-at-angle-common)))
  (:look-at-angle-common ()
    "Common method for :look-at-pitch-command-callback and :look-at-yaw-command-callback.
It sends :angle-vector to robot and return 'finished' message to ocs"
    (cond
     (head-overwrite-mode
      (send *ri* :set-joint-angles-of-group "head_for_teleop"
            (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 1.0)
      (send *ri* :wait-interpolation-of-group "head_for_teleop"))
     (t
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)))
    ;; return
    (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
      (send msg :type drc_com_common::FC2OCSSmall::*LOOK_AT_FINISHED*)
        (send self :send-to-ocs msg))
    (send smachine :go-to :initial))
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
  (:drill-rotate-recog-callback (msg)         ;geometry_msgs/PoseStamped
    (when (send smachine :state? :recognizing-drill-rotate-pose)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (ros::tf-pose-stamped->coords msg))
             (from-body-coords (send trans :transform coords)))
        (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
          (send msg :type drc_com_common::FC2OCSSmall::*DRILL_ROTATE_POSE*)
          (send msg :params (coords-to-float-vector from-body-coords (length (send msg :params))))
          (send self :send-to-ocs msg)
          (send smachine :go-to :initial)
          ))))
  ;; (:look-at-panorama-recog-callback (msg)
  ;;   (ros::ros-warn "panorama callback!")
  ;;   (send smachine :go-to :recognizing-look-at-point)
  ;;   (send self :look-at-recog-callback (pose-stamped-to-point-stamped msg)))
  (:look-at-recog-callback (msg)
    (when (send smachine :state? :recognizing-look-at-point)
      (let* ((trans (send self :lookup-from-body-transform (send msg :header)))
             (coords (make-coords :pos (ros::tf-point->pos (send msg :point))))
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
              (cond
               (head-overwrite-mode
                (send *ri* :set-joint-angles-of-group "head_for_teleop" (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 1.0)
                (send *ri* :wait-interpolation-of-group "head_for_teleop")
                )
               (t
                (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
                (send *ri* :wait-interpolation)))
              ))))
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
  )

;; grasp and impedance
(defmethod fc-executive-app
  (:limb-grasp-callback
   (limb pose &optional av)
   (cond
    ((find-method *ri* (if (eq pose :grasp) :start-grasp :stop-grasp))
     (format t ":limb-grasp-callback: ~A (~A)~%"
             (if (eq pose :grasp) :start-grasp :stop-grasp) limb)
     (send *ri* (if (eq pose :grasp) :start-grasp :stop-grasp) limb))
    ((and (find-method *ri* :hand-angle-vector)
          (find-method (send *ri* :get-val 'robot) :hand-angle-vector))
     (cond
      ((eq (length (send (send *ri* :get-val 'robot) :hand-angle-vector))
           (length av))
       (format t ":limb-grasp-callback: receive hav=~A~%" av)
       (send (send *ri* :get-val 'robot) :hand-angle-vector av))
      (t
       (format t ":limb-grasp-callback: grasp-pose=~A~%"
               (if (eq pose :grasp) :grasp-pose :hook-pose))
       (send (send *ri* :get-val 'robot) :hand limb
             (if (eq pose :grasp) :grasp-pose :hook-pose))))
     (send *ri* :hand-angle-vector
           (send (send *ri* :get-val 'robot) :hand-angle-vector))
     ))
   )
  (:start-grasp-callback
   (msg) (send self :limb-grasp-callback
               (cond
                ((eq (send msg :arm_side)
                     drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                ((eq (send msg :arm_side)
                     drc_com_common::OCS2FCSmall::*LARM*) :larm)
                (t :arms))
               :grasp
               (send msg :float_params)))
  (:stop-grasp-callback
   (msg) (send self :limb-grasp-callback
               (cond
                ((eq (send msg :arm_side)
                     drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                ((eq (send msg :arm_side)
                     drc_com_common::OCS2FCSmall::*LARM*) :larm)
                (t :arms))
               :open
               (send msg :float_params)))
  ;;
  (:start-imp-callback
   (msg)
   (let* ((limb (cond
                 ((eq (send msg :arm_side)
                      drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                 ((eq (send msg :arm_side)
                      drc_com_common::OCS2FCSmall::*LARM*) :larm)
                 ((eq (send msg :arm_side)
                      drc_com_common::OCS2FCSmall::*RLEG*) :rleg)
                 ((eq (send msg :arm_side)
                      drc_com_common::OCS2FCSmall::*LLEG*) :lleg)
                 (t :arms)))
          (param nil))
     (format t "start-imp-callback ~A ~A~%" limb param)
     (send* *ri* :start-impedance limb param)))
  (:stop-imp-callback
   (msg)
   (format t "stop-imp-callback")
   (send *ri* :stop-impedance
         (print (cond ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*LARM*) :larm)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*RLEG*) :rleg)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*LLEG*) :lleg)
                      (t :arms)))))
  (:start-imp-with-param-callback
   (msg)
   (let* ((limb
           (cond ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*RARM*) :rarm)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*LARM*) :larm)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*RLEG*) :rleg)
                      ((eq (send msg :arm_side)
                           drc_com_common::OCS2FCSmall::*LLEG*) :lleg)
                      (t :arms)))
          (param
           (if (and (send msg :float_params)
                    (>= (length (send msg :float_params)) 6))
               (flatten
                (map cons 'list
                     (list :m-p :d-p :k-p :m-r :d-r :k-r)
                     (send msg :float_params))))))
     (format t "start-imp-callback ~A ~A~%" limb param)
     (send* *ri* :start-impedance limb param)))
  )

(defmethod fc-executive-app
  (:reset-pose-command-callback (msg)
    (send *ri* :angle-vector (send *robot* :reset-pose) 10000)
    (send *ri* :wait-interpolation)
    (send self :send-rviz-button-finished-to-ocs))
  (:reset-manip-pose-command-callback (msg)
    (send *ri* :angle-vector (send *robot* :reset-manip-pose) 10000)
    (send *ri* :wait-interpolation)
    (send self :send-rviz-button-finished-to-ocs))
  (:drill-reset-pose-command-callback (msg)
    (let* ((arm-side (send msg :arm_side))
           (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
      (if arm
          (progn
            (setq *drill-main-arm* arm)            )
        (ros::ros-info "arm side empty")
        )
      )
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send self :send-rviz-button-finished-to-ocs))
  (:watch-drill-pose-command-callback (msg)
    (let* ((arm-side (send msg :arm_side))
           (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
      (if arm
          (progn
            (setq *drill-main-arm* arm)
            )
        (ros::ros-info "arm side empty")
        )
      )
    (when (or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (model2real :time 5000)
      )
    (send *robot* :angle-vector (get-watch-drill-pose))
    (model2real :time 5000)
    (send self :send-rviz-button-finished-to-ocs))
  (:door-through-pose-command-callback (msg)
    (send *robot* :angle-vector (get-door-through-pose))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (hand-start-grasp :real? t)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-reset-pose-command-callback (msg)
    (hand-stop-grasp :real? t :type :reset-pose)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-hook-pose-command-callback (msg)
    (hand-stop-grasp :real? t)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-hook-after-pose-command-callback (msg)
    (unix::sleep 5)
    (hand-stop-grasp :real? t)
    (send self :send-rviz-button-finished-to-ocs))
  (:hand-grasp-pose-command-callback (msg)
    (cond
     ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
      (drill-t-grasp :arm :arms)
      (unix::usleep 500000)
      (drill-full-grasp :arm :arms)
      )
     (t
      (hand-start-grasp :real? t :type :default-grasp)
      ))
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-abc-command-cb (msg)
    (send *ri* :start-auto-balancer)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-st-command-cb (msg)
    (send *ri* :start-st)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-imp-command-cb (msg)
    (start-default-impedance)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-imp-command-soft-cb (msg)
    (start-soft-impedance)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-start-imp-command-hard-cb (msg)
    (start-hard-impedance)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-reset-force-offset-cb (msg)
    (send *ri* :reset-force-moment-offset-arms)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-abc-command-cb (msg)
    ;(send *ri* :stop-auto-balancer)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-st-command-cb (msg)
    ;(send *ri* :stop-st)
    (send self :send-rviz-button-finished-to-ocs))
  (:hrpsys-stop-imp-command-cb (msg)
    (send *ri* :stop-impedance :arms)
    (send self :send-rviz-button-finished-to-ocs))
  (:go-pos-zero-cb (msg)
    (send *ri* :go-pos 0 0 0)
    (send self :send-rviz-button-finished-to-ocs))
  (:ref-force-zero-cb (msg)
    (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms)
    (send self :send-rviz-button-finished-to-ocs))
  (:calib-hand-command-cb (msg)
    (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "JAXON_RED") (equal *robot-name* "STARO"))
           (send *ri* :hand-reset))
          (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
           (send *ri* :hand-joint-calib)))
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
  (:reach-until-touch-command-callback
   (msg)
   (when (send smachine :state? :initial)
     (multiple-value-bind
         (dir thre max-dist dist-step time-step rotation-axis)
         (float-vector-to-reach-until-touch-params (send msg :float_params))
       (let* ((arm-side (send msg :arm_side))
              (arm (cond ((equal arm-side drc_com_common::OCS2FCSmall::*LARM*) :larm) ((equal arm-side drc_com_common::OCS2FCSmall::*RARM*) :rarm) (t nil))))
         (send smachine :go-to :executing-reach-until-touch)
         (format t "reach-until-touch ~a ~a :thre ~a :max-dist ~a :dist-step ~a :time-step ~a :rotation-axis ~a~%"
                 arm dir thre max-dist dist-step time-step rotation-axis)
         (send *ri* :reach-until-touch
               arm dir :thre thre :max-dist max-dist
               :dist-step dist-step :time-step time-step :rotation-axis rotation-axis
               :coords-system :world)
         ))
     (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
       (send msg :type drc_com_common::FC2OCSSmall::*REACH_UNTIL_TOUCH_FINISHED*)
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
  (:enable-overwrite-head-joint-command-callback
   (msg)
   (unless head-overwrite-mode (send *ri* :add-joint-group "head_for_teleop" (send-all (send *robot* :head :joint-list) :name)))
   (setq head-overwrite-mode t)
   )
  (:disable-overwrite-head-joint-command-callback
   (msg)
   (when head-overwrite-mode (send *ri* :remove-joint-group "head_for_teleop"))
   (setq head-overwrite-mode nil)
   )
  ;; stair
  (:move-to-stair-pose-callback
    (msg)
    (move-to-stair-pose)
    ;; (let ((msg (instance drc_com_common::FC2OCSSmall :init)))
    ;;    (send msg :type drc_com_common::FC2OCSSmall::*MOVE_TO_STAIR_POSE_FINISHED*)
    ;;    (send self :send-to-ocs msg))
    )
  (:execute-stair-footsteps-callback
    (msg)
    (let ((footstep-coords (cons (make-coords)
                                 (float-vector-to-coords-set (send msg :float_params))))
          (footstep-names '(:lleg :rleg :lleg)))
      (let ((footstep-coords* (mapcar #'(lambda (c n)
                                          (make-coords :pos (send c :worldpos)
                                                       :name n))
                                      footstep-coords footstep-names)))
        ;; change ri parameter
        ;; too ugly, yeah too ugly...
        (default-stair-param-hrp2jsk)
        (send *ri* :set-gait-generator-param
              :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
              :swing-trajectory-delay-time-offset 0.2
              :default-step-time 1.7
              :default-double-support-ratio 0.29
              :default-double-support-static-ratio 0.117
              )
        (when (or (string= "JAXON" *robot-name*)  (equal *robot-name* "JAXON_RED"))
          (send *ri* :set-auto-balancer-param :default-zmp-offsets
                (list (float-vector -20 0 0) (float-vector -20 0 0)))
          )
        (print footstep-coords*)
        (send *ri* :set-foot-steps footstep-coords*)
        ;; fix ri parmaeter for planar transition
        (if (or (string= "JAXON" *robot-name*)  (equal *robot-name* "JAXON_RED"))
            (send *ri* :set-gait-generator-param
                    ;;:default-orbit-type :cycloid
                    :default-orbit-type :stair
                    :toe-angle 0 :heel-angle 0
                    :default-step-time 1.7
                    :default-double-support-ratio 0.29
                    :default-double-support-static-ratio 0.117
                    ;;:default-double-support-ratio 0.16
                    :default-step-height 0.05)
          (send *ri* :set-gait-generator-param
                :default-orbit-type :cycloiddelay
                :default-step-time 1.1
                :toe-angle 0 :heel-angle 0
                :default-double-support-ratio 0.32
                :default-step-height 0.05))
        )))
  ;; terrain
  (:project-footprint-callback
   (input-msg)
   (ros::ros-info ":project-footprint-callback")
   (let ((msg (instance jsk_interactive_marker::SnapFootPrintInput :init)))
     (send msg :input_pose
           (ros::coords->tf-pose-stamped
            (float-vector-to-coords (subseq (send input-msg :float_params) 0 6))
            "ground"))
     (send msg :lleg_pose
           (ros::coords->tf-pose
            (float-vector-to-coords (subseq (send input-msg :float_params) 6 12))))
     (send msg :rleg_pose
           (ros::coords->tf-pose
            (float-vector-to-coords (subseq (send input-msg :float_params) 12 18))))
     (ros::publish "/footstep_planner/project_footprint" msg)))
  (:projected-footprint-callback
   (msg)
   (ros::ros-info ":projected-footprint-callback")
   (let ((ocs-msg (instance drc_com_common::FC2OCSSmall :init)))
     (send ocs-msg :type drc_com_common::FC2OCSSmall::*PROJECTED_FOOTSTEP*)
     (send ocs-msg :params (coords-to-float-vector
                            (ros::tf-pose->coords (send msg :pose))
                            (length (send ocs-msg :params))))
     (send self :send-to-ocs ocs-msg)))
  (:lookground-callback
    (msg)
    (ros::ros-info ":lookground")
    (when (send smachine :state? :initial)
      (send smachine :go-to :executing-terrain-lookground)
      (let ((req (instance std_srvs::EmptyRequest :init)))
        (ros::service-call "/lookaround_ground" req))
      (let ((ocs-msg (instance drc_com_common::FC2OCSSmall :init)))
        (send ocs-msg :type drc_com_common::FC2OCSSmall::*LOOK_GROUND_FINISHED*)
        (send self :send-to-ocs ocs-msg))
      (send smachine :go-to :initial)))
  (:request-footstep-planning-callback
    (msg)
    (ros::ros-info ":request-footstep-planning-callback")
    (when (send smachine :state? :initial)
      (send smachine :go-to :planning-terrain-footsteps)
      (let ((coords-list (float-vector-to-coords-set (send msg :float_params))))
        (let ((goal-coords (subseq coords-list 0 2))
              (initial-coords (subseq coords-list 2 4)))
          (let ((goal (instance jsk_footstep_msgs::PlanFootstepsActionGoal :init))
                (client footstep-planner-client))
            (let ((goal-footstep (footstep-array-from-two-coordinates
                                  goal-coords (send msg :image_roi_info)))
                  (initial-footstep (footstep-array-from-two-coordinates
                                     initial-coords (subseq (send msg :image_roi_info) 2 4))))
              (send goal :goal :goal_footstep goal-footstep)
              (send goal :goal :initial_footstep initial-footstep))
            (ros::ros-info "Waiting for planning server")
            (send client :wait-for-server)
            (send client :send-goal goal)
            (send smachine :go-to :planning-terrain-footsteps)
            (let ((successp (send client :wait-for-result)))
              (let ((ocs-msg (instance drc_com_common::FC2OCSSmall :init)))
                (if (not successp)
                    (send ocs-msg :type drc_com_common::FC2OCSSmall::*FOOTSTEP_PLANNING_FAILED*)
                  (let ((result (send client :get-result)))
                    (multiple-value-bind
                     (float-vector-footstep integer-vector-footstep)
                     (footstep-array->vector (send result :result)
                                             (length (send ocs-msg :params))
                                             (length (send ocs-msg :int_params)))
                     (send ocs-msg :params float-vector-footstep)
                     (send ocs-msg :int_params integer-vector-footstep)
                     (send ocs-msg :type drc_com_common::FC2OCSSmall::*FOOTSTEP_PLANNING_RESULT*))))
                (send self :send-to-ocs ocs-msg))))))
      (send smachine :go-to :initial)))
  (:execute-footstep-callback
    (msg)
    (ros::ros-info ":execute-footstep-callback")
    (when (send smachine :next? :executing-terrain-footsteps)
      (send smachine :go-to :executing-terrain-footsteps)
      (let ((coords-list (float-vector-to-coords-set (send msg :float_params)))
            (client footstep-controller-client))
        (ros::ros-info "Waiting for controller server")
        (send client :wait-for-server)
        (let ((goal (instance jsk_footstep_msgs::ExecFootstepsActionGoal :init))
              (footstep-array (vector->footstep-array (send msg :float_params)
                                                      (send msg :image_roi_info))))
          (if (<= (length coords-list) 1)
              (ros::ros-error "Only 1 or 0 coords")
            (progn
              (send footstep-array :header :frame_id "ground")
              (send footstep-array :header :stamp (ros::time-now))
              (send goal :goal :footstep footstep-array)
              (send client :send-goal goal)
              (send client :wait-for-result)
              (ros::ros-warn "footstep-array: ~A" (send footstep-array :footsteps))
              (ros::ros-warn "send-goal: ~A" goal)))
          (let ((ocs-msg (instance drc_com_common::FC2OCSSmall :init)))
            (send ocs-msg :type drc_com_common::FC2OCSSmall::*EXECUTE_FOOTSTEPS_FINISHED*)
            (send self :send-to-ocs ocs-msg))
          (send smachine :go-to :initial)
          ))))
  (:send-to-ocs (msg)
    (ros::publish "/fc_to_ocs_low_speed/input" msg))
  )

(setq *robot-name* (string-upcase (unix::getenv "ROBOT")))

(generate-robot *robot-name* :generate-ri? t)
(set-drill-environment :draw? nil :generate-ri? t) ;; for init param

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
(ros::advertise "/lookat/input" geometry_msgs::PointStamped)
(ros::advertise "/fisheye/input" geometry_msgs::PointStamped)
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
(ros::advertise "/footstep_planner/project_footprint" jsk_interactive_marker::SnapFootPrintInput)
(ros::advertise "/go_pos_goal" geometry_msgs::PoseStamped)

(ros::subscribe "/fc_from_ocs_low_speed/output" drc_com_common::OCS2FCSmall
  #'send (send *app* :ocs2fc-message-handler) :root-callback)
(ros::subscribe "/go_pos_recognition/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :go-pos-recog-callback)
(ros::subscribe "/lookat/output_passthrough/output" geometry_msgs::PointStamped
  #'send *app* :look-at-recog-callback)
;; (ros::subscribe "/panorama/snapshot/output" geometry_msgs::PoseStamped
;;   #'send *app* :look-at-panorama-recog-callback)
(ros::subscribe "/fisheye/fisheye_click_to_pose/output_point" geometry_msgs::PointStamped
  #'send *app* :look-at-recog-callback)
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
(ros::subscribe "/drill_recognition_for_button/rotate_snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :drill-rotate-recog-callback)
(ros::subscribe "/drill_recognition_for_put/snapshot/output" geometry_msgs::PoseStamped
  #'send *app* :drill-put-recog-callback)
(ros::subscribe "/drill_recognition_for_wall/snapshot/output" geometry_msgs::PoseArray
  #'send *app* :drill-wall-recog-callback)
(ros::subscribe "/footstep_planner/projected_footprint" geometry_msgs::PoseStamped
  #'send *app* :projected-footprint-callback)

(ros::rate 10)

;;(ros::spin)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (send (send *app* :state-machine) :proc)
  )

