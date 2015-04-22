#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(ros::roseus-add-srvs "std_srvs")

;; for drc_task_common
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "drc_task_common")

(defclass robot-driving-controller
  :super propertied-object
  :slots (motion
          ros-rate 
          step-accel-command step-brake-command
          release-accel-command release-brake-command
          is-handle-reached-to-target
          grasped-flag accel-flag
          max-turn-angle max-turn-angle-diff-deg
          old-handle-deg
          polygon-stamped-msg
          estimated-robot-coords
          old-estimated-handle-angle
          ) 
  )

(defmethod robot-driving-controller
  (:init
   (&key ((:real tmp-real) nil)
         ((:motion tmp-motion) nil)
         ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         (approach-handle nil) (approach-pedal nil)
         ((:max-turn-angle tmp-max-turn-angle) 45.0)
         ((:max-turn-angle-diff tmp-max-turn-angle-diff) 15.0)
         )
   (when roseus-init
     (ros::roseus "robot_driving_controller")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance robot-driving-motion :init :real tmp-real))
     )
   (setq *motion* tmp-motion)
   (setq motion *motion*)
   (when approach-handle
     (send motion :approach-handle)
     )
   (when approach-pedal
     (send motion :approach-accel-pedal)
     (send motion :approach-brake-pedal)
     )
   ;; initialize instance valiables
   (setq ros-rate tmp-ros-rate
         grasped-flag nil
         accel-flag t
         max-turn-angle tmp-max-turn-angle
         max-turn-angle-diff-deg tmp-max-turn-angle-diff
         old-handle-deg 0.0
         old-estimated-handle-angle 0.0
         )
   ;; set defalut value for pedaling
   (setq step-accel-command 28
         release-accel-command 14
         step-brake-command 20
         release-brake-command 5
         )
   ;; subscriber for operator command
   (ros::subscribe "/drive/operation/accel_cmd"
                   std_msgs::Float64
                   #'send self :accel-callback)
   (ros::subscribe "/drive/operation/brake_cmd"
                   std_msgs::Float64
                   #'send self :brake-callback)
   (ros::subscribe "/drive/operation/grasp_cmd"
                   std_msgs::String
                   #'send self :grasp-callback)
   (ros::subscribe "/drive/operation/handle_cmd"
                   std_msgs::Float64
                   #'send self :handle-callback)
   (ros::advertise "/drive/controller/pedal_state" std_msgs::Bool 1)
   ;; handle-angle commands
   (ros::advertise "/drive/controller/goal_handle_angle" std_msgs::Float32 1)
   (ros::advertise "/drive/controller/estimated_handle_angle" std_msgs::Float32 1)
   ;; for steering estimation
   (ros::subscribe "/drive/torus_finder/output/pose"
                   geometry_msgs::PoseStamped
                   #'send self :torus-finder-callback)
   (ros::advertise "/drive/torus_finder/input/polygon" geometry_msgs::PolygonStamped 1)

   ;; for vehicle ui
   (ros::advertise-service "/drive/controller/initialize" std_srvs::Empty #'send self :initialize-callback)
   (ros::advertise-service "/drive/controller/grasp" std_srvs::Empty #'send self :grasp-callback)
   (ros::advertise-service "/drive/controller/release" std_srvs::Empty #'send self :release-callback)
   (ros::advertise-service "/drive/controller/correct" std_srvs::Empty #'send self :correct-callback)
   (ros::advertise-service "/drive/controller/set_max_step" drc_task_common::SetValue #'send self :set-max-step-callback)
   (ros::advertise-service "/drive/controller/set_min_step" drc_task_common::SetValue #'send self :set-min-step-callback)
   )

  ;; vehicle ui callback
  (:initialize-callback
   (req)
   (let ((res (send req :response)))
     (send self :initialize)
     res
     )
   )
  (:grasp-callback
   (req)
   (let ((res (send req :response)))
     (send self :grasp)
     res
     )
   )
  (:release-callback
   (req)
   (let ((res (send req :response)))
     (send self :release)
     res
     )
   )
  (:correct-callback
   (req)
   (let ((res (send req :response)))
     (send self :correct)
     res
     )
   )
  (:set-max-step-callback
   (req)
   (let ((res (send req :response)))
     (send res :set_value (send self :step-accel-command (send req :value)))
     (warn "set step-accel-command to ~A~%" (send self :step-accel-command))
     res
     )
   )
  (:set-min-step-callback
   (req)
   (let ((res (send req :response)))
     (send res :set_value (send self :release-accel-command (send req :value)))
     (warn "set release-accel-command to ~A~%" (send self :release-accel-command))
     res
     )
   )

  ;; interface command
  (:initialize
   (&key (sync nil))
   (when sync
     (send motion :sync-all-controller)
     )
   (send motion :ride :init-pose t)
   (send motion :drive-init-pose-crank :limb :larm :move-arm nil) ;; todo: fix and remove
   (send self :approach-accel)
   (send self :approach-floor)
   (send self :approach-handle)
   nil
   )
  (:grasp
   ()
   (send motion :grasp-handle)
   (setq grasped-flag t)
   ;; handle-angle is estimated to be right when grasped
   (setq old-estimated-handle-angle (send motion :car :handle-joint :joint-angle))
   )
  (:release
   ()
   (send motion :open-hand)
   (setq grasped-flag nil)
   )
  (:correct
   ()
   (warn "not implemented yet~%")
   )
  (:turn
   (angle &key (mode :sequence) (weight 1.0) (rad nil) (supress-diff-max nil))
   ;; turn can be used only when grasped
   (unless grasped-flag
     (warn "Robot do not grasp handle~%")
     (return-from :turn)
     )
   (let ((pub-msg (instance std_msgs::Float32 :init))
         handle-deg real-handle-deg)
     ;; if radius, transform to deg
     (if rad
         (setq handle-deg (/ (rad2deg angle) weight))
       (setq handle-deg (/ angle weight))
       )
     ;; supress in min-max angle
     (if (> handle-deg max-turn-angle)
         (setq handle-deg max-turn-angle)
       (if (< handle-deg (- max-turn-angle))
         (setq handle-deg (- max-turn-angle))
         )
       )
     ;; supress difference max
     (let ((turn-angle-diff-deg (- handle-deg old-handle-deg)))
       (if (and (> (abs turn-angle-diff-deg) max-turn-angle-diff-deg)
                supress-diff-max)
           (progn (setq real-handle-deg
                        (if (> turn-angle-diff-deg 0)
                            (+ old-handle-deg max-turn-angle-diff-deg)
                          (- old-handle-deg max-turn-angle-diff-deg)
                          )
                        )
                  (warn "max-diff-angle limitation ~A [deg] was enabled:~% original: ~A~% real: ~A~% "
                        max-turn-angle-diff-deg handle-deg real-handle-deg)
                  )
         (setq real-handle-deg handle-deg)
         )
       )
     (warn "turn ~A[deg]~%" real-handle-deg)

     ;; turn-handle using motion
     (case mode
       (:once (send motion :turn-handle-once real-handle-deg :relative nil :use-controller t))
       (:sequence (send motion :turn-handle real-handle-deg :relative nil :use-controller t))
       (t (send motion :turn-handle real-handle-deg :relative nil :use-controller t))
       )
       
     (setq old-handle-deg real-handle-deg)
     ;; publish target angle
     (send pub-msg :data real-handle-deg)
     (ros::publish "/drive/controller/goal_handle_angle" pub-msg)
     )
   )
  (:approach-handle
   (&key (sync grasped-flag))
   (send motion :approach-handle :sync-controller sync)
   (setq grasped-flag nil)
   )
  (:approach-accel
   ()
   (send motion :approach-accel-pedal)
   )
  (:approach-brake
   ()
   (send motion :approach-brake-pedal)
   )
  (:approach-floor
   ()
   (send motion :approach-floor :offset #f(0 0 50)
         :start-impedance t :stop-ref-force t :start-ref-force nil) ;; approach to floor
   )
  (:step-accel
   (command)
   (send motion :step-accel-pedal command :relative nil :use-controller t) ;; step accel
   )
  (:detatch-accel
   ()
   (send motion :detatch-accel-pedal :use-controller t) ;; detatch accel for emergency
   )
  (:step-brake
   (command)
   (send motion :step-brake-pedal command :relative nil :use-controller t) ;; step brake
   )
  (:support-by-leg
   ()
   (send motion :approach-floor :offset #f(0 0 0)
         :start-impedance nil :stop-ref-force nil :start-ref-force t) ;; approach to floor
   )

  ;; internal functions
  (:accel-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-angle-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     (send self :step-accel cmd-angle)
     )
   )
  (:brake-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-angle-from-cmd cmd release-brake-command step-brake-command)))
     (warn "brake ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     (send self :step-brake cmd-angle)
     )
   )

  ;; handle controller callback
  (:handle-callback
   (msg)
   (let* ((data (send msg :data))) ;; data is estimated as radius
       (send self :turn data :rad t :mode :once)
     )
   )
  (:accel-callback
   (msg)
   (if accel-flag
       (send self :accel-cmd (send msg :data))
     (warn "accel-cmd is disabled~%")
     )
   (let ((pub-msg (instance std_msgs::Bool :init)))
     (send pub-msg :data accel-flag)
     (ros::publish "/drive/pedal_state" pub-msg)
     )
   )
  (:brake-callback
   (msg)
   (send self :brake-cmd (send msg :data))
   (send self :accel-cmd 0.0) ;; release accel
   )
  (:get-pedal-angle-from-cmd
   (data min-pedal-angle max-pedal-angle) ;; data will be 0.0 from 1.0
   (+ min-pedal-angle (* data (- max-pedal-angle min-pedal-angle)))
   )

  ;; steering estimation
  (:estimate-steering
   (&key (limb :larm))
   (let* ((tmp-robot (copy-object (send motion :robot)))
          (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
          (steering-coords (send motion :car :vehicle-handle :steering-center))
          header polygon
          body-relative-ec ec-relative-steering-coords tmp-ec tmp-pos tmp-point32 tmp-point32-list
          end-coords-list)
     ;; collect end-coords of real robot
     (setq polygon-stamped-msg (instance geometry_msgs::PolygonStamped :init))
     (setq header (send polygon-stamped-msg :header))
     (setq polygon (send polygon-stamped-msg :polygon))
     (warn "start recording~%")
     (do-until-key
       (send tmp-robot :angle-vector (send *ri* :state :angle-vector))
       (push (send (send tmp-robot limb :end-coords) :copy-worldcoords) end-coords-list)
       )
     (warn "finish recording~%")
     ;; translate end-coords: worldcoords of grasp-point -> body related coords of steering-point
     (dolist (ec (reverse end-coords-list))
       (setq tmp-ec (send ec :copy-worldcoords))
       (setq ec-relative-steering-coords (send tmp-ec :transformation steering-coords :local))
       (send tmp-ec :translate (float-vector 0 0 (elt (send ec-relative-steering-coords :pos) 2)) :local) ;; project end-coords(estimated as grasp-point) to same plane as steering
       (setq body-relative-ec (send body-coords :transformation tmp-ec :local)) ;; make relative coords from body-link to end-coords
       (setq tmp-pos (send body-relative-ec :pos))
       ;; (setq tmp-pos (send ec :pos))
       (setq tmp-point32 (instance geometry_msgs::Point32 :init))
       (setq tmp-pos (scale 0.001 tmp-pos))
       (send tmp-point32 :x (elt tmp-pos 0))
       (send tmp-point32 :y (elt tmp-pos 1))
       (send tmp-point32 :z (elt tmp-pos 2))
       (push tmp-point32 tmp-point32-list)
       )
     ;; publish
     (warn "publish data~%")
     (send polygon :points tmp-point32-list)
     (send header :stamp (ros::time-now))
     (send header :frame_id "BODY")
     (ros::publish "/drive/torus_finder/input/polygon" polygon-stamped-msg)
     (ros::spin-once)
     (ros::sleep)
     )
   )
  (:torus-finder-callback
   (msg)
   (warn "Update handle coordinates according to torus_finder~%")
   (let ((pose (send msg :pose)) (tmp-robot (copy-object (send motion :robot)))
         (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
         (steering-coords (send motion :car :vehicle-handle :steering-center))
         body-relative-steering-coords real-steering-worldcoords steering-relative-body-coords real-body-worldcoords)
     ;; (warn "old: ~A~%" (send motion :car :vehicle-handle :copy-worldcoords))
     (warn "old: ~A~%" (send tmp-robot :copy-worldcoords))
     (setq body-relative-steering-coords (make-coords
                                          :pos (scale 1000 (float-vector (send pose :position :x) (send pose :position :y) (send pose :position :z))) ;; ros[m] -> eus[mm]
                                          :rot (quaternion2matrix (float-vector (send pose :orientation :w) (send pose :orientation :x)
                                                                                (send pose :orientation :y) (send pose :orientation :z)))
                                          )
           )
     (warn "trans: ~A~%" body-relative-steering-coords)
     ;; (setq real-steering-worldcoords (send body-coords :transform body-relative-steering-coords :local))
     ;; (warn "steering: ~A~%" real-steering-worldcoords)
     (setq steering-relative-body-coords (send body-relative-steering-coords :inverse-transformation))
     (setq real-body-worldcoords (send steering-coords :transform steering-relative-body-coords :local))
     (warn "new body: ~A~%" real-body-worldcoords)
     (send tmp-robot :transform (send body-coords :transformation real-body-worldcoords :world) :world)
     (setq estimated-robot-coords (send tmp-robot :copy-worldcoords))
     (warn "new robot: ~A~%" estimated-robot-coords)
     ;; (send robot :transform (send steering-coords :transformation real-steering-worldcoords :world) :world)
     ;; (send motion :car :vehicle-handle :transform (send steering-coords :transformation real-steering-worldcoords :world) :world)
     ;; (send motion :car :vehicle-handle :newcoords  body-relative-handle-coords)
     )
   )
  (:republish-steering-trajectory
   ()
   (ros::publish "/drive/torus_finder/input/polygon" polygon-stamped-msg)
   )
  (:apply-steering-estimation
   ()
   (ros::spin-once)
   (unless estimated-robot-coords
       (warn "steering estimation was not executed~%")
       (return-from :apply-steering-estimation nil)
       )
   (let ((old-robot-coords (send motion :robot :copy-worldcoords))
          )
     (send motion :robot :newcoords estimated-robot-coords)
     (send *viewer* :draw-objects)
     (unless (y-or-n-p)
       (send motion :robot :newcoords old-robot-coords) ;; revert
       )
     (send motion :robot :copy-worldcoords)
     )
   )
  (:estimate-current-handle-angle
   ()
   (let ((current-angle old-estimated-handle-angle)
         (half-range 180)
         (pub-msg (instance std_msgs::Float32 :init))
         current-end-coords steering-center-coords-at-zero-deg
         steering-center-relative-end-coords relative-current-pos-from-steerint-center estimated-angle
         steering-center-relative-grasp-point relative-grasp-point-pos-from-steerint-center estimated-angle-offset
         tmp-angle-vector
         )
     ;; preserve current angle-vector
     (setq tmp-angle-vector (send (send motion :robot) :angle-vector))
     ;; get current end-coords and resume angle-vector
     (send (send motion :robot) :angle-vector (send *ri* :state :potentio-vector))
     (setq current-end-coords (send (send motion :robot) (send motion :grasp-arm) :end-coords :copy-worldcoords))
     (send (send motion :robot) :angle-vector tmp-angle-vector)
     ;; fix steering-center coords to 0[deg], handle-joint is counter-clockwise
     (setq steering-center-coords-at-zero-deg (send (send (send motion :car :vehicle-handle :steering-center) :copy-worldcoords)
                                                    :rotate (deg2rad (- (send motion :car :handle-joint :joint-angle))) :z :local))
     ;; get relative-coords from steering-center
     (setq steering-center-relative-end-coords
           (send (send steering-center-coords-at-zero-deg :copy-worldcoords) :transformation current-end-coords :local))
     ;; get relative angle from polar-angle
     (setq relative-current-pos-from-steerint-center (send steering-center-relative-end-coords :pos))
     (setq estimated-angle (rad2deg (atan (elt relative-current-pos-from-steerint-center 1) (elt relative-current-pos-from-steerint-center 0))))
     ;; calculate offset angle from relative-coords of grasp-point
     (setq steering-center-relative-grasp-point
           (send (send (send motion :car :vehicle-handle :steering-center) :copy-worldcoords) :transformation (send (send motion :car (send motion :grasp-point)) :copy-worldcoords) :local))
     (setq relative-grasp-point-pos-from-steerint-center (send steering-center-relative-grasp-point :pos))
     (setq estimated-angle-offset (rad2deg (atan (elt relative-grasp-point-pos-from-steerint-center 1) (elt relative-grasp-point-pos-from-steerint-center 0))))
     (setq estimated-angle (+ estimated-angle estimated-angle-offset)) ;; current estimated angle is 0-360
     (let ((counter 0))
       (while (not (and (<= (- old-estimated-handle-angle half-range) estimated-angle) (<= estimated-angle (+ old-estimated-handle-angle half-range))))
         (when (< estimated-angle (- old-estimated-handle-angle half-range))
           (setq estimated-angle (+ estimated-angle (* half-range 2)))
           )
         (when (> estimated-angle (+ old-estimated-handle-angle half-range))
           (setq estimated-angle (- estimated-angle (* half-range 2)))
           )
         (when (> counter 10)
           (warn "cannnot estimate~%")
           (return-from :estimate-current-handle-angle nil)
           )
         (setq counter (+ counter 1))
         )
       )
     (setq old-estimated-handle-angle estimated-angle)
     ;; publish estimated angle
     (send pub-msg :data estimated-angle)
     (ros::publish "/drive/controller/estimated_handle_angle" pub-msg)
     estimated-angle
     )
   )
  ;; accessor
  (:motion () motion)

  ;; setter
  (:step-accel-command (&optional (angle nil)) (when angle (setq step-accel-command angle)) step-accel-command)
  (:release-accel-command (&optional (angle nil)) (when angle (setq release-accel-command angle)) release-accel-command)
  (:step-brake-command (&optional (angle nil)) (when angle (setq step-brake-command angle)) step-brake-command)
  (:release-brake-command (&optional (angle nil)) (when angle (setq release-brake-command angle)) release-brake-command)
  (:max-turn-angle (&optional (val nil)) (when val (setq max-turn-angle val)) max-turn-angle)
  (:max-turn-angle-diff (&optional (val nil)) (when val (setq max-turn-angle-diff-deg val)) max-turn-angle-diff-deg)
  (:grasped-flag (&optional (val 0)) (unless (equal val 0) (setq grasped-flag val)) grasped-flag)
  (:accel-flag (&optional (val 0)) (unless (equal val 0) (setq accel-flag val)) accel-flag)

  ;; main loop
  (:execute
   ()
   (let ((count 0))
     (do-until-key
      (send self :estimate-current-handle-angle)
      (ros::spin-once)
      (ros::sleep)
      )
     )
   )
  )
