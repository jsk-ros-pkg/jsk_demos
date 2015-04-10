#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(defclass robot-driving-controller
  :super propertied-object
  :slots (motion
          ros-rate 
          step-accel-angle step-brake-angle
          release-accel-angle release-brake-angle
          is-handle-reached-to-target
          grasped-flag accel-flag
          max-turn-angle max-turn-angle-diff-deg
          old-handle-deg
          polygon-stamped-msg
          estimated-robot-coords
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
         )
   ;; set defalut value for pedaling
   (setq step-accel-angle 28
         release-accel-angle 14
         step-brake-angle 20
         release-brake-angle 5
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
   (ros::subscribe "/rfsensor"
                   geometry_msgs::WrenchStamped
                   #'send self :rfsensor-callback)
   (ros::subscribe "/drive/torus_finder/output/pose"
                   geometry_msgs::PoseStamped
                   #'send self :torus-finder-callback)
   (ros::advertise "/drive/pedal_state" std_msgs::Bool 1)
   (ros::advertise "/drive/torus_finder/input/polygon" geometry_msgs::PolygonStamped 1)
   )

  ;; handle command
  (:grasp
   ()
   (send motion :grasp-handle)
   (setq grasped-flag t)
   )
  (:turn
   (angle &key (weight 1.0) (rad nil) (supress-diff-max t))
   ;; turn can be used only when grasped
   (unless grasped-flag
     (warn "Robot do not grasp handle~%")
     (return-from :turn)
     )   
   (let (handle-deg real-handle-deg)
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
     (send motion :turn-handle real-handle-deg :relative nil :use-controller t)
     (setq old-handle-deg real-handle-deg)
     )
   )
  ;; approach
  (:approach-handle
   (&key (sync nil))
   (send motion :approach-handle :sync-controller sync)
   (setq grasped-flag nil)
   )
  (:approach-accel
   (&key (sync nil))
   (send motion :approach-accel-pedal)
   )
  (:approach-brake
   (&key (sync nil))
   (send motion :approach-brake-pedal)
   )

  ;; pedal command
  (:step-accel
   (&optional (angle step-accel-angle)) 
   (send motion :step-accel-pedal angle :relative nil :use-controller t) ;; step accel
   )
  (:release-accel
   (&optional (angle release-accel-angle))
   (send motion :step-accel-pedal angle :relative nil :use-controller t) ;; init accel
   )
  (:step-brake
   (&optional (angle step-brake-angle))
   (send motion :step-brake-pedal angle :relative nil :use-controller t) ;; step brake
   )
  (:release-brake
   (&optional (angle release-brake-angle))
   (send motion :step-brake-pedal angle :relative nil :use-controller t) ;; init brake
   )

  (:accel-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-angle-from-cmd cmd release-accel-angle step-accel-angle)))
     (warn "accel ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     ;; (send self :step-accel cmd-angle)
     (send motion :step-accel-pedal cmd-angle :relative nil
	   :time 700 :no-wait t :use-controller t) ;; step accel
     )
   )
  (:brake-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-angle-from-cmd cmd release-brake-angle step-brake-angle)))
     (warn "brake ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     (send self :step-brake cmd-angle)
     )
   )

  ;; operator callback
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
  
  (:grasp-callback
   (msg)
   (let ((data (send msg :data)))
     (cond
      ((equal data "grasp")
       (warn "receive grasp~%")
       (send self :grasp)
       )
      ((equal data "approach")
       (warn "receive approach~%")
       (send self :approach-handle)
       )
      (t nil)
      )
     )
   )
  (:handle-callback
   (msg)
   (let* ((data (send msg :data))) ;; data is estimated as radius
       (send self :turn data :rad t :supress-diff-max t)
     )
   )

  ;; sensor feedback
  (:rfsensor-callback
   (msg)
   (let ((wrench (send msg :wrench)))
     (setq rfsensor-value wrench)
     )
   )
  (:adjust-accel-pedal
   (&optional (thre 10.0) (step 1.0))
   (unless rfsensor-value
     (warn "no rfsensor values~%")
     (return-from :adjust-accel-pedal)
     )
   (let ((force (send rfsensor-value :force)))
     (do-until-key
      (when (> (norm force) thre) ;; finish
        (warn "Touch detected~%")
        (send motion :step-accel-pedal (- step) :relative t)
        (send *interface* :release-accel-angle (send *staro* :rleg :ankle-p :joint-angle)) ;; todo: avoid hard-cording of robot name and robot joint
        (send self :min-accel-angle (send *interface* :release-accel-angle)) 
        (return-from :adjust-accel-pedal t)
        )
      (send motion :step-accel-pedal step :relative t)
      (ros::spin-once)
      (ros::sleep)
      )
     nil
     )
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
  
  ;; accessor
  (:controller () controller)

  ;; setter
  (:step-accel-angle (&optional (angle nil)) (when angle (setq step-accel-angle angle)) step-accel-angle)
  (:release-accel-angle (&optional (angle nil)) (when angle (setq release-accel-angle angle)) release-accel-angle)
  (:step-brake-angle (&optional (angle nil)) (when angle (setq step-brake-angle angle)) step-brake-angle)
  (:release-brake-angle (&optional (angle nil)) (when angle (setq release-brake-angle angle)) release-brake-angle)
  (:max-turn-angle (&optional (val nil)) (when val (setq max-turn-angle val)) max-turn-angle)
  (:max-turn-angle-diff (&optional (val nil)) (when val (setq max-turn-angle-diff-deg val)) max-turn-angle-diff-deg)
  (:grasped-flag (&optional (val 0)) (unless (equal val 0) (setq grasped-flag val)) grasped-flag)
  (:accel-flag (&optional (val 0)) (unless (equal val 0) (setq accel-flag val)) accel-flag)
  
  (:execute
    ()
    (let ((count 0))
      (do-until-key
       (when (> count 10)
         (warn "current-angle: ~A~%" (send motion :estimate-current-handle-angle))
         (setq count 0)
         )
       (ros::spin-once)
       (ros::sleep)
       (setq count (+ count 1))
       )
      )
    )
  )
