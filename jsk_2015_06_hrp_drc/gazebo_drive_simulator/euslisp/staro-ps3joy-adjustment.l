#!/usr/bin/env roseus

(load "package://hrp2_cmd_vel/src/ps3joy-eus.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")

(defclass staro-ps3joy-adjustment
  :super ps3joy-eus
  :slots (robot step joint-list enabled-flag
          target-limb target-joint)
  )
(defmethod staro-ps3joy-adjustment
  (:init
   (&key ((:robot tmp-robot) nil) (topic-name "/ps3joy/joy")
         (roseus-init nil) ((:ros-rate tmp-ros-rate) 10)
         )
   (send-super :init)
   (when roseus-init
     (ros::roseus "staro_ps3joy_adjustment")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-robot
     (staro-init)
     (setq tmp-robot *staro*)
     )
   (ros::subscribe
    topic-name
    sensor_msgs::Joy
    #'send self :joy-cb)
   (setq robot tmp-robot
         joint-list (list :crotch-p :knee-p :ankle-p)
         step 0.5
         enabled-flag nil
         )
   (send self :set-target-limb :rleg)
   (send self :set-target-joint (elt joint-list 0))
   (send self :copy-real-robot-angle-vector)
   self
   )

  (:axes-cb
    (axes)
    (let ((axv (subseq axes (cadr (assoc :arrow-f key-mapping)) (1+ (cadr (assoc :arrow-l key-mapping))))))
      (unless (eps= (norm axv) 0.0)
        (when (> (abs (elt axv 0)) 0.2)
          (send self :arrow-f-button)
          )
        (when (> (abs (elt axv 1)) 0.2)
          (send self :arrow-r-button)
          )
        (when (> (abs (elt axv 2)) 0.2)
          (send self :arrow-b-button)
          )
        (when (> (abs (elt axv 3)) 0.2)
          (send self :arrow-l-button)
          )
        )
      )
    )

  ;; toggle l/r
  (:l2-button
   ()   
   (send self :set-target-limb :lleg)
   (send self :copy-real-robot-angle-vector)
   )
  (:r2-button
   ()
   (send self :set-target-limb :rleg)
   (send self :copy-real-robot-angle-vector)
   )
  
  ;; modify joint angle
  (:arrow-f-button
   ()
   (send self :modify-joint-angle step)
   )
  (:arrow-b-button
   ()
   (send self :modify-joint-angle (- step))
   )

  ;; toggle joint angle
  (:arrow-l-button
   ()
   (let (index old-target-joint new-target-joint)
     (setq old-target-joint target-joint)
     (setq index (1- (position old-target-joint joint-list)))
     (if (< index 0)
         (send self :set-target-joint (elt joint-list (1- (length joint-list)))) ;; back to end
       (send self :set-target-joint (elt joint-list index))
       )
     )
   )

  (:arrow-r-button
   ()
   (let (index old-target-joint new-target-joint)
     (setq old-target-joint target-joint)
     (setq index (1+ (position old-target-joint joint-list)))
     (if (> index (1- (length joint-list)))
         (send self :set-target-joint (elt joint-list 0)) ;; back to first
       (send self :set-target-joint (elt joint-list index))
       )
     )
   )

  ;; apply
  (:circle-button
   ()
   (warn "Apply joint angles~%")
   (send self :apply-joint-angles)
   )
  ;; reset
  (:cross-button
   ()
   (warn "Reset joint angles~%")
   (send self :copy-real-robot-angle-vector)
   )

  ;; joint angle controllers (works only when enabled-flag is t)
  (:modify-joint-angle
   (diff-joint-angle)
   (let (old-joint-angle)
     (if enabled-flag
         (progn (setq old-joint-angle (send robot target-limb target-joint :joint-angle))
                (send robot target-limb target-joint :joint-angle diff-joint-angle :relative t)
                (warn "Change ~A ~A from ~A to ~A~%" target-limb target-joint
                      old-joint-angle (send robot target-limb target-joint :joint-angle))
                (send *viewer* :draw-objects)
                )
       (warn "Joint angle adjustment is not enabled~%")
       )
     )
   )
  (:copy-real-robot-angle-vector
   ()
   (if enabled-flag
       (progn (send robot :angle-vector (send *ri* :state :potentio-vector))
              (send *viewer* :draw-objects)
              )
     (warn "Joint angle adjustment is not enabled~%")
     )
   )
  (:apply-joint-angles
   ()
   (let (controller)
     (if enabled-flag
         (progn (case target-limb
                  (:lleg (setq controller :lleg-controller))
                  (:rleg (setq controller :rleg-controller))
                  )
                (send *ri* :angle-vector (send *staro* :angle-vector) 20000 controller)
                (send *ri* :wait-interpolation)
                )
       (warn "Joint angle adjustment is not enabled~%")
       )
     )
   )

  ;; accessor
  (:set-target-limb
   (limb)
   (warn "Change target limb to ~A~%" limb)
   (setq target-limb limb)
   )
  (:set-target-joint
   (joint)
   (warn "Change target joint to ~A~%" joint)
   (setq target-joint joint)
   )
  (:enabled-flag
   (&optional (val 0))
   (unless (equal val 0)
     (setq enabled-flag val)
     )
   enabled-flag
   )

  ;; execute
  (:execute
   ()
   (do-until-key
    (ros::spin-once)
    (ros::sleep)
    )   
   )
  
  )

(defun main
  ()
  (unless (boundp '*joy*)
    (setq *joy* (instance staro-ps3joy-adjustment :init)))  
  (ros::rate 10)
  (send *joy* :execute)
  )
