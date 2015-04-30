#!/usr/bin/env roseus

(load "package://hrp2_cmd_vel/src/ps3joy-eus.l")
(load "package://gazebo_drive_simulator/euslisp/atlas-controller-for-drive-simulation.l")

(defclass atlas-drive-handle-cmdvel
  :super ps3joy-eus
  :slots (step interval send-av-flag controller real)
  )
(defmethod atlas-drive-handle-cmdvel
  (:init
   (&key ((:step tmp-step) 5.0) ((:interval tmp-interval) 100)
         (init-pose nil) ((:real tmp-real) t))
   (send-super :init)
   (setq step tmp-step
         interval tmp-interval
         real tmp-real
         send-av-flag nil)
   (setq controller (instance atlas-controller-for-drive-simulation :init
                              :init-pose init-pose))
   self
   )

  (:sensor-cb
   (msg)
   ;(send-super :sensor-cb msg)
   )

  (:button-cb
   (msg)
   (send-super :button-cb msg)
   ;; (let ((buttons (send msg :buttons)))
   ;;   (when (> (elt buttons 9) 0)
   ;;     (send self :start-button)
   ;;     )
   ;;   )
   )
  
  (:axes-cb
   (axes)
   (let ((axv (subseq axes 3 5)))
     (unless (eps= (norm axv) 0.0)
       (when (> (elt axv 0) 0.2)
	 (send self :arrow-l)
         )
       (when (< (elt axv 0) -0.2)
         (send self :arrow-r)
         )
       (when (> (elt axv 1) 0.2)
         (send self :arrow-f)
         )
       (when (< (elt axv 1) -0.2)
	 (send self :arrow-b)
         )
       )
     )
   (when send-av-flag
     (send controller :send-angle-vector-to-real :interval interval :wait nil)
     (setq send-av-flag nil)
     )
   )
  
  (:start-button
   ()
   (send controller :drive-init-pose :real real)
   )

  (:l2-button
   ()
   )

  (:r1-button
   ()
   )

  (:r2-button
   ()
   )

  (:circle-button
   ()
   )
  (:cross-button
   ()
   )
  (:triangle-button
   ()
   )
  (:square-button
   ()
   )
  
  (:arrow-f
   ()
   (send controller :move-camera (- step) :p :real nil)
   (setq send-av-flag t)
   )
  (:arrow-b
   ()
   (send controller :move-camera step :p :real nil)
   (setq send-av-flag t)
   )
  (:arrow-r
   ()
   (send controller :move-camera (- step) :y :real nil)
   (setq send-av-flag t)
   )
  (:arrow-l
   ()
   (send controller :move-camera step :y :real nil)
   (setq send-av-flag t)
   )
  )

;; main function
(defun main ()
  (unless (boundp '*handle-controller-cmdvel*)
    (setq *handle-controller-cmdvel*
          (instance atlas-drive-handle-cmdvel :init))
    )
  (ros::roseus "atlas_handle_controller_test")
  (ros::subscribe
   "/joy"
   sensor_msgs::Joy
   #'send *handle-controller-cmdvel* :joy-cb)
  ;; (ros::subscribe
  ;;   "/cmd_vel"
  ;;   geometry_msgs::Twist
  ;;   #'send *hrp4r-drive-cmdvel* :cmd-vel-cb)
  (objects *atlas*)
  (ros::rate 10)
  (do-until-key
   (ros::spin-once)
   (warn "~A ~A~%" (send *atlas* :head :neck-p :joint-angle) (send *atlas* :torso :waist-y :joint-angle))
   (send *viewer* :draw-objects)
   (ros::sleep)
   )
  )

(main)
