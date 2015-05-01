#!/usr/bin/env roseus

(setq *atlas-name* :atlas-v0) ;; use atlas_v0 model
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(defclass atlas-controller-for-drive-simulation
  :super object
  :slots (robot)
  )
(defmethod atlas-controller-for-drive-simulation
  (:init
   (&key (init-pose nil))   
   (atlas-init)
   (setq robot *atlas*)
   (send robot :angle-vector (send *ri* :state :potentio-vector)) ;; trace joint angles of real atlas 
   (when init-pose ;; send init-pose when startup
     (send self :drive-init-pose :real t)
     )
   self
   )
  (:drive-init-pose
   (&key (real nil) (wait nil))
    (let ((init-pose (float-vector 0.0 3.0 0.0 ;; torso
                                   120.0 100.0 5.0 -65.0 35.0 90.0 ;; larm
                                   -120.0 100.0 -5.0 65.0 35.0 90.0 ;; rarm
                                   55.0 0.0 -60.0 25.0 0.0 30.0 ;; lleg
                                   55.0 0.0 -60.0 -25.0 0.0 -30.0 ;; rleg
                                   0.0 ;; head
                                   )))
      (send robot :angle-vector init-pose)
      (when real
        (send self :send-angle-vector-to-real :wait wait)
        )
      )
    )
  (:move-camera
   (degree direction &key (real nil) (wait nil))
   (case direction
     (:r nil) ;; roll
     (:p (send robot :head :neck-p :joint-angle degree :relative t)) ;; pitch
     (:y (send robot :torso :waist-y :joint-angle degree :relative t)) ;; yaw
     )
   (when real
     (send self :send-angle-vector-to-real :wait wait)
     )
   )
  (:send-angle-vector-to-real
   (&key (wait nil) (interval 3000))
   (send* *ri* :angle-vector (send robot :angle-vector) interval)
   (when wait
     (send *ri* :wait-interpolation)
     )
   )
  (:robot () robot)
  )

(defun main
  ()
  (setq *controller* (instance atlas-controller-for-drive-simulation :init))
  )
