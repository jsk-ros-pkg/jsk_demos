#!/usr/bin/env roseus
;; pr2-force-utils.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun wrench->float-vector (msg)
  (float-vector
   (send msg :wrench :force :x)
   (send msg :wrench :force :y)
   (send msg :wrench :force :z)
   (send msg :wrench :torque :x)
   (send msg :wrench :torque :y)
   (send msg :wrench :torque :z)))

(defmethod pr2-interface
  (:wrench-rarm-callback (msg)
   (send self :put :wrench-rarm-callback msg))
  (:wrench-larm-callback (msg)
   (send self :put :wrench-larm-callback msg))
  (:wrench (arm)
   (let ((ns (if (eq arm :rarm) "r_arm_controller" "l_arm_controller"))
         (cb (if (eq arm :rarm) :wrench-rarm-callback :wrench-larm-callback)))
     (send self :put cb nil)
     (with-subscribe ((format nil "~A/wrench" ns) geometry_msgs::WrenchStamped
                      #'send self cb 1)
         (if (wait-until-set 10 (send self :get cb))
             (wrench->float-vector (send self :get cb)) nil))))
  (:wrench-raw (arm)
   (let ((ns (if (eq arm :rarm) "r_arm_controller" "l_arm_controller"))
         (cb (if (eq arm :rarm) :wrench-rarm-callback :wrench-larm-callback)))
     (send self :put cb nil)
     (with-subscribe ((format nil "~A/wrench/raw" ns) geometry_msgs::WrenchStamped
                      #'send self cb 1)
         (if (wait-until-set 10 (send self :get cb))
             (wrench->float-vector (send self :get cb)) nil))))
  (:reset-wrench (arm)
   (call-empty-service
    (format nil "~A/wrench/reset"
            (if (eq arm :rarm) "r_arm_controller" "l_arm_controller")))))

(provide :pr2-force-utils)
