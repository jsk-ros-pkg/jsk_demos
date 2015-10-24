#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/door-open/subscribe_checkerboard.l")

(generate-robot "hrp2jsknt" :generate-ri? t)
(get-target-coords)
(unix:sleep 3)
(get-target-coords)

(print "preparation has been done.")
(objects (list *robot*))
(defun look-at-door-knob()
  (get-target-coords)
  (send *robot* :angle-vector (send *ri* :potentio-vector))
  (send *robot* :look-at-target (get-target-coords))
  (send *ri* :set-joint-angles-of-group "head_for_teleop" 
        (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 
        0.5)
  (send *ri* :wait-interpolation-of-group "head_for_teleop")
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  )
;;(send *ri* :remove-joint-group "head_for_teleop")
(send *ri* :add-joint-group "head_for_teleop" (send-all (send *robot* :head :joint-list) :name))
(send *ri* :set-joint-angles-of-group "head_for_teleop" 
      (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 
      1)
(send *ri* :wait-interpolation-of-group "head_for_teleop")




(do-until-key (look-at-door-knob))

