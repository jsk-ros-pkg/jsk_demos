(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/test-drc-door-task.l")


(defun set-door-environment
  (&key (door-knob-size)
        (door-coords (make-coords :pos (float-vector 0 0 1200) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; setup door motion
  (setup-door-open :draw nil)
  ;; generate door
  (setq *door* (make-drc-sagami-door))
  ;;(setq *door* (make-drc-box2-door))
  ;; place door to door-coords
  (when door-coords
    (send *door* :newcoords door-coords))
  (setq *door-stand-coords* nil)
  ;; draw
  (setq *ground* (make-cube 1000 1000 1))
  (when (or draw? (boundp '*irtviewer*))
    (objects (list *robot* *door* *ground*)))
  (when draw?
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-door-environment-full
  (door-knob-size
   door-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (door-worldcoords)
    ;; place robot and door to initial coords
    (set-door-environment :door-knob-size door-knob-size :door-coords nil :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *door* :move-coords
          (send (send (send (send (send *robot* :body_lk) :copy-worldcoords) :transform door-coords :local)
                      :rotate (deg2rad 90) :y) :rotate (deg2rad 180) :z)
          (send *door* :handle :larm-knob-handle))
    (when draw? (send *irtviewer* :draw-objects))
    )
  )

(defun get-door-motion-full
  (knob-size
   coords
   &key (draw? nil) (use-ri? t))
  (set-door-environment-full knob-size coords :draw? draw? :use-ri? use-ri?)
  ;; TODO : implement (generate-door-motion) and (get-door-motion)
  (setq *door-ik-results* nil)
  (setq *door-motion* (list (send *robot* :reset-manip-pose)))
  ;; this is copied from test-drc-door-task.l
  (setq *init-standing-coords* (make-coords :pos (float-vector -600 400 0) :rpy (list (deg2rad -20) 0 0))) ;; for sagami-door
  ;;(setq *init-standing-coords* (make-coords :pos (float-vector -650 -450 0) :rpy (list (deg2rad -15) 0 0))) ;; for drcbox2-door
  (setq *door-stand-coords* (send (send *door* :copy-worldcoords) :transform *init-standing-coords* :local))
  (ros::ros-warn "change stand position to ~a for door motion.~%" *door-stand-coords*)
  (send *robot* :fix-leg-to-coords *door-stand-coords*)
  (when draw? (send *irtviewer* :draw-objects))
  )

(defun send-door-motion
  (knob-size coords &key (real? t))
  (get-door-motion-full knob-size coords)
  ;; reset-manip-pose
  (send *robot* :reset-manip-pose)
  (unless (check-angle-vector-distance)
    (when real? (model2real :time 3000)))
  ;; walk
  (when *door-stand-coords*
    (let* ((pos (scale 0.001 (send *door-stand-coords* :worldpos)))
           (rpy (mapcar #'rad2deg (car (send *door-stand-coords* :rpy-angle)))))
      (ros::ros-warn "go-pos to ~a ~a ~a for door~%" (elt pos 0) (elt pos 1) (elt rpy 0))
      (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
      ))
  ;; set controller param for door
  (setup-door-open-real)
  ;; door motion
  (test-door-open-drc-sagami-door :walk-p nil :wait nil :draw nil)
  ;; (test-door-open-drcbox2-door :walk-p nil :wait nil :draw nil)
  )

(defun publish-door-motion
  (&key knob-size coords (step-time 400) (update? t))
  (when update?
    (get-door-motion-full knob-size coords :use-ri? nil :draw? nil))
  (ros::ros-info "door-stand-coords: ~a~%" *door-stand-coords*)
  (ros::ros-info "door-ik-results: ~a~%" *door-ik-results*)
  (when *door-stand-coords*
    (move-robot-model *door-stand-coords* "ground"))
  (publish-motion-joint-states *door-motion* :step-time step-time)
  )

;;(set-door-environment-full (list 0 0 0) (make-coords :pos (float-vector 642.373 43.578 296.552) :rpy (list -1.343 -0.009 -1.602)))

