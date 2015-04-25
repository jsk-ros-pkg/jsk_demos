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
  (setup-door-open :draw nil :generate-robot nil)
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
    (setq door-worldcoords
          (send (send (send (send (send *robot* :body_lk) :copy-worldcoords) :transform door-coords :local)
                      :rotate (deg2rad 90) :y) :rotate (deg2rad 180) :z))
    ;; fix door handle coords to become horizontal
    ;; :new-x => :new-yと:old-zと垂直
    ;; :new-y => #f(0 0 1)
    ;; :new-z => :new-x, :new-yと垂直
    (let* ((y-vec (float-vector 0 0 1))
           (x-vec (v* y-vec (send door-worldcoords :rotate-vector (float-vector 0 0 1))))
           (z-vec (v* x-vec y-vec)))
      (setq door-worldcoords
            (make-coords :pos (send door-worldcoords :pos) :rot (transpose (matrix x-vec y-vec z-vec)))))
    (send *door* :move-coords door-worldcoords (send *door* :handle :larm-knob-handle))
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
  (test-door-open-drc-sagami-door :wait nil :draw nil)
  ;; (test-door-open-drcbox2-door :walk-p nil :wait nil :draw nil)
  (set-original-abc-st-param-from-door)
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

(defun do-drcbox2-door-motion-for-jaxon
  (&key (real? nil) (draw? t) (wait? t))
  ;; generate model
  (setup-door-open :generate-robot? t :generate-ri? nil :robot-name "JAXON")
  (make-drc-box2-door)
  (objects (list *robot* *door*))
  (setq *av-list* nil)
  (send *robot* :reset-pose)
  (push (send *robot* :angle-vector) *av-list*)
  (setq *init-standing-coords* (make-coords :pos (float-vector -800 -450 0) :rpy (list (deg2rad -80) 0 0)))
  ;; attach handle to robot
  (let* (larm-door-translate-offset
         rarm-door-translate-offset
         larm-door-z-rotate-offset
         rarm-door-z-rotate-offset)
    (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
           (setq larm-door-translate-offset (float-vector 40.0 40.0 0.0))
           (setq rarm-door-translate-offset (float-vector 40.0 -40.0 0.0))
           (setq larm-door-z-rotate-offset (deg2rad 0))
           (setq rarm-door-z-rotate-offset (deg2rad -0))
           )
          (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
           (setq larm-door-translate-offset (float-vector 0 0 0))
           (setq rarm-door-translate-offset (float-vector 0 0 0))
           (setq larm-door-z-rotate-offset 0)
           (setq rarm-door-z-rotate-offset 0)))
    (send *robot* :put :larm-door-coords
          (make-cascoords
           :coords (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate larm-door-translate-offset :local) :rotate larm-door-z-rotate-offset :z :local)
           :parent (send (send *robot* :larm :end-coords) :parent)))
    (send *robot* :put :rarm-door-coords
          (make-cascoords
           :coords (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate rarm-door-translate-offset :local) :rotate rarm-door-z-rotate-offset :z :local)
           :parent (send (send *robot* :rarm :end-coords) :parent))))
  ;; generate reaching motion
  (let* ((trans-list
          (list (list (float-vector -100 200 -150) -60 0)
                (list (float-vector 0 50 -100) -90 0)
                (list (float-vector 0 -25 -50) -90 0)
                (list (float-vector 0 -25 -50) -90 -10)
                (list (float-vector 0 -25 -50) -90 -20)
                )))
    (dolist (trans trans-list)
      (send *robot* :reset-pose)
      (send *robot* :fix-leg-to-coords *init-standing-coords*)
      (send (send *door* :joint :door-hinge-joint) :joint-angle (elt trans 2))
      (with-move-target-link-list
       (mt ll *robot* '(:larm :rleg :lleg))
       (setf (elt mt 0) (send *robot* :get :larm-door-coords))
       (let* ((tc (list (send (send (send (send *door* :handle :larm-knob-handle) :copy-worldcoords)
                                    :translate (elt trans 0) :local)
                              :rotate (deg2rad (elt trans 1)) :z :local)
                        (send *robot* :rleg :end-coords :copy-worldcoords)
                        (send *robot* :lleg :end-coords :copy-worldcoords))))
         (solve-ik-for-do-drcbox2-door-motion-for-jaxon tc mt ll))
       )
      )
    )
  (setq *av-list* (reverse *av-list*))
  ;; send motion to real robot
  (dolist (av *av-list*)
    (send *robot* :angle-vector av)
    (when draw? (send *irtviewer* :draw-objects))
    (when wait? (read-line))
    (when real?
      (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
      (send *ri* :wait-interpolation)))
  ;; (send *robot* :reset-pose)
  ;; (play-motion-on-eus-viewer *av-list* :interpolation? t)
  )

(defun solve-ik-for-do-drcbox2-door-motion-for-jaxon
  (tc mt ll &key (draw? nil))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis (list t t t)
        :rotation-axis (list t t t)
        :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.01 0.15 0.5 0.5)
        :additional-weight-list
        (list (list (send *robot* :torso :waist-r :child-link) 0.1)
              (list (send *robot* :torso :waist-y :child-link) 0.1)
              (list (send *robot* :torso :waist-p :child-link) 0.1))
        :max (float-vector  500  200  25  10  20  10)
        :min (float-vector  -500  -200  -200  -10  -20  -10)
        :stop 50
        :debug-view nil
        :avoid-collision-distance 100
        :avoid-collision-null-gain 5.0
        :avoid-collision-joint-gain 0.8
        :collision-avoidance-link-pair
        (list (list (elt (send *robot* :larm :links) 2) (send *robot* :link "CHEST_LINK2")))
        )
  (push (send *robot* :angle-vector) *av-list*)
  )
