(load "~/prog/euslib/irteus_proposals/motion-lib-proposal.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defun setup-door-open (&key (draw? t) (generate-robot? t) (generate-ri? t) (robot-name "HRP2JSKNT"))
  (when generate-robot?
    (cond (robot-name
           (setq *robot-name* robot-name))
          (t ;;(unix::getenv "ROBOT")
           (setq *robot-name* (string-upcase (unix::getenv "ROBOT")))))
    (generate-robot *robot-name* :generate-ri? generate-ri?)
    )
  (when draw? (objects (list *robot*)))
  ;; initialize end-coords for door-grasp
  (mapcar #'(lambda (arm trs)
              (let ((mc (make-cascoords
                         :coords (send (send *robot* arm :end-coords :copy-worldcoords)
                                       :translate trs)
                         :name (read-from-string (format nil "~A-tip-grasp-coords" arm)))))
                (send *robot* :put (read-from-string (format nil "~A-tip-grasp-coords" arm)) mc)
                (send (send (send *robot* arm :end-coords) :parent) :assoc mc)
                ))
          '(:rarm :larm)
          (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))          
                 (list (float-vector 40 -40 0) (float-vector 40 40 0)))
                (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
                 (list (float-vector 50 -25 0) (float-vector 50 25 0))))
          )
  (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (setq *rarm-avoid-pose* #f(0.0 40.0 -20.0 -5.0 -80.0 0.0 0.0 -20.0))
         (setq *larm-avoid-pose* #f(0.0 40.0 20.0 5.0 -80.0 0.0 0.0 -20.0))
         (setq *rarm-reset-manip-pose* #f(0.0 55.0 -20.0 -15.0 -100.0 -25.0 0.0 -45.0))
         (setq *larm-reset-manip-pose* #f(0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
         )
        (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (setq *rarm-avoid-pose* #f(40.1007 -29.388 4.91744 -71.6494 -22.7883 -2.70865 7.77037 15.0))
         (setq *larm-avoid-pose* #f(40.1007 29.388 -4.91744 -71.6494 22.7883 2.70865 7.77037 15.0))
         (setq *rarm-reset-manip-pose* #f(50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0))
         (setq *larm-reset-manip-pose* #f(50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
         ))
  (setq *door-grasp-preshape-pose* #f(0.0 70.0 0.0 -10.0 30.0 30.0))
  (setq *door-grasp-pose* #f(0.0 -20.0 10.0 -10.0 120.0 120.0))
  )
;;(warn ";; (setup-door-open)~%")

(defun setup-door-open-real (&key (start-abc-st t))
  ;; ABC, ST, IMP param
  ;; save original parameter of ABC and ST
  (let* ((gait-generator-param (send (send *ri* :get-gait-generator-param) :slots))
         (auto-balancer-param (send (send *ri* :get-auto-balancer-param) :slots)))
    (setq *original-default-step-time*
          (cdr (assoc 'ros::_default_step_time gait-generator-param)))
    (setq *original-default-double-support-ratio*
          (cdr (assoc 'ros::_default_double_support_ratio gait-generator-param)))
    (setq *original-default-zmp-offsets*
          (list (subseq (scale 1000 (send (cdr (assoc 'ros::_default_zmp_offsets auto-balancer-param)) :data)) 0 3)
                (subseq (scale 1000 (send (cdr (assoc 'ros::_default_zmp_offsets auto-balancer-param)) :data)) 3 6)))
    )
  ;; set new parameter
  (send *ri* :set-gait-generator-param :default-step-time 0.9 :default-double-support-ratio 0.32)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 10 0) (float-vector 0 -10 0)))
  ;;(send *ri* :set-gait-generator-param :default-step-time 1.0 :default-double-support-ratio 0.35)
  ;;(send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 -10 0) (float-vector 0 10 0)))
  (send *ri* :set-impedance-controller-param :arms :k-p 200 :d-p 180 :m-p 5 :m-r 1 :d-r 15 :k-r 20)
  (send *ri* :start-impedance :arms)
  (when start-abc-st
    (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
    (send *ri* :start-st))
  ;; (send *ri* :remove-force-sensor-offset)
  ;; (send *ri* :reset-force-moment-offset-arms)
  )
;;(warn ";; (setup-door-open-real)~%")

(defun set-original-abc-st-param-from-door
  ()
  (when (and (boundp '*original-default-step-time*) (boundp '*original-default-double-support-ratio*))
    (send *ri* :set-gait-generator-param
          :default-step-time *original-default-step-time*
          :default-double-support-ratio *original-default-double-support-ratio*))
  (when (boundp '*original-default-zmp-offsets*)
    (send *ri* :set-auto-balancer-param
          :default-zmp-offsets *original-default-zmp-offsets*))
  )

(defclass param-door
  :super cascaded-link
  :slots (handles)
  )

(defmethod param-door
  (:init
    (door-width door-knob-depth door-knob-width door-knob-height
     door-knob-pos door-handle-pos
     &key (handle-l/r :left) (hinge-l/r (if (eq handle-l/r :left) :right :left)))
    (send-super :init)
    (let* ((door-depth 30) (door-height 1800)
           (b0 (make-cube 1 1 1))
           (b1 (make-cube door-depth door-width door-height))
           (b2 (make-cube door-knob-depth door-knob-width door-knob-height))
           (b-leg1 (make-cube 700 100 100))
           (b-leg2 (make-cube 700 100 100))
           (b-wall1 (make-cube 5 250 door-height))
           (b-wall2 (make-cube 5 250 door-height))
           )
      (send b-leg1 :set-color :gray)
      (send b-leg2 :set-color :gray)
      (send b0 :set-color :gray70)
      (send b1 :set-color :gray70)
      (send b2 :set-color :gray50)
      (send b1 :translate (float-vector 0 (/ door-width (case handle-l/r (:left 2.0) (t -2.0))) (/ door-height 2.0)))
      (send b2 :locate (float-vector (/ door-knob-depth -2.0) (/ door-knob-width (case handle-l/r (:left -2.0) (t 2.0))) 0))
      (send b-leg1 :translate (float-vector -350 (* (case handle-l/r (:right 1.0) (t -1.0)) 50) 50))
      (send b-leg2 :translate (float-vector -350 (* (case handle-l/r (:right 1.0) (t -1.0)) (- -50 door-width)) 50))
      (send b-wall1 :translate (float-vector -2.5 (* (case handle-l/r (:right 1.0) (t -1.0)) 125) (* 0.5 door-height)))
      (send b-wall2 :translate (float-vector -2.5 (* (case handle-l/r (:right 1.0) (t -1.0)) (- -125 door-width)) (* 0.5 door-height)))
      (mapcar #'(lambda (pb cb)
                  (send pb :assoc cb))
              (list b0 b0 b0 b0 b0 b1) (list b-leg1 b-leg2 b-wall1 b-wall2 b1 b2))
      (let* ((l0 (instance bodyset-link :init (make-cascoords) :bodies (list b0 b-leg1 b-leg2 b-wall1 b-wall2) :name :root-link))
             (l1 (instance bodyset-link :init (make-cascoords) :bodies (list b1) :name :door-panel))
             (l2 (instance bodyset-link :init (make-cascoords) :bodies (list b2) :name :door-knob)))
        (send l2 :locate door-knob-pos :world)
        (let ((j0 (instance rotational-joint :init :name :door-hinge-joint
                            :parent-link l0 :child-link l1 :axis (case hinge-l/r (:left :z) (t :-z))))
              (j1 (instance rotational-joint :init :name :door-knob-joint
                            :parent-link l1 :child-link l2 :axis (case handle-l/r (:left :-x) (t :x))))
              (h0 (make-cascoords :pos door-handle-pos :rpy (list 0 (deg2rad 0) pi/2) :name :larm-knob-handle))
              (h1 (make-cascoords :pos door-handle-pos :rpy (list 0 (deg2rad 0) -pi/2) :name :rarm-knob-handle))
              (h2 (make-cascoords :pos door-handle-pos :rpy (list 0 (deg2rad 0) pi/2) :name :knob-handle)))
          (mapcar #'(lambda (pl cl)
                      (send pl :assoc cl))
                  (list self l0 l1 l2 l2 l2) (list l0 l1 l2 h0 h1 h2))
          (setq links (list l0 l1 l2))
          (setq joint-list (list j0 j1))
          (setq handles (list h0 h1 h2))
          (send self :put :handle-l/r handle-l/r)
          (send self :init-ending)
          self))))
  (:handle (name) (find name handles :test #'equal :key #'(lambda (x) (send x :name))))
  )

;; drc box2 door param
;;   door width : 700mm
;;   hinge -> knob-axis : 640mm
;;   knob depth : 60mm
;;   knob width : 100mm
;;   knob height : 20mm
;;   knob pos z : 950mm
(defun make-drc-box2-door ()
  "Door on drc box2."
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :handle-l/r :right)))

;; drc test door param
;;   door width : 840mm
;;   hinge -> knob-axis : 800mm
;;   knob depth : 60mm
;;   knob width : 120mm
;;   knob height : 20mm
;;   knob pos z : 940mm
(defun make-drc-test-door-right ()
  "DRC testing door (right-handled side)"
  (setq *door* (instance param-door :init 840 60 120 20
                         (float-vector 0 -800 940)
                         (float-vector -60 -740 940)
                         :handle-l/r :right)))

(defun make-drc-test-door-left ()
  "DRC testing door (left-handled side)"
  (setq *door* (instance param-door :init 840 60 120 20
                         (float-vector 0 800 940)
                         (float-vector -60 740 940)
                         :handle-l/r :left)))

(defun make-drc-sagami-door ()
  "drc door in sagami full-task-course environment"
  (setq *door* (instance param-door :init 895 60 110 20
                         (float-vector 0 800 845)
                         (float-vector -50 760 845)
                         ;; (float-vector 0 800 1120)
                         ;; (float-vector -50 760 1120)
                         :handle-l/r :left))
  )

(defun make-drc-sagami-door-mirror ()
  "mirror of drc door in sagami full-task-course environment"
  (setq *door* (instance param-door :init 895 60 110 20
                         (float-vector 0 -800 845)
                         (float-vector -50 -760 845)
                         :handle-l/r :right))
  )

(defun make-drc-lasvegas-indoor-door ()
  "drc door in lasvegas indoor environment"
  (setq *door* (instance param-door :init 800 60 110 20
                         (float-vector 0 750 860)
                         (float-vector -50 700 860)
                         :handle-l/r :left))
  )

(defun make-drc-lasvegas-outdoor-door ()
  "drc door in lasvegas outdoor environment"
  (setq *door* (instance param-door :init 795 45 120 30
                         (float-vector 0 735 850)
                         (float-vector -45 675 850)
                         :handle-l/r :left))
  )

(defun make-drc-lasvegas-indoor-door-mirror ()
  "drc door in lasvegas indoor environment"
  (setq *door* (instance param-door :init 800 60 110 20
                         (float-vector 0 -750 860)
                         (float-vector -50 -700 860)
                         :handle-l/r :right))
  )

(defun make-drc-lasvegas-outdoor-door-mirror ()
  "drc door in lasvegas outdoor environment"
  (setq *door* (instance param-door :init 795 45 120 30
                         (float-vector 0 -735 850)
                         (float-vector -45 -675 850)
                         :handle-l/r :right))
  )

(defun exec-robot-state-list
  (rs-list &key (real nil))
  "Execute robot state list.
   rs-list is robot state list, e.g., list of (list :angle-vector av :time tm :root-coords :hand-angle-vector hav ...)"
  (dolist (rs rs-list)
    (send *robot* :angle-vector (cadr (memq :angle-vector rs)))
    (send *robot* :hand-angle-vector (cadr (memq :hand-angle-vector rs)))
    (send *robot* :move-coords (cadr (memq :root-coords rs)) (car (send *robot* :links)))
    (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (if real
        (bench
         (send *ri* :hand-angle-vector (send *robot* :hand-angle-vector) (cadr (memq :time rs)))
         (send *ri* :angle-vector (send *robot* :angle-vector) (cadr (memq :time rs)))
         (send *ri* :wait-interpolation))
      ;;(read-line)
      )
    )
  )

(defun get-current-robot-state
  (tm)
  "tm is angle-vector time [s]."
  (list :angle-vector (send *robot* :angle-vector)
        :hand-angle-vector (send *robot* :hand-angle-vector)
        :root-coords (send (car (send *robot* :links)) :copy-worldcoords)
        :time tm)
  )

(defun fullbody-ik-for-door-open
  (target-coords
   &rest args &key (arm :larm)
   &allow-other-keys)
  (with-move-target-link-list
   (mt ll *robot* (list arm :rleg :lleg))
   (send* *robot* :fullbody-inverse-kinematics
         (list target-coords
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords))
         :additional-nspace-list
         (list (list (send *robot* :torso :waist-p :child-link)
                     #'(lambda () (* 0.5 (elt (v* (send (send *robot* :torso :waist-p :child-link) :rotate-vector #F(0 0 1)) #f(0 0 1)) 1))))
               (list (send *robot* :rarm :wrist-p :child-link)
                     #'(lambda () (* 0.05 (deg2rad (- 0.0 (send *robot* :rarm :wrist-p :joint-angle))))))
               (list (send *robot* :larm :wrist-p :child-link)
                     #'(lambda () (* 0.05 (deg2rad (- 0.0 (send *robot* :larm :wrist-p :joint-angle)))))))
         :move-target (append (list (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm)))) (cdr mt))
         :link-list ll
         :look-at-target t
         :debug-view :no-message
         args))
  )

(defun initialize-pose-for-door-open
  (&key (real t)
        (arm :larm)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0)))
        (initial-waist-y 0))
  (let ((rs-list))
    ;; initialize
    (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
    (send (send *door* :joint :door-knob-joint) :joint-angle 0)
    (send *robot* :reset-manip-pose)
    (send *robot* :torso :waist-y :joint-angle initial-waist-y)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* arm :move-end-rot (case arm (:larm 90) (t -90)) :x)
    (send *robot* arm :move-end-rot (case arm (:larm 30) (t -30)) :z)
    (send *robot* arm :move-end-pos (float-vector 25 0 50) :world)
    (send *robot* :fix-leg-to-coords (send (send *door* :copy-worldcoords) :transform initial-standing-coords :local))
    (case arm
      (:larm (send *robot* :rarm :angle-vector *rarm-avoid-pose*))
      (:rarm (send *robot* :larm :angle-vector *larm-avoid-pose*)))
    (send (send *robot* :hand arm) :angle-vector *door-grasp-preshape-pose*)
    (when (boundp '*irtviewer*) (objects (list *robot* *door*)))
    (exec-robot-state-list (list (get-current-robot-state 5000)) :real real)
    ))

(defun reach-grasp-door
  (&key (real t)
        (arm :larm)
        (way-point-num 2))
  (let ((rs-list))
    ;; reaching poses
    (let* ((knob-target-coords (send *door* :handle (read-from-string (format nil "~A-knob-handle" arm))))
           (cds-list (list (send (send knob-target-coords :copy-worldcoords)
                               :translate (case arm (:larm (float-vector -100 -200 150)) (t (float-vector -100 0 150))) :world)
                           (send (send (send knob-target-coords :copy-worldcoords)
                                       ;;:translate (case arm (:larm (float-vector -50 50 150)) (t (float-vector -50 -50 150))) :world)
                                       :translate (case arm (:larm (float-vector -50 -100 100)) (t (float-vector -50 0 150))) :world)
                                 :rotate (case arm (:larm (deg2rad 10)) (t (deg2rad -10))) :z) ;; way point2
                           knob-target-coords ;; target point
                           )))
      (if (= way-point-num 1) (setq cds-list (cdr cds-list)))
      (dolist (cds cds-list)
        (fullbody-ik-for-door-open cds :arm arm :rotation-axis (list (if (= (position cds cds-list) 0) :z t) t t))
        (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
        (push (get-current-robot-state 2000) rs-list)
        ))
    ;; grasp
    (send (send *robot* :hand arm) :angle-vector *door-grasp-pose*)
    (push (get-current-robot-state 2000) rs-list)
    (reverse rs-list)
    ))

(defun release-grasp-door
  (&key (real t)
        (arm :larm)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0)))
        (release-way-point (case arm
                             (:larm (float-vector -50 -100 100))
                             (t (float-vector -50 100 100))))
        (initialize-p t))
  (let ((rs-list))
    ;; initialize
    (send (send *robot* :hand arm) :angle-vector *door-grasp-preshape-pose*)
    (push (get-current-robot-state 2000) rs-list)
    ;; reaching poses
    (let* ((current-coords (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm))))
           (cds-list (list (send (send (send current-coords :copy-worldcoords)
                                 :translate release-way-point :world)
                                 :rotate (case arm (:larm (deg2rad 10)) (t (deg2rad -10))) :z)))) ;; way point 1
      (dolist (cds cds-list)
        (fullbody-ik-for-door-open cds :arm arm)
        (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
        (push (get-current-robot-state 2000) rs-list)
        ))
    ;; grasp
    (if initialize-p (initialize-pose-for-door-open :real nil :arm arm :initial-standing-coords initial-standing-coords))
    (push (get-current-robot-state 2000) rs-list)
    (reverse rs-list)
    ))

(defun pull-push-door-by-arm
  (&rest args
   &key (arm :larm)
        (push/pull :pull)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0)))
        (angle-list (list 0 -10 -15))
   &allow-other-keys)
  (let ((rs-list))
    (block
     :ik-loop
     (dolist (door-ja angle-list)
       (send (send *door* :joint :door-hinge-joint) :joint-angle (case push/pull (:pull door-ja) (t (- door-ja))))
       ;; rotate knob
       (send (send *door* :joint :door-knob-joint) :joint-angle
             (if (or (= (position door-ja angle-list) 0) (= (position door-ja angle-list) 1)) -55.0 0.0))
       (let ((ik-ret
              (apply #'fullbody-ik-for-door-open
               (send *door* :handle (read-from-string (format nil "~A-knob-handle" arm)))
               :arm arm args)))
         (cond
          ((or (null ik-ret) (send *robot* :self-collision-check))
           (return-from :ik-loop nil))
          (t
           (push (get-current-robot-state 2000) rs-list)))
         )))
    (reverse rs-list)
    ))

(defmacro with-preserve-robot-state
  (robot &rest args)
  "Macro with preserving robot state (angle-vector, worldcoords).
   args is arbitral s-expression.
   Even if robot state is changed in args function, this macro return to the robot state before execution of args."
  (with-gensyms
   (prev-av prev-rc)
   `(let ((,prev-av (send ,robot :angle-vector))
          (,prev-rc (send ,robot :copy-worldcoords)))
      (prog1
          ,@args
        ;; return to original state
        (send ,robot :angle-vector ,prev-av)
        (send ,robot :newcoords ,prev-rc)
        ))))

(defun get-actual-robot-state-with-rpy-estimation
  ()
  "Reflect actual robot state (potentio-vector and estimated rpy).
   Robot foot pos and yaw rotation is fixed."
  (let ((ref-yaw (caar (send (send *robot* :foot-midcoords) :rpy-angle)))
        (ref-fm-pos (send (send *robot* :foot-midcoords) :worldpos)))
    (send *ri* :state)
    (send *robot* :angle-vector (send *ri* :potentio-vector))
    (let* ((act-fm-ypr (car (send (send (send *ri* :robot) :foot-midcoords) :rpy-angle))))
      (send *robot* :fix-leg-to-coords
            (make-coords :pos ref-fm-pos
                         :rpy (list ref-yaw (elt act-fm-ypr 1) (elt act-fm-ypr 2))))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )))

(defun estimate-hand-diff-pos
  (&key (arm :larm) (func))
  "Estimate hand difference betweeen before execution of func and after execution of func."
  (let ((pos1) (pos2))
    (with-preserve-robot-state
     *robot*
     (get-actual-robot-state-with-rpy-estimation)
     (setq pos1 (copy-object (send (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm))) :worldpos))))
    (with-preserve-robot-state
     *robot*
     (funcall func)
     (get-actual-robot-state-with-rpy-estimation)
     (setq pos2 (copy-object (send (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm))) :worldpos))))
    (v- pos2 pos1)))

(defun door-try-push-pull
  (&key (arm :larm) (push/pull :push) (push/pull-stride 40))
  (estimate-hand-diff-pos
   :arm arm
   :func
   #'(lambda ()
       ;; try push or pull
       (fullbody-ik-for-door-open
        (send (send (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm))) :copy-worldcoords)
              :translate (float-vector (* (case push/pull (:push 1) (t -1)) push/pull-stride) 0 0))
        :arm arm)
       (exec-robot-state-list
        (list (get-current-robot-state 1500)) :real t)
       (unix:sleep 1))))

(defun door-push-pull-detection
  (&key (arm :larm) (knob-ja -55))
  "Detect push or pull.
   Return value is :push, :pull, or nil.
   nil is too small hand diff for both push and pull."
  (let ((push-diff) (pull-diff))
    (send (send *door* :joint :door-knob-joint) :joint-angle knob-ja)
    (fullbody-ik-for-door-open
     (send *door* :handle (read-from-string (format nil "~A-knob-handle" arm)))
     :arm arm)
    (exec-robot-state-list
     (list (get-current-robot-state 1000)) :real t)
    (with-preserve-robot-state
     *robot*
     (setq push-diff (door-try-push-pull :arm arm :push/pull :push)))
    (with-preserve-robot-state
     *robot*
     (setq pull-diff (door-try-push-pull :arm arm :push/pull :pull)))
    (let ((ret
           (cond
            ((and (< (norm pull-diff) 10.0) (< (norm push-diff) 10.0)) ;; [mm]
             nil)
            ((< (norm pull-diff) (norm push-diff))
             :push)
            (t :pull))))
      (format t ";; push-diff-norm ~A, pull-diff-norm ~A => ~A~%"
              (norm push-diff) (norm pull-diff)
              (if ret ret "too small diff!! Failed to open?"))
      ;; return to initial pose
      (exec-robot-state-list
       (list (get-current-robot-state 1000)) :real t)
      ret)))

(defun test-door-open-drcbox2-door (&key (walk-p t) (wait t) (draw t) (real t))
  "Test for drc box2 door"
  (print ";; Initial pose")
  (make-drc-box2-door)
  (when draw (objects (list *robot* *door*)))
  ;; (send *door* :move-coords
  ;;       (send (send (send (send (car (send *robot* :links)) :copy-worldcoords) :transform (make-coords :pos (float-vector 642.373 43.578 296.552) :rpy (list -1.343 -0.009 -1.602))) :rotate (deg2rad 90) :y) :rotate (deg2rad 180) :z)
  ;;       (send *door* :handle :larm-knob-handle))
  ;; (send *door* :translate #f(20 -50 0) :world)
  (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (setq *init-standing-coords* (make-coords :pos (float-vector -1000 -450 0) :rpy (list (deg2rad -80) 0 0)))
         )
        (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (setq *init-standing-coords* (make-coords :pos (float-vector -650 -450 0) :rpy (list (deg2rad -15) 0 0)))
         ))
  (initialize-pose-for-door-open :real nil :arm :larm :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real real)
  (print ";; Reaching and open. Press Enter")
  (when wait (read-line))
  (setq *rs-list* (reach-grasp-door :real nil))
  (exec-robot-state-list *rs-list* :real real)
  (let ((push/pull :pull))
  ;;(let ((push/pull (door-push-pull-detection :arm :larm)))
    (when push/pull
      (setq *rs-list* (pull-push-door-by-arm :push/pull push/pull))
      (exec-robot-state-list *rs-list* :real real)
      (when walk-p
        (print ";; go-velocity Press Enter")
        (when wait (read-line))
        (let ((rc (send (send *robot* :foot-midcoords) :transformation (send *robot* :larm :end-coords))))
          (send *ri* :set-auto-balancer-param :graspless-manip-arm "larm"
                :graspless-manip-mode t
                ;;:graspless-manip-p-gain #f(0.9 0.9 0.9)
                :graspless-manip-p-gain #f(0.7 0.7 0.9)
                :graspless-manip-reference-trans-pos (send rc :worldpos)
                :graspless-manip-reference-trans-rot (send rc :worldrot)))
        (send* *ri* :go-velocity (send *ri* :calc-go-velocity-param-from-velocity-center-offset -6.0 (send (send (send *robot* :foot-midcoords) :transformation (send (send *door* :joint :door-hinge-joint) :child-link)) :worldpos)))
        ;;(send* *ri* :go-velocity (send *ri* :calc-go-velocity-param-from-velocity-center-offset -4.5 (send (send (send *robot* :foot-midcoords) :transformation (send (send *door* :joint :door-hinge-joint) :child-link)) :worldpos)))
        ;;(unix:sleep 3)
        ;;(unix:sleep 5)
        ;;(unix:sleep 18)
        (unix:sleep 10)
        ;;(unix:sleep 5)
        (send *ri* :go-stop)
        )
      ))
    (print ";; release")
    (when wait (read-line))
    (setq *rs-list* (release-grasp-door :real nil :arm :larm :initial-standing-coords *init-standing-coords*))
    (exec-robot-state-list *rs-list* :real real)
    )

(defun test-door-open-drc-test-door-right (&key (walk-p t))
  "Test for drc-test door right"
  (make-drc-test-door-right)
  (objects (list *robot* *door*))
  (setq *init-standing-coords* (make-coords :pos (float-vector -600 -400 0) :rpy (list (deg2rad 20) 0 0)))
  (initialize-pose-for-door-open :real nil :arm :rarm :initial-standing-coords *init-standing-coords* :initial-waist-y -20)
  (send *robot* :head :look-at (send (send (send *door* :joint :door-knob-joint) :child-link) :worldpos))
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
  (print ";; reach and open")
  (read-line)
  (setq *rs-list* (reach-grasp-door :real nil :arm :rarm))
  (exec-robot-state-list *rs-list* :real t)
  (let ((push/pull (door-push-pull-detection :arm :rarm)))
    (when push/pull
      ;;(read-line)
      (setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords *init-standing-coords* :push/pull push/pull :angle-list (list 0 -5 -10)))
      (exec-robot-state-list *rs-list* :real t))
    ;;(read-line)
    (setq *rs-list* (release-grasp-door :real nil :arm :rarm :initial-standing-coords *init-standing-coords*))
    (exec-robot-state-list *rs-list* :real t)
    (send (send *robot* :hand :rarm) :angle-vector *door-grasp-pose*)
    (exec-robot-state-list (list (get-current-robot-state 1000)) :real t))
  (when walk-p
    (print ";; go-pos fwd")
    (read-line)
    (dotimes (i 2)
      (let* ((fwd-param (float-vector 50 0 0))
             (go-pos-param (send (send *robot* :foot-midcoords) :inverse-rotate-vector fwd-param)))
        (send *ri* :go-pos (* 1e-3 (elt go-pos-param 0)) (* 1e-3 (elt go-pos-param 1)) 0)
        (send *robot* :translate fwd-param)))
    (print ";; move door by rarm")
    (read-line)
    (initialize-pose-for-door-open :real nil :arm :rarm :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
    (send *robot* :rarm :angle-vector *rarm-reset-manip-pose*)
    (send *robot* :rarm :move-end-pos #f(300 100 0) :world :rotation-axis nil)
    (send (send *robot* :hand :rarm) :angle-vector *door-grasp-pose*)
    (exec-robot-state-list (list (get-current-robot-state 2000)) :real t)
    (initialize-pose-for-door-open :real nil :arm :rarm :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
    (send *robot* :rarm :angle-vector *rarm-reset-manip-pose*)
    (send (send *robot* :hand :rarm) :angle-vector *door-grasp-pose*)
    (exec-robot-state-list (list (get-current-robot-state 1000)) :real t)
    (print ";; go-pos rotate")
    (read-line)
    ;;(send *ri* :go-pos 0 0 70)
    (dotimes (i 3)
      (send *ri* :go-pos 0 0 20))
    ;; (send *ri* :go-pos 0 0 20)
    ;; (send *ri* :go-pos 0 -0.5 0)
    ;; (send *ri* :go-pos 0 -0.5 0)
    )
  )

(defun test-door-open-drc-test-door-left (&key (walk-p t))
  "Test for drc-test door left"
  (print ";; Initial pose")
  (make-drc-test-door-left)
  (objects (list *robot* *door*))
  (setq *init-standing-coords* (make-coords :pos (float-vector -750 650 0) :rpy (list (deg2rad -10) 0 0)))
  (initialize-pose-for-door-open :real nil :arm :rarm :initial-standing-coords *init-standing-coords* :initial-waist-y 20)
  (send *robot* :head :look-at (send (send (send *door* :joint :door-knob-joint) :child-link) :worldpos))
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
  (print ";; Reach and open. Press enter")
  (read-line)
  (setq *rs-list* (reach-grasp-door :real nil :arm :rarm))
  (exec-robot-state-list *rs-list* :real t)
  (let ((push/pull (door-push-pull-detection :arm :rarm)))
    (when push/pull
      (setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords *init-standing-coords*
                                             :push/pull push/pull :angle-list (list 0 -5 -10 -25 -30)))
      (exec-robot-state-list *rs-list* :real t))
    (print ";; Release door.")
    (setq *rs-list* (release-grasp-door :real nil :arm :rarm :initial-standing-coords *init-standing-coords*
                                        :release-way-point (float-vector -55 -100 100))))
  (send (send *door* :joint :door-hinge-joint) :joint-angle 30)
  (exec-robot-state-list *rs-list* :real t)
  (send *robot* :hand :arms :angle-vector *door-grasp-pose*)
  (send *robot* :larm :angle-vector *larm-reset-manip-pose*)
  (send *robot* :rarm :angle-vector *rarm-avoid-pose*)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (exec-robot-state-list (list (get-current-robot-state 2000)) :real t)
  (when walk-p
    (print ";; go-pos bwd. Press Enter.")
    (read-line)
    (let* ((bwd-param (float-vector -250 0 0))
           (go-pos-param (send (send *robot* :foot-midcoords) :inverse-rotate-vector bwd-param)))
      (send *ri* :go-pos (* 1e-3 (elt go-pos-param 0)) (* 1e-3 (elt go-pos-param 1)) 0))
    (print ";; Reach larm. Press Enter.")
    (read-line)
    (send *robot* :translate #f(-250 0 0) :world)
    (fullbody-ik-for-door-open
     (send (send (send *door* :handle :larm-knob-handle) :copy-worldcoords) :translate (float-vector 150 0 -150))
     :translation-axis (list t t t)
     :arm :larm
     :rotation-axis (list nil t t))
    (exec-robot-state-list (list (get-current-robot-state 1000)) :real t)
    (fullbody-ik-for-door-open
     (send (send (send *door* :handle :larm-knob-handle) :copy-worldcoords) :translate (float-vector 150 0 0))
     :translation-axis (list t t t)
     :arm :larm
     :rotation-axis (list nil t t))
    (exec-robot-state-list (list (get-current-robot-state 1000)) :real t)
    (setq *rs-list* (pull-push-door-by-arm :arm :larm :initial-standing-coords *init-standing-coords* :push/pull :push :angle-list (list -35 -45 -55) :rotation-axis (list nil t t)))
    (exec-robot-state-list *rs-list* :real t)
    (print ";; gopos -30 and reset-manip-pose")
    (read-line)
    (send *ri* :go-pos 0 0 -30)
    (send *robot* :reset-manip-pose)
    (send *robot* :arms :move-end-pos #f(70 0 0))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
    (dotimes (i 2)
      (send *ri* :go-pos 0 0 -30))
    (read-line)
    (send *ri* :go-pos 0.15 0 0)
    )
  )

(defun test-door-open-drc-sagami-door (&key (walk-p t) (wait t) (draw t) (real t))
  "drc door in sagami full-task-course environment"
  (make-drc-sagami-door)
  (when draw (objects (list *robot* *door*)))
  (setq *init-standing-coords* (make-coords :pos (float-vector -600 400 0) :rpy (list (deg2rad -20) 0 0)))
  ;;(setq *init-standing-coords* (make-coords :pos (float-vector -750 650 0) :rpy (list (deg2rad -10) 0 0)))
  ;; initial pose
  (initialize-pose-for-door-open :real nil :arm :larm :initial-standing-coords *init-standing-coords* :initial-waist-y -20)
  (send *robot* :head :look-at (send (send (send *door* :joint :door-knob-joint) :child-link) :worldpos))
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real real)
  (print ";; reach and open")
  (when wait (read-line))
  ;; reach
  (setq *rs-list* (reach-grasp-door :real nil :arm :larm))
  (exec-robot-state-list *rs-list* :real real)
  ;; push
  (let ((push/pull :push))
    (when push/pull
      (setq *rs-list* (pull-push-door-by-arm :arm :larm :initial-standing-coords *init-standing-coords* :push/pull push/pull :angle-list (list 0 -5 -10 -15)))
      (exec-robot-state-list *rs-list* :real real))
    (setq *rs-list* (release-grasp-door :real nil :arm :larm :initial-standing-coords *init-standing-coords*))
    (exec-robot-state-list *rs-list* :real real)
    (send (send *robot* :hand :larm) :angle-vector *door-grasp-pose*)
    (exec-robot-state-list (list (get-current-robot-state 1000)) :real real))
  ;; walk
  (when walk-p
    (print ";; walk")
    (when wait (read-line))
    (let* ((walk-pos (float-vector 100 0 0)))
      (setq walk-pos (send (send *door* :copy-worldcoords) :rotate-vector walk-pos))
      (when real (send *ri* :go-pos (* 1e-3 (elt walk-pos 0)) (* 1e-3 (elt walk-pos 1)) 0))
      (send *robot* :translate walk-pos :local)))
  ;; punch
  (print ";; punch")
  (when wait (read-line))
  (initialize-pose-for-door-open :real nil :arm :larm
                                 :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
  (send *robot* :larm :angle-vector *larm-reset-manip-pose*)
  (send *robot* :larm :move-end-pos #f(200 -100 0) :local :rotation-axis nil)
  (send (send *robot* :hand :larm) :angle-vector *door-grasp-pose*)
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real real)
  (initialize-pose-for-door-open :real nil :arm :larm
                                 :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
  (send *robot* :larm :angle-vector *larm-reset-manip-pose*)
  (send *robot* :larm :move-end-pos #f(350 -100 0) :local :rotation-axis nil)
  (send (send *robot* :hand :larm) :angle-vector *door-grasp-pose*)
  (exec-robot-state-list (list (get-current-robot-state 1000)) :real real)
  (initialize-pose-for-door-open :real nil :arm :larm
                                 :initial-standing-coords *init-standing-coords* :initial-waist-y 0)
  (send *robot* :larm :angle-vector *larm-reset-manip-pose*)
  (send (send *robot* :hand :larm) :angle-vector *door-grasp-pose*)
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real real)
  (send *robot* :larm :angle-vector *larm-avoid-pose*)
  (send *robot* :rarm :angle-vector *rarm-avoid-pose*)
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real real)
  )

#|
(defun door-initial-open
  (&key (arm :larm))
  (dolist (knob-ja (list -20 -40 -60))
    (door-push-pull-detection :arm arm :knob-ja knob-ja)
    (read-line)))


(defun test-check-walk ()
  (send *ri* :go-velocity 0 0 0)
  (send *robot* :torso :waist-y :joint-angle 40)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *robot* :torso :waist-y :joint-angle 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :go-stop)
  )


(defun test-door-open-2 ()
  (setq *init-standing-coords* (make-coords :pos (float-vector -600 -400 0) :rpy (list (deg2rad 20) 0 0)))
  (initialize-pose-for-door-open :real nil :arm :rarm :initial-standing-coords *init-standing-coords* :initial-waist-y -20)
  (send *robot* :head :look-at (send (send (send *door* :joint :door-knob-joint) :child-link) :worldpos))
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
  (setq *rs-list* (reach-grasp-door :real nil :arm :rarm))
  (exec-robot-state-list *rs-list* :real t)
  (read-line)
  ;;(setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0)) :push/pull :push))
  (setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0)) :push/pull :push :angle-list (list 0 -5 -10)))
  (exec-robot-state-list *rs-list* :real t)
  (read-line)
  (let ((rc (send (send *robot* :foot-midcoords) :transformation (send *robot* :rarm :end-coords))))
    (send *ri* :set-auto-balancer-param :graspless-manip-arm "rarm"
          :graspless-manip-mode t
          :graspless-manip-p-gain #f(0.7 0.7 0.7)
          :graspless-manip-reference-trans-pos (send rc :worldpos)
          :graspless-manip-reference-trans-rot (send rc :worldrot)))
  (send* *ri* :go-velocity (send *ri* :calc-go-velocity-param-from-velocity-center-offset 6.0 (send (send (send *robot* :foot-midcoords) :transformation (send (send *door* :joint :door-hinge-joint) :child-link)) :worldpos)))
  (unix:sleep 1)
  ;;(unix:sleep 3)
  ;;(unix:sleep 5)
  ;;(unix:sleep 18)
  (send *ri* :go-stop)
  (read-line)
  (exec-robot-state-list (reverse *rs-list*) :real t)
  (read-line)
  (release-grasp-door :real t :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0) :rpy (list (deg2rad 15) 0 0)))
  )
;; (dolist (rs ret) (send *robot* :angle-vector (cadr (memq :angle-vector rs))) (send *robot* :move-coords (cadr (memq :root-coords rs)) (car (send *robot* :links))) (send *irtviewer* :draw-objects) (read-line))

(defun test-door-open-3 ()
  (make-drc-test-door-right)
  (objects (list *robot* *door*))
  (setq *init-standing-coords* (make-coords :pos (float-vector -600 -450 0) :rpy (list (deg2rad -40) 0 0)))
  (initialize-pose-for-door-open :real nil :arm :larm :initial-standing-coords *init-standing-coords* :initial-waist-y 30)
  ;;(exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
  (send (send *robot* :hand :larm) :angle-vector #f(0.0 70.0 0.0 -10.0 30.0 30.0))
  (setq *rs-list* (reach-grasp-door :real nil :arm :larm))
  (exec-robot-state-list (list (elt *rs-list* 1)) :real nil)
  (send *robot* :larm :move-end-pos #f(0 200 0) :world :rotation-axis :z)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (exec-robot-state-list (list (get-current-robot-state 3000)) :real t)
  (read-line)
  (let ((go-pos-param (send (send *robot* :foot-midcoords) :inverse-rotate-vector (float-vector 50 0 0))))
    (send *ri* :go-pos (* 1e-3 (elt go-pos-param 0)) (* 1e-3 (elt go-pos-param 1)) 0))
  (read-line)
  (exec-robot-state-list (cdr *rs-list*) :real t)
  (read-line)
  (setq *rs-list* (pull-push-door-by-arm :arm :larm :initial-standing-coords *init-standing-coords*
                                         :push/pull :push :angle-list (list 0 -5 -10)))
  (exec-robot-state-list *rs-list* :real t)
  (read-line)
  (setq *rs-list* (release-grasp-door :real nil :arm :larm :initial-standing-coords *init-standing-coords*))
  )
|#

(defun test-lateral-walk-1m-left ()
  (progn
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :legs :move-end-pos (float-vector 0 0 50))
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation))
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 20 0) (float-vector 0 -20 0)))
  (let ((base 250))
    (send *ri* :set-foot-steps
          (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 base 0)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 base 0)) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 2) 0)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 2) 0)) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 3) 0)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 3) 0)) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 4) 0)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 (* base 4) 0)) :name :rleg)
                )))
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 0 0) (float-vector 0 0 0)))
  )
