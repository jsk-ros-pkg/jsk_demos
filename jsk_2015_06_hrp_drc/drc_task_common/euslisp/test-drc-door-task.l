(load "~/prog/euslib/irteus_proposals/motion-lib-proposal.l")
(defun setup-door-open ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (make-drc-box2-door)
  (objects (list *robot*))
  ;; initialize end-coords for door-grasp
  (mapcar #'(lambda (arm trs)
              (let ((mc (make-cascoords
                         :coords (send (send *robot* arm :end-coords :copy-worldcoords)
                                       :translate trs)
                         :name (read-from-string (format nil "~A-tip-grasp-coords" arm)))))
                (send *robot* :put (read-from-string (format nil "~A-tip-grasp-coords" arm)) mc)
                (send (send (send *robot* arm :end-coords) :parent) :assoc mc)
                ))
          '(:rarm :larm) (list #f(50 -25 0) #f(50 25 0)))
  ;; ABC, ST, IMP param
  ;;(send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 -10 0) (float-vector 0 10 0)))
  ;;(send *ri* :set-gait-generator-param :default-step-time 1.0 :default-double-support-ratio 0.35)
  (send *ri* :set-gait-generator-param :default-step-time 0.9 :default-double-support-ratio 0.32)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 10 0) (float-vector 0 -10 0)))
  (send *ri* :set-impedance-controller-param :arms :k-p 200 :d-p 120 :m-p 5 :m-r 1 :d-r 15 :k-r 20)
  (send *ri* :start-impedance :arms)
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
  (send *ri* :start-st)
  ;; (send *ri* :remove-force-sensor-offset)
  ;; (send *ri* :reset-force-moment-offset-arms)
  )

(defclass param-door
  :super cascaded-link
  :slots (handles)
  )

(defmethod param-door
  (:init
    (door-width door-knob-depth door-knob-width door-knob-height
     door-knob-pos door-handle-pos
     &key (handle-l/r :left))
    (send-super :init)
    (let* ((door-depth 30) (door-height 1800)
           (b0 (make-cube 1 1 1))
           (b1 (make-cube door-depth door-width door-height))
           (b2 (make-cube door-knob-depth door-knob-width door-knob-height))
           (b-leg1 (make-cube 700 100 100))
           (b-leg2 (make-cube 700 100 100)))
      (send b1 :translate (float-vector 0 (/ door-width (case handle-l/r (:left 2.0) (t -2.0))) (/ door-height 2.0)))
      (send b2 :locate (float-vector (/ door-knob-depth -2.0) (/ door-knob-width (case handle-l/r (:left -2.0) (t 2.0))) 0))
      (send b-leg1 :translate (float-vector -350 50 50))
      (send b-leg2 :translate (float-vector -350 (+ -50 (* (case handle-l/r (:left 1.0) (t -1.0)) door-width)) 50))
      (mapcar #'(lambda (pb cb)
                  (send pb :assoc cb))
              (list b0 b1) (list b1 b2))
      (let* ((l0 (instance bodyset-link :init (make-cascoords) :bodies (list b0 b-leg1 b-leg2) :name :root-link))
             (l1 (instance bodyset-link :init (make-cascoords) :bodies (list b1) :name :door-panel))
             (l2 (instance bodyset-link :init (make-cascoords) :bodies (list b2) :name :door-knob)))
        (send l2 :locate door-knob-pos :world)
        (let ((j0 (instance rotational-joint :init :name :door-hinge-joint
                            :parent-link l0 :child-link l1 :axis :z))
              (j1 (instance rotational-joint :init :name :door-knob-joint
                            :parent-link l1 :child-link l2 :axis (case handle-l/r (:left :-x) (t :x))))
              (h0 (make-cascoords :pos door-handle-pos :rpy (list 0 (deg2rad 0) pi/2) :name :larm-knob-handle))
              (h1 (make-cascoords :pos door-handle-pos :rpy (list 0 (deg2rad 0) -pi/2) :name :rarm-knob-handle)))
          (mapcar #'(lambda (pl cl)
                      (send pl :assoc cl))
                  (list self l0 l1 l2 l2) (list l0 l1 l2 h0 h1))
          (setq links (list l0 l1 l2))
          (setq joint-list (list j0 j1))
          (setq handles (list h0 h1))
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
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :handle-l/r :right)))

(defun make-drc-test-door ()
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 640 950)
                         (float-vector -60 600 950)
                         :handle-l/r :left)))

(defun exec-robot-state-list
  (rs-list &key (real nil))
  "Execute robot state list.
   rs-list is robot state list, e.g., list of (list :angle-vector av :time tm :root-coords :hand-angle-vector hav ...)"
  (dolist (rs rs-list)
    (send *robot* :angle-vector (cadr (memq :angle-vector rs)))
    (send *robot* :hand-angle-vector (cadr (memq :hand-angle-vector rs)))
    (send *robot* :move-coords (cadr (memq :root-coords rs)) (car (send *robot* :links)))
    (send *irtviewer* :draw-objects)
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
  (target-coords &key (arm :larm))
  (with-move-target-link-list
   (mt ll *robot* (list arm :rleg :lleg))
   (send *robot* :fullbody-inverse-kinematics
         (list target-coords
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords))
         :additional-nspace-list
         (list (list (send *robot* :torso :waist-p :child-link)
                             #'(lambda () (* 0.5 (elt (v* (send (send *robot* :torso :waist-p :child-link) :rotate-vector #F(0 0 1)) #f(0 0 1)) 1)))))
         ;; :additional-weight-list
         ;; (append (list (list (send *robot* :torso :waist-p :child-link) 0.5))
         ;;         (mapcar #'(lambda (l) (list (send *robot* l :toe-p :child-link) 0)) '(:rleg :lleg)))
         :move-target (append (list (send *robot* :get (read-from-string (format nil "~A-tip-grasp-coords" arm)))) (cdr mt))
         :link-list ll
         :look-at-target t
         :debug-view :no-message))
  )

(defun initialize-pose-for-door-open
  (&key (real t)
        (arm :larm)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0))))
  (let ((rs-list))
    ;; initialize
    (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
    (send (send *door* :joint :door-knob-joint) :joint-angle 0)
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords initial-standing-coords)
    (send *robot* arm :move-end-rot (case arm (:larm 90) (t -90)) :x)
    (send *robot* arm :move-end-pos (float-vector 100 0 50) :world)
    (send *robot* arm :move-end-rot (case arm (:larm 10) (t -10)) :z)
    (send (send *robot* :hand arm) :angle-vector #f(0.0 70.0 0.0 -10.0 30.0 30.0))
    (objects (list *robot* *door*))
    (exec-robot-state-list (list (get-current-robot-state 5000)) :real real)
    ))

(defun reach-grasp-door
  (&key (real t)
        (arm :larm)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0))))
  (let ((rs-list))
    ;; initialize
    (initialize-pose-for-door-open :real nil :arm arm :initial-standing-coords initial-standing-coords)
    (push (get-current-robot-state 3000) rs-list)
    ;; reaching poses
    (let ((knob-target-coords (send *door* :handle (read-from-string (format nil "~A-knob-handle" arm)))))
    (dolist (cds (list (send (send (send knob-target-coords :copy-worldcoords)
                                   ;;:translate (case arm (:larm (float-vector -50 50 150)) (t (float-vector -50 -50 150))) :world)
                                   :translate (case arm (:larm (float-vector -50 50 150)) (t (float-vector -50 0 150))) :world)
                             :rotate (case arm (:larm (deg2rad 10)) (t (deg2rad -10))) :z) ;; way-point
                       knob-target-coords ;; target point
                       ))
      (fullbody-ik-for-door-open cds :arm arm)
      (send *irtviewer* :draw-objects)
      (push (get-current-robot-state 2000) rs-list)
      ))
    ;; grasp
    (send (send *robot* :hand arm) :angle-vector #f(0.0 -20.0 10.0 -10.0 120.0 120.0))
    (push (get-current-robot-state 2000) rs-list)
    (exec-robot-state-list (reverse rs-list) :real real)
    rs-list
    ))

(defun release-grasp-door
  (&key (real t)
        (arm :larm)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0))))
  (exec-robot-state-list (cdr (reach-grasp-door :real nil :arm arm :initial-standing-coords initial-standing-coords))
                         :real real)
  )

(defun pull-push-door-by-arm
  (&key (arm :larm)
        (push/pull :pull)
        (initial-standing-coords
         (make-coords :pos (float-vector -650 -450 0)
                      :rpy (list (deg2rad -15) 0 0)))
        (angle-list (list 0 -10 -15)))
  ;;(initialize-pose-for-door-open :real nil :arm arm :initial-standing-coords initial-standing-coords)
  (let ((rs-list))
    (block
     :ik-loop
     ;;(dolist (door-ja (list 0 -15 -30 -45 -60))
     ;;     (dolist (door-ja (list 0 -5 -10 -15 -20))
     (dolist (door-ja angle-list)
       (send (send *door* :joint :door-hinge-joint) :joint-angle (case push/pull (:pull door-ja) (t (- door-ja))))
       ;; rotate knob
       (send (send *door* :joint :door-knob-joint) :joint-angle
             (if (or (eps= (float door-ja) 0.0) (eps= (abs (float door-ja)) 10.0)) -40.0 0.0))
       (let ((ik-ret
              (fullbody-ik-for-door-open
               (send *door* :handle (read-from-string (format nil "~A-knob-handle" arm)))
               :arm arm)))
         (cond
          ((or (null ik-ret) (send *robot* :self-collision-check))
           (return-from :ik-loop nil))
          (t
           (push (get-current-robot-state 2000) rs-list)))
         )))
    (reverse rs-list)
    ))

(defun test-door-open-1 ()
  (reach-grasp-door :real t)
  (setq *rs-list* (pull-push-door-by-arm))
  (exec-robot-state-list *rs-list* :real t)
  (read-line)
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
  (send *ri* :go-stop)
  (read-line)
  (exec-robot-state-list (reverse *rs-list*) :real t)
  (release-grasp-door :real t)
  )

(defun test-door-open-2 ()
  ;;(reach-grasp-door :real t :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0)))
  (reach-grasp-door :real t :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0) :rpy (list (deg2rad 15) 0 0)))
  (read-line)
  ;;(setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0)) :push/pull :push))
  (setq *rs-list* (pull-push-door-by-arm :arm :rarm :initial-standing-coords (make-coords :pos (float-vector -550 -250 0)) :push/pull :push :angle-list (list 0 -10 -20)))
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
