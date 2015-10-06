#!/usr/bin/env roseus

(ros::roseus "jvrc_grasp_spreader")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")


(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://jvrc_models/models/spreader.l")
(require "package://drc_task_common/euslisp/primitive-marker-util.l")


(when (not (boundp '*jaxon_red*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :start-auto-balancer :limbs (list :rleg :lleg :rarm))
  (send *ri* :start-st)
  (send *ri* :set-impedance-controller-param :rarm :m-p 2000)
  (send *ri* :set-gait-generator-param :default-step-time 0.8) ;; walking speed
  (send *ri* :set-gait-generator-param :stride-parameter (float-vector 0.2 0.1 15.0 0.1))
  )

(when (not (boundp '*spreader*))
  (setq *spreader* (spreader))
  (setq *spreader-grip* (make-cascoords :pos #f(0 170 140) :rpy (list 0 (deg2rad 90) (deg2rad 90))))
  (setq *spreader-body* (make-cascoords :pos #f(0 15 60) :rpy (list 0 (deg2rad 90) (deg2rad 90))))
  (setq *spreader-head* (make-cascoords :pos #f(0 -335 35) :rpy (list 0 0 0))))
  (send *spreader* :assoc *spreader-grip*)
  (send *spreader* :assoc *spreader-body*)
  (send *spreader* :assoc *spreader-head*)
  )
(when (not (boundp '*tf*))
  (setq *tf* (instance ros::transform-listener :init))
  )

(objects (list *robot* *spreader* *spreader-grip* *spreader-body*))
(format t ";; (select-func)~%")

;; insert spreader marker
(defun insert-spreader (&key (place "grasp"))
  (insert-primitive-marker :shape-type 3 :name "spreader" :description "" :resource "package://jvrc_models/models/spreader-visual.dae")
  (ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
  (set-primitive-marker-color :r 0 :b 1 :g 1 :a 1)
  (if (equal place "grasp")
      (set-primitive-marker-pose (make-cascoords :pos #f(600 -100 -125) :rpy (list (deg2rad 180) (deg2rad 0) 0)) "BODY" :name "spreader")
    (set-primitive-marker-pose (make-cascoords :pos #f(200 -400 600) :rpy (list (deg2rad 90) (deg2rad 0) 0)) "RLEG_LINK5" :name "spreader")
    )
  )

;; get robot position in eus
(defun set-robot-eus (&key (set-current-pose t))
  (send *robot* :locate #f(0 0 0) :world)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *lfoot* (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq *rfoot* (send *robot* :rleg :end-coords :copy-worldcoords))
  (when set-current-pose
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    )
  (send *robot* :fix-leg-to-coords (make-coords))
  )

;; get spreader marker pose
(defun get-spreader-pose (&key (frame-id "BODY"))
  (let* (
         (spreader-pose (get-primitive-marker-pose :name "spreader" :frame-id frame-id))
         ;; (spreader-pose (make-cascoords :pos #f(600 -200 -300) :rpy (list (deg2rad 90) (deg2rad 0) 0)));; (list (deg2rad 180) (deg2rad 90) 0)))
         )
    (set-robot-eus :set-current-pose t)
    (setq spreader-pose (send spreader-pose :copy-worldcoords))

    (send spreader-pose :transform (send *robot* :body_lk :worldcoords) :world)
    (send *spreader* :move-to (send spreader-pose :copy-worldcoords) :world)
    (send *irtviewer* :draw-objects)
    )
  )

;; reach spreader
(defun reach-spreader (&key (ang 0) (real nil) (base-time 4000) (grasp-point *spreader-body*) (limb :rarm))
  (if real
      (progn
        (send *ri* :stop-impedance limb)
        (send *ri* :start-grasp)
        (send *ri* :wait-interpolation)
        )
    )
  (get-spreader-pose)
  (if (equal limb :rarm)
      (progn
        (send *robot* :rarm_joint0 :joint-angle -8)
        (send *robot* :rarm_joint2 :joint-angle -100)
        )
    (progn
      (send *robot* :larm_joint0 :joint-angle 8)
      (send *robot* :larm_joint2 :joint-angle 100)
      )
    )
  (if real
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) (* base-time 0.5))
        (send *ri* :wait-interpolation)
        )
    )
  (let* ((reach-z (send grasp-point :rotate-vector (float-vector 0 0 1)))
         (reach-y (v* reach-z (float-vector 0 0 -1)))
         (reach-x (v* reach-y reach-z))
         (reach-r (transpose (matrix reach-x reach-y reach-z)))
         (spreader-reach (make-cascoords :pos (send grasp-point :worldpos) :rot reach-r))
         (link-list (list limb :lleg :rleg))
         )
    ;; (setq *lfoot* (send *robot* :lleg :end-coords :copy-worldcoords))
    ;; (setq *rfoot* (send *robot* :rleg :end-coords :copy-worldcoords))
    ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))

    (send spreader-reach :translate #f(0 0 300) :world)
    (send spreader-reach :rotate (deg2rad ang) :z)

    ;; fullbody-ik (arm will be above spreader, and feet on ground)
    (with-move-target-link-list
     (mt ll *robot* link-list)
     (send *robot*
           :fullbody-inverse-kinematics (list spreader-reach *lfoot* *rfoot*)
           :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 30
           :move-target mt
           :link-list ll
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           ;;:min-loop nil
           :cog-null-space nil
           :use-6dof-joint-leg-gain nil
           :revert-if-fail nil
           :debug-view :no-message) ;;t)
     )
    (if real
        (progn
          (send *ri* :stop-grasp)
          (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
          (send *ri* :wait-interpolation)
          )
      )
    (send spreader-reach :translate #f(0 0 -120) :world)
    ;; fullbody-ik (arm will be just above spreader, and feet on ground)
    (with-move-target-link-list
     (mt ll *robot* link-list)
     (send *robot*
           :fullbody-inverse-kinematics (list spreader-reach *lfoot* *rfoot*)
           :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 30
           :move-target mt
           :link-list ll
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           ;;:min-loop nil
           :cog-null-space nil
           :use-6dof-joint-leg-gain nil
           :debug-view :no-message) ;;t)
     )
    (setq *grasp-point* grasp-point)
    (setq *grasp-hand* limb)
    )
  (if real
      (progn
        (send *ri* :stop-grasp)
        (send *ri* :angle-vector (send *robot* :angle-vector) (* base-time 0.5))
        (send *ri* :wait-interpolation)
        (send *ri* :start-impedance *grasp-hand*)
        )
      )
  )


;; adjust position and the degree of hand opening
(defun adjust-spreader (&key (real nil) (base-time 500) (limb :rarm))
  (when (not (boundp '*grasp-hand*))
    (setq *grasp-hand* limb)
    )
  (let* ((hand-opening) (direction) (length) (move-fv #f(0 0 0)))
    ;; (while (y-or-n-p "adjust the degree of hand-opening? press y to adjust or n to finish")
    ;;   (format t "How much to open? [0, 1]~%")
    ;;   (setq hand-opening (read-from-string (read-line)))
    ;;   (when (numberp hand-opening)
    ;;     (send *ri* :move-gripper *grasp-hand* hand-opening)
    ;;     )
    ;;   )
    (send *ri* :move-gripper *grasp-hand* 0.14)
    (while (y-or-n-p "adjust spreader position? press y to adjust or n to finish")
      (format t "(x):x, (y):y, (z):z~%")
      (setq direction (read-line))
      (format t "How much to move? [mm]~%")
      (setq length (read-from-string (read-line)))
      (when (numberp length)
        (setq move-fv (float-vector 0 0 0))
        (cond
         ((equal direction "x") (setf (elt move-fv 0) length))
         ((equal direction "y") (setf (elt move-fv 1) length))
         ((equal direction "z") (setf (elt move-fv 2) length))
         (t )
         )
        (print move-fv)
        (send *robot* *grasp-hand* :move-end-pos move-fv :world)
        (send *irtviewer* :draw-objects)
        (if real
            (progn
              (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
              (send *ri* :wait-interpolation)
              )
          )
        )
      )
    )
  )


;; hold spreader and step back
(defun hold-spreader (&key (real nil) (up 200) (step-back 1.0) (base-time 5000) (limb :rarm) (grasp-point *spreader-body*))
  (when (not (boundp '*grasp-point*))
    (setq *grasp-point* grasp-point)
    )
  (when (not (boundp '*grasp-hand*))
    (setq *grasp-hand* limb)
    )
  (let* ((hand-opening) (direction) (length) (move-fv #f(0 0 0)) (target-coords))
    ;; (while (y-or-n-p "adjust the degree of hand-opening? press y to adjust or n to finish")
    ;;   (format t "How much to open? [0, 1]~%")
    ;;   (setq hand-opening (read-from-string (read-line)))
    ;;   (when (numberp hand-opening)
    ;;     (send *ri* :move-gripper *grasp-hand* hand-opening)
    ;;     )
    ;;   )
    (send *ri* :start-grasp)
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (if real
        (progn
          (send *ri* :set-ref-force #f(0 0 -140.14) 2000 *grasp-hand*)
          (send *ri* :wait-interpolation)
          )
      )
    (send *robot* *grasp-hand* :move-end-pos (float-vector 0 0 up) :world)
    (send *irtviewer* :draw-objects)
    (if real
        (progn
          (send *ri* :angle-vector (send *robot* :angle-vector) (* base-time 0.4))
          (send *ri* :wait-interpolation)
          (send *ri* :go-pos (- (abs step-back)) 0 0)
          )
      )
    ;; (when (y-or-n-p "finish and move spreader close to reset pose ?")
      ;; (send *robot* *grasp-hand* :move-end-pos (float-vector 0 (- 400 (elt (send *robot* *grasp-hand* :end-coords :worldpos) 1)) 0) :world)
      (let* ((target-coords (send *robot* *grasp-hand* :end-coords :copy-worldcoords)))
        (send target-coords :translate (float-vector 0 (- -400 (elt (send *robot* *grasp-hand* :end-coords :worldpos) 1)) 0) :world)
        (send *robot* *grasp-hand* :inverse-kinematics target-coords :rotation-axis :x :debug-view :no-message)
        (send *irtviewer* :draw-objects)
        )
      (if real
          (progn
            (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
            (send *ri* :wait-interpolation)
            )
        )
      (let* ((link-list (list limb :lleg :rleg)))
        (insert-spreader :place "use")
        (get-spreader-pose)
        (set-robot-eus :set-current-pose nil)
        (with-move-target-link-list
         (mt ll *robot* link-list)
         (send *robot*
               :fullbody-inverse-kinematics (list grasp-point *lfoot* *rfoot*)
               :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
               :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
               :centroid-thre 30
               :move-target mt
               :link-list ll
               :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                             (list (send *robot* :head-neck-y :child-link) 0))
               :rotation-axis (list t t t)
               ;; nozawa
               ;;:min-loop 2
               :cog-null-space nil
               :use-6dof-joint-leg-gain nil
               :debug-view :no-message) ;;t)
         )
        )
      (send *irtviewer* :draw-objects)
      (if real
          (progn
            (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
            (send *ri* :wait-interpolation)
            )
        )
      ;; )
    )
  )


;; use spreader by setting it toward triangle mark, using spreader marker
(defun use-spreader (&key (real nil) (base-time 5000) (grasp-point *spreader-body*) (limb :rarm))
  (send *ri* :set-st-param :emergency-check-mode 2)
  (send *ri* :set-st-param :cp-check-margin (* 60 1e-3))
  ;;(send *ri* :set-gait-generator-param :default-step-time 1.2) ;; default
  (when (not (boundp '*grasp-point*))
    (setq *grasp-point* grasp-point)
    )
  (when (not (boundp '*grasp-hand*))
    (setq *grasp-hand* limb)
    )
  (let* ((link-list (list limb :lleg :rleg)))
    (send *ri* :stop-impedance *grasp-hand*)
    (get-spreader-pose)
    (when (boundp '*marker-offset*)
      (send *spreader* :transform *marker-offset*)
      )
    (set-robot-eus :set-current-pose nil)
    (send *robot* :translate #f(0 0 -100) :world)
    (with-move-target-link-list
     (mt ll *robot* link-list)
     (send *robot*
           :fullbody-inverse-kinematics (list *grasp-point* *lfoot* *rfoot*)
           :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 30
           :move-target mt
           :link-list ll
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           ;; nozawa
           ;;:min-loop 2
           :cog-null-space nil
           :use-6dof-joint-leg-gain nil
           :debug-view :no-message) ;;t)
     )
    )
  (send *robot* :look-at-target *spreader-head*)
  (if real
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )
  )

;; remove offset by interacative markar
(defun offset-remove ()
  (let* ((before-spreader-pose (send *spreader* :copy-worldcoords))
         (after-spreader-pose)
         )
    (get-spreader-pose)
    (setq after-spreader-pose (send *spreader* :copy-worldcoords))
    (send *spreader* :move-to before-spreader-pose :world)
    (setq *marker-offset* (send after-spreader-pose :transformation before-spreader-pose))
    (print *marker-offset*)
    )
  )


;; rotate spreader in the spreader-head coords
(defun move-head-rot (&key (real nil) (base-time 1000) (grasp-point *spreader-body*) (limb :rarm))
  (when (not (boundp '*grasp-point*))
    (setq *grasp-point* grasp-point)
    )
  (when (not (boundp '*grasp-hand*))
    (setq *grasp-hand* limb)
    )
  (let* ((spreader-coords (make-cascoords))
         (spreader-head (make-cascoords))
         )
    (send spreader-coords :move-to (send *spreader* :copy-worldcoords) :world)
    (send spreader-head :move-to (send *spreader-head* :copy-worldcoords) :world)
    (send spreader-head :assoc spreader-coords)
    (let* ((axis) (ang))
      (while (y-or-n-p "move spreader head rotation? press y to adjust or n to finish")
        (format t "(x):x, (y):y, (z):z~%")
        (setq axis (read-line))
        (format t "How much to rotate? [deg]~%")
        (setq ang (read-from-string (read-line)))
        (when (numberp ang)
          (cond
           ((equal axis "x") (send spreader-head :rotate (deg2rad ang) :x))
           ((equal axis "y") (send spreader-head :rotate (deg2rad ang) :y))
           ((equal axis "z") (send spreader-head :rotate (deg2rad ang) :z))
           (t )
           )
          ;; (print ang)
          (send *spreader* :move-to (send spreader-coords :copy-worldcoords) :world)
          (send *irtviewer* :draw-objects)
          )
        )
      )
    (send spreader-head :dissoc spreader-coords)
    )
  ;;(use-spreader :real real :base-time base-time :grasp-point *grasp-point* :limb :rarm)
  (let* ((link-list (list limb :lleg :rleg)))
    (when (boundp '*marker-offset*)
      (send *spreader* :transform *marker-offset*)
      )
    (set-robot-eus :set-current-pose nil)
    (with-move-target-link-list
     (mt ll *robot* link-list)
     (send *robot*
           :fullbody-inverse-kinematics (list *grasp-point* *lfoot* *rfoot*)
           :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 30
           :move-target mt
           :link-list ll
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           ;; nozawa
           ;;:min-loop 2
           :cog-null-space nil
           :use-6dof-joint-leg-gain nil
           :debug-view :no-message) ;;t)
     )
    )
  )


;; throw away spreader and reset ref-force
(defun throw-away-spreader (&key (real nil) (base-time 3000) (grasp-point *spreader-body*) (limb :rarm))
  (when (not (boundp '*grasp-point*))
    (setq *grasp-point* grasp-point)
    )
  (when (not (boundp '*grasp-hand*))
    (setq *grasp-hand* limb)
    )
  (if real
      (progn
        (send *ri* :stop-grasp)
        (send *ri* :set-ref-force #f(0 0 0) 2000 *grasp-hand*)
        )
    )
  (let* (
         (spreader-pose)
         (link-list (list limb :lleg :rleg))
         )
    (if (equal limb :rarm)
        (setq spreader-pose (make-cascoords :pos #f(400 -400 550) :rpy (list (deg2rad 90) (deg2rad 0) (deg2rad -90))))
      (setq spreader-pose (make-cascoords :pos #f(700 100 550) :rpy (list (deg2rad 90) (deg2rad 0) (deg2rad -90))))
      )
    (send *spreader* :move-to (send spreader-pose :copy-worldcoords) :world)
    (send *irtviewer* :draw-objects)

    (set-robot-eus :set-current-pose nil)
    (with-move-target-link-list
     (mt ll *robot* link-list)
     (send *robot*
           :fullbody-inverse-kinematics (list *grasp-point* *lfoot* *rfoot*)
           :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 30
           :move-target mt
           :link-list ll
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           ;; nozawa
           ;;:min-loop 2
           :cog-null-space nil
           :use-6dof-joint-leg-gain nil
           :debug-view :no-message) ;;t)
     )
    )
  (if real
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )
  )


(defun select-func ()
  (let* ((selection) (real nil) (limb :larm) (grasp-point *spreader-grip*))
    (while t
      (format t "(i):insert-spreader (r):reach-spreader (a):adjust-spreader (h):hold-spreader (u): use-spreader (o): offset-remove (m): move-head-rot (t): throw-away-spreader (e): emergency-release~%")
      (setq selection (read-line))
      (cond
       ((equal selection "i")
        (when (y-or-n-p "insert-spreader?")
          (if (y-or-n-p "for using?")
              (insert-spreader :place "use")
            (insert-spreader :place "grasp")
            )
          ))
       ((equal selection "r")
        (when (y-or-n-p "reach-spreader?")
          (when (y-or-n-p ":real t?")
            (setq real t)
            )
          (when (y-or-n-p ":limb :rarm?")
            (setq limb :rarm)
            )
          (when (y-or-n-p ":grasp-point *spreader-body*?")
            (setq grasp-point *spreader-body*)
            )
          (reach-spreader :real real :limb limb :grasp-point grasp-point)))
       ((equal selection "a")
        (when (y-or-n-p "adjust-spreader?") (adjust-spreader :real t)))
       ((equal selection "h")
        (when (y-or-n-p "hold-spreader?")
          (when (y-or-n-p ":real t?")
            (setq real t)
            )
          (hold-spreader :real real)))
       ((equal selection "u")
        (when (y-or-n-p "use-spreader?")
          (when (y-or-n-p ":real t?")
            (setq real t)
            )
          (use-spreader :real real)))
       ((equal selection "o")
        (when (y-or-n-p "offset-remove?")
          (offset-remove)))
       ((equal selection "m")
        (when (y-or-n-p "move-head-rot?")
          (when (y-or-n-p ":real t?")
            (setq real t)
            )
          (move-head-rot :real real)))
       ((equal selection "t")
        (when (y-or-n-p "throw-away-spreader?")
          (when (not (y-or-n-p ":real nil?"))
            (setq real t)
            ;; (when (y-or-n-p ":rarm?")
            ;;   (setq limb :rarm)
            ;;   )
            )
          (throw-away-spreader :real real :limb :rarm)))
       ((equal selection "e")
        (when (y-or-n-p "emergency-release?")
          (send *ri* :emergency-release-motion)))
       (t )
       )
      (setq real nil)
      (setq limb :larm)
      (setq grasp-point *spreader-grip*)
      (when (= (send *ri* :state :emergency-mode) 1)
        (print "emergency-mode working!!")
        )
      )
    )
  )
