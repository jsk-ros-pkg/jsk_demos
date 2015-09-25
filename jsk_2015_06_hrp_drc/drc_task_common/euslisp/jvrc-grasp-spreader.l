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
  )

(when (not (boundp '*spreader*))
  (setq *spreader* (spreader))
  ;; totte
  ;; (setq *spreader-grip* (make-cascoords :pos #f(0 170 140) :rpy (list 0 (deg2rad 90) (deg2rad 90))))
  ;; hontai
  (setq *spreader-grip* (make-cascoords :pos #f(0 40 60) :rpy (list 0 (deg2rad 90) (deg2rad 90))))
  (send *spreader* :assoc *spreader-grip*)
  )
(when (not (boundp '*tf*))
  (setq *tf* (instance ros::transform-listener :init))
  )

(objects (list *robot* *spreader* *spreader-grip*))

;; insert spreader marker
(defun insert-spreader ()
  (insert-primitive-marker :shape-type 3 :name "spreader" :description "" :resource "package://jvrc_models/models/spreader-visual.dae")
  )

;; get robot position in eus
(defun set-robot-eus ()
  (send *robot* :locate #f(0 0 0) :world)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *lfoot* (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq *rfoot* (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :fix-leg-to-coords (make-coords))
  )

;; get spreader marker pose
(defun get-spreader-pose (&key (frame-id "BODY"))
  (let* (
         (spreader-pose (get-primitive-marker-pose :name "spreader" :frame-id frame-id))
         ;; (spreader-pose (make-cascoords :pos #f(600 -200 -300) :rpy (list (deg2rad 90) (deg2rad 0) 0)));; (list (deg2rad 180) (deg2rad 90) 0)))
         )
    (set-robot-eus)
    (setq spreader-pose (send spreader-pose :copy-worldcoords))

    (send spreader-pose :transform (send *robot* :body_lk :worldcoords) :world)
    (send *spreader* :move-to (send spreader-pose :copy-worldcoords) :world)
    )
  )

;; reach spreader
(defun reach-spreader (&key (ang 0) (real nil) (base-time 5000))
  (send *ri* :stop-impedance :rarm)
  (get-spreader-pose)
  (let* ((reach-z (send *spreader-grip* :rotate-vector (float-vector 0 0 1)))
         (reach-y (v* reach-z (float-vector 0 0 -1)))
         (reach-x (v* reach-y reach-z))
         (reach-r (transpose (matrix reach-x reach-y reach-z)))
         (spreader-reach (make-cascoords :pos (send *spreader-grip* :worldpos) :rot reach-r))
         )
    ;; (setq *lfoot* (send *robot* :lleg :end-coords :copy-worldcoords))
    ;; (setq *rfoot* (send *robot* :rleg :end-coords :copy-worldcoords))
    ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))

    (send spreader-reach :translate #f(0 0 150) :world)
    (send spreader-reach :rotate (deg2rad ang) :z)

    ;; fullbody-ik (rarm will be above spreader, and feet on ground)
    (with-move-target-link-list
     (mt ll *robot* '(:rarm :lleg :rleg))
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
           :min-loop nil
           :debug-view :no-message) ;;t)
     ))
  (if real
      (progn
        (send *ri* :stop-grasp)
        (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        (send *ri* :start-impedance :rarm)
        )
      )
  )


;; adjust position and the degree of hand opening
(defun adjust-spreader (&key (real nil) (base-time 1000))
  (let* ((hand-opening) (direction) (length) (move-fv #f(0 0 0)) (target-coords))
    (while (y-or-n-p "adjust the degree of hand-opening? press y to adjust or n to finish")
      (format t "How much to open? [0, 1]~%")
      (setq hand-opening (read-from-string (read-line)))
      (when (numberp hand-opening)
        (send *ri* :move-gripper :rarm hand-opening)
        )
      )
    (while (y-or-n-p "adjust spreader position? press y to adjust or n to finish")
      (format t "(x):x, (y):y, (z):z~%")
      (setq direction (read-line))
      (format t "How much to move? [mm]~%")
      (setq length (read-from-string (read-line)))
      (when (numberp length)
        (setq move-fv #f(0 0 0))
        (cond
         ((equal direction "x") (setf (elt move-fv 0) length))
         ((equal direction "y") (setf (elt move-fv 1) length))
         ((equal direction "z") (setf (elt move-fv 2) length))
         (t )
         )
        (setq target-coords (send *robot* :rarm :end-coords :copy-worldcoords))
        (send target-coords :translate move-fv :world)
        (send *robot* :rarm :inverse-kinematics target-coords :debug-view :no-message)
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
(defun hold-spreader (&key (real nil) (up 200) (step-back 1.0) (base-time 5000))
  (let* ((hand-opening) (direction) (length) (move-fv #f(0 0 0)) (target-coords))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :rarm :move-end-pos (float-vector 0 0 up) :world)
    (send *irtviewer* :draw-objects)
    (if real
        (progn
          (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
          (send *ri* :wait-interpolation)
          (send *ri* :go-pos (- (abs step-back)) 0 0)
          )
      )
    (when (y-or-n-p "finish and move spreader close to reset pose ?")
      ;; (send *robot* :rarm :move-end-pos (float-vector 0 (- 400 (elt (send *robot* :rarm :end-coords :worldpos) 1)) 0) :world)
      (let* ((target-coords (send *robot* :rarm :end-coords :copy-worldcoords)))
        (send target-coords :translate (float-vector 0 (- -400 (elt (send *robot* :rarm :end-coords :worldpos) 1)) 0) :world)
        (send *robot* :rarm :inverse-kinematics target-coords :rotation-axis :x :debug-view :no-message)
        (send *irtviewer* :draw-objects)
        )
      (if real
          (progn
            (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
            (send *ri* :wait-interpolation)
            )
        )
      )
    )
  )


;; use spreader by setting it toward triangle mark, using spreader marker
(defun use-spreader (&key (real nil) (base-time 5000))
  (send *ri* :stop-impedance :rarm)
  (get-spreader-pose)
  (with-move-target-link-list
   (mt ll *robot* '(:rarm :lleg :rleg))
   (send *robot*
         :fullbody-inverse-kinematics (list *spreader-grip* *lfoot* *rfoot*)
         :root-link-virtual-joint-weight #f(0.4 0.4 0.4 0.2 0.2 0.4) ;; root-link has 6DOF
         :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
         :centroid-thre 30
         :move-target mt
         :link-list ll
         :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                       (list (send *robot* :head-neck-y :child-link) 0))
         :rotation-axis (list t t t)
         :min-loop nil
         :debug-view :no-message) ;;t)
   )
  (if real
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )
  )


