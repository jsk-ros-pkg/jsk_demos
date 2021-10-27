#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_robot_startup")
(ros::load-ros-manifest "geometry_msgs")

(defparameter *spots* nil)

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*
(objects (list *fetch* *scene*))
(send *ri* :objects (send *scene* :objects))

(defun view-fetch ()
  (send *irtviewer* :look-all
        (send (geo::make-bounding-box
               (flatten (send-all (send *fetch* :bodies) :vertices))) :grow 2.0)))

(defun move-dock-front ()
  (setq *dock-front*
        (send (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :copy-worldcoords))
  (send *fetch* :move-to *dock-front* :world)
  (send *ri* :move-to *dock-front*))

(defun move-73b2-door-front ()
  (setq *73b2-door-front*
        (send (send *scene* :spot "door-spot") :copy-worldcoords))
  (send *fetch* :move-to *73b2-door-front* :world)
  (send *ri* :move-to *73b2-door-front*))

(defun get-spot-coords (name)
  (let ((spot-coords nil))
    (setq spot-coords (send (send *scene* :spot name) :copy-worldcoords))
    spot-coords))

(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (clear-costmap t))
  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  ;;(if clear-costmap (send *ri* :clear-costmap))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose ret))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *fetch* :move-to goal-pose :world)
    (send *ri* :move-to goal-pose)))

(defun door-open ()
  (send *ri* :stop-grasp)
  (go-to-spot "door-spot" :relative-pos #f(200 -300 0))

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords :pos #f(700 70 950) :rpy (float-vector 0 0 -pi/2))))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :move-end-pos #f(100 0 0))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords :pos #f(800 70 890) :rpy (float-vector 0 0 (* 2 (/ -pi 3))))))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :angle-vector-sequence
        (list (send *fetch* :rarm :move-end-pos #f(-50 0 0))
              (send *fetch* :rarm :move-end-pos #f(-50 0 0)))
        (list 3000 3000))
  ;; (send *fetch* :rarm :move-end-pos #f(-100 0 0))
  ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; opening door
  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords :pos #f(700 70 950) :rpy (float-vector 0 0 -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 12)))) (+ 70 (- 700 (* 700 (cos (/ pi 12))))) 950)
               :rpy
               (float-vector (/ -pi 12) 0 -pi/2))) :rotation-axis t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 8)))) (+ 70 (- 700 (* 700 (cos (/ pi 8))))) 950)
               :rpy
               (float-vector (/ -pi 6) 0 -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 6)))) (+ 70 (- 700 (* 700 (cos (/ pi 6))))) 950)
               :rpy
               (float-vector (/ -pi 5) 0 -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 5)))) (+ 70 (- 700 (* 700 (cos (/ pi 5))))) 950)
               :rpy
               (float-vector (/ -pi 5) 0 -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 4)))) (+ 70 (- 700 (* 700 (cos (/ pi 4))))) 950)
               :rpy
               (float-vector (/ -pi 4) 0 -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; TODO
  )
