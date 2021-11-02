#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_robot_startup")
(ros::load-ros-manifest "geometry_msgs")

(defparameter *spots* nil)
(defvar *topic-name* "/door_detector/euclidean_clustering_decomposer/boxes")
(defvar *bounding-box-list* nil)
(setq time-stamp (ros::time 0))

(defun get-spot-coords (name)
  (let ((spot-coords nil))
    (setq spot-coords (send (send *scene* :spot name) :copy-worldcoords))
    spot-coords))

(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (clear-costmap t))
  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (if clear-costmap (send *ri* :clear-costmap))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose ret))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *fetch* :move-to goal-pose :world)
    (send *ri* :move-to goal-pose)))

(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes))
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (when (ros::time> time-stamp (send msg :header :stamp))
        (return-from bounding-box-array-cb nil))
      (ros::ros-info "tf")
      (setq *obj-pos* (send (send base-coords
                                :transform base->obj-coords) :worldpos))
      (setq found-obj t)
      (ros::ros-info "obj-pos:~A" *obj-pos*)
      (ros::ros-info "base-coords:~A" base-coords)
      (ros::ros-info "base->obj-coords:~A" base->obj-coords)
      *obj-pos*)))

(defun door-open-with-rec(obj-pos)
  (let ((cam-coords nil)
        (obj-coords (make-coords :pos obj-pos :rpy #f(0 0 0))))
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords (send *tfl*
                           :lookup-transform
                           "/base_link" "/head_camera_depth_optical_frame"
                           (ros::time 0)))
    (send obj-coords :transform cam-coords :world)
    (ros::ros-info "obj-coords: ~A" obj-coords)
    (send obj-coords :transform (make-coords :pos #f(-120 -10 0)) :world)
    (send *fetch* :rarm :inverse-kinematics
          (make-coords :pos (send obj-coords :pos)
                       :rpy (float-vector 0 0 pi/2)) :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ;; reaching
    (send *fetch* :rarm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (unix::sleep 2)
    (send *ri* :start-grasp)
    ;; arm servo off
    (send *ri* :servo-off :gripper nil :head nil)
    (unix::sleep 1)
    ;; opening the door by using base moving
    (send *ri* :go-velocity -0.2 0 0 4000)
    (send *ri* :go-velocity 0 0 -0.25 6280) ;; pi/2 rotation
    (send *ri* :go-velocity -0.1 0 0 15000)

    ;; release handle
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)

    (send *ri* :stop-grasp)
    (send *fetch* :rarm :move-end-pos #f(-100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)))

(defun door-close-with-rec (obj-pos)
  (let ((cam-coords nil)
        (obj-coords (make-coords :pos obj-pos :rpy #f(0 0 0))))
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords (send *tfl*
                           :lookup-transform
                           "/base_link" "/head_camera_depth_optical_frame"
                           (ros::time 0)))
    (send obj-coords :transform cam-coords :world)
    (ros::ros-info "obj-coords: ~A" obj-coords)
    (send obj-coords :transform (make-coords :pos #f(-120 -10 0)) :world)
    (send *fetch* :rarm :inverse-kinematics
          (make-coords :pos (send obj-coords :pos)
                       :rpy (float-vector 0 0 pi/2)) :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)

    (send *fetch* :rarm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (unix::sleep 2)))

(defun door-open-not-recog ()
;;  (send *ri* :stop-grasp)
  (go-to-spot "door-spot" :relative-pos #f(120 -700 0)) ;; adjust environment

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
        (send (send *fetch* :copy-worldcoords)
              :transform (make-coords :pos #f(800 70 890)
                                      :rpy (float-vector 0 0 (* 2 (/ -pi 3))))))
  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords)
              :transform (make-coords :pos #f(800 70 890)
                                      :rpy (float-vector 0 (/ pi 4) (* 2 (/ -pi 3))))))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; (send *ri* :angle-vector-sequence
  ;;       (list (send *fetch* :rarm :move-end-pos #f(-50 0 0))
  ;;             (send *fetch* :rarm :move-end-pos #f(-50 0 0)))
  ;;       (list 3000 3000))
  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords)
              :transform (make-coords :pos #f(700 70 890)
                                      :rpy (float-vector 0 (/ pi 4) (* 2 (/ -pi 3))))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; opening door
  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords)
              :transform (make-coords :pos #f(700 70 950)
                                      :rpy (float-vector 0 (/ pi 4) -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 12)))) (+ 70 (- 700 (* 700 (cos (/ pi 12))))) 950)
               :rpy (float-vector (/ -pi 12) (/ pi 4) -pi/2))) :rotation-axis t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 8)))) (+ 70 (- 700 (* 700 (cos (/ pi 8))))) 950)
               :rpy (float-vector (/ -pi 6) (/ pi 4) -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 6)))) (+ 70 (- 700 (* 700 (cos (/ pi 6))))) 950)
               :rpy (float-vector (/ -pi 5) (/ pi 4) -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 5)))) (+ 70 (- 700 (* 700 (cos (/ pi 5))))) 950)
               :rpy (float-vector (/ -pi 5) (/ pi 4) -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *fetch* :rarm :inverse-kinematics
        (send (send *fetch* :copy-worldcoords) :transform
              (make-coords
               :pos
               (float-vector
                (- 800 (* 700 (sin (/ pi 4)))) (+ 70 (- 700 (* 700 (cos (/ pi 4))))) 950)
               :rpy (float-vector (/ -pi 4) (/ pi 4) -pi/2))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation))

(defun 73b2-door-open()
  (setq *obj-pos* nil)
  (ros::subscribe *topic-name*
                  jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-array-cb 1)
  (while (not *obj-pos*)
    (ros::spin-once))
  (door-open-with-rec *obj-pos*))

(defun 73b2-door-close()
  (setq *obj-pos* nil)
  (ros::subscribe *topic-name*
                  jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-array-cb 1)
  (while (not *obj-pos*)
    (ros::spin-once))
  (door-close-with-rec *obj-pos*))
