;; (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

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
        (send (geo::make-bounding-box (flatten (send-all (send *fetch* :bodies) :vertices))) :grow 2.0)))

(defun move-dock-front ()
  (setq *dock-front* (send (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :copy-worldcoords))
  (send *fetch* :move-to *dock-front* :world)
  (send *ri* :move-to *dock-front*))

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
    (send *ri* :move-to goal-pose)))

(defun move-to-spot-front ()
  (send *ri* :go-pos-unsafe 0 0 -90))
;; (setq offset #f(-450 -400 0))
;;(setq success (go-to-spot "/eng2/7f/room73B2-microwave-front" :relative-pos offset :relative-rot -pi/2))
