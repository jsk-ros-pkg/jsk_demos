#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*

;; show in irtviewer
(objects (list *fetch* *scene*))
;; show in "fetch Kinematics Simulator"
(send *ri* :objects (send *scene* :objects))

(defun move-dock-front ()
  (setq *dock-front* (send (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :copy-worldcoords))
  (send *fetch* :move-to *dock-front* :world))

(send *irtviewer* :look-all
      (send (geo::make-bounding-box
             (flatten (send-all (send *fetch* :bodies) :vertices)))
            :grow 2.0))

(defun move-73-A2-front ()
  (setq *room73A2-front* (send (send *scene* :spot "/eng2/7f/room73A2-front") :copy-worldcoords))
  (send *fetch* :move-to *room73A2-front* :world)
  ;;(send *ri* :move-to *room73A2-front* :world)
  )
;;(send *fetch* :locate #f(300 0 0))
(
;;(send *ri* :move-to *room73A2-front* :world)
#|
(setq *room73A3-front* (send (send *scene* :spot "/eng2/7f/room73A3-front") :copy-worldcoords))
(send *fetch* :move-to *room73A3-front* :world)

|#

(defun pose-of-give ()
  (send *ri* :start-grasp :effort 60)
  (unix:sleep 1) 
  ;;(send *fetch* :torso_lift_joint :joint-angle 60)
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  ;;(send *fetch* :stop-grasp)
  (send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 100 1000) :rpy (float-vector 0 0 pi/2))) :debug-view t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t)
  ;;(send *fetch* :wrist_roll_joint :joint-angle 0)
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t)
  (send *ri* :stop-grasp)
  #|
  (send *ri* :wait-interpolation t)
  (send *ri* :start-grasp)
  (send *ri* :wait-interpolation t)
  |#
  #|
  (send *fetch* :torso_lift_joint :joint-angle 60)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t)
  |#
  ;;(send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
  )
  ;;(send *fetch* :rarm :angle-vector)
  
#|
(defun raise-hand ()
  (send *fetch* :angle-vector #f(5.0000 88.0000 -42.0000 0 -60 1 -77.0000 7.000 0 2.0000)))
|#
#|
(defun catch ()
  (send *fetch* :angle-vector #f(5 52 3 77 -59 10 -78 74 0 2)))
|#
