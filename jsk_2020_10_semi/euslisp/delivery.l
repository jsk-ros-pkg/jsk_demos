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
  (send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 300 1000))) :debug-view t))

#|
(defun raise-hand ()
  (send *fetch* :angle-vector #f(5.0000 88.0000 -42.0000 0 -60 1 -77.0000 7.000 0 2.0000)))
|#
#|
(defun catch ()
  (send *fetch* :angle-vector #f(5 52 3 77 -59 10 -78 74 0 2)))
|#

#| 顔を検知してそこに描かれた長方形の面積を求める |#
(defclass face_area
  :slots (*rect*)
  )
(defmethod face_area
  (:init ()
	 (ros::roseus "face_recognized_client")
	 (ros::subscribe "edgetpu_face_detector/output/rects" jsk_recognition_msgs::RectArray #'send self :cb)
	 (setq *rect* nil)
	 (ros::ros-info "init")
	 self)
  (:cb (msg)
       (setq *rect* (send msg :rects))
       )
  (:loop ()
	 (ros::rate 10)
	 (while (ros::ok)
	   (if *rect*
	       (progn
		 (setq w (send (elt *rect* 0) :width))
		 (setq h (send (elt *rect* 0) :height))
		 (setq *area* (* w h))
		 (ros::ros-info "area : ~A" *area*)
		 )
	     )
	     (ros::spin-once)
	     (ros::sleep)
	    )
	 (exit)
	 )
  )
