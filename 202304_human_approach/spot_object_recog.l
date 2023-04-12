#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::roseus "test")

(defun cb (msg)
  (let (rect-list sorted-list max-rect pu pv px py)
    (setq rect-list (send msg :rects))
    (if (not (= (length rect-list) 0))
	(progn
	  (setq sorted-list (sort rect-list #'compare-area-of-rects))
	  (setq max-rect (car sorted-list))
	  (setq pu (+ (send max-rect :x) (/ (send max-rect :width) 2)))
	  (setq pv (+ (send max-rect :y) (/ (send max-rect :height) 2)))
	  (setq px (convert-x pu))
	  (setq py (convert-y pv))
	  (dolist (x sorted-list)
	    (progn
	      (print (area-of-rects x))
	      (print px)
	      (print py)
	    ))
	  (print "done")
	)
      )
    )
  )
;;(print (sen (car (send msg :rects)) :y))))
(defun area-of-rects (x)
  (* (send x :width) (send x :height))
  )
(defun compare-area-of-rects (a b)
  (>= (* (send a :width) (send a :height)) (* (send b :width) (send b :height))))

(defun convert-x (u)
  (- u 2000)
  )
(defun convert-y (v)
  (+ (* -1 v) 1000)
  )




(ros::subscribe "/human" jsk_recognition_msgs::RectArray #'cb)
(ros::spin)
