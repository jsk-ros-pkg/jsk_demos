#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")

(load "package://spoteus/spot-interface.l")
(spot-init)

(ros::roseus "test")



(defun move-to-human (msg)
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
	  (setq *angle* (* (* -1.0 px) (/ 180.0 2000.0)))
	  (print *angle*)
	  (send *ri* :go-pos 0 0 *angle*)
	  (send *ri* :go-pos 0.8 0 0)
	  ;;(while (<= (area-of-rects max-rect) 80000)
	    ;;(send *ri* :go-pos 0.5 0 0)
	    ;;)
	  (send *ri* :speak-jp "どちらにいきますか？")
	  ;;(if (>= (area-of-rects max-rect) 80000)
	      ;;(send *ri* :go-pos 0 0 0)
	    ;;)
	  ;;(dolist (x sorted-list)
	    ;;(progn
	      ;;(print (area-of-rects x))
	      ;;(print px)
	      ;;(print py)
              ;;(send *ri* :go-pos 0 0 (* (* -1.0 px) (/ 180.0 2000.0)))
	    ;;))
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


(setq *human* (one-shot-subscribe "human" jsk_recognition_msgs::RectArray))

(move-to-human *human*)

;; (ros::subscribe "/human" jsk_recognition_msgs::RectArray #'cb)
(ros::spin)
