#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/generate-drill-motion.l")
(setq *robot-name* "HRP2JSKNTS")

(generate-robot *robot-name* :generate-ri? nil)
(set-drill-environment :draw? nil)
(send *robot* :angle-vector (get-watch-drill-pose))

(setq angle (round (ros::get-param "/angle" 0)))

(defun check-reachability-fot-button
  (&key (width 30) (arm *drill-main-arm*) angle-vector (draw? t))
  (when angle-vector (send *robot* :angle-vector angle-vector))
  (let ((success-num 0) (current-av (send *robot* :angle-vector)))
    (dotimes (ix 2)
      (dotimes (iy 2)
	(dotimes (iz 2)
          (dotimes (ir 3)
            (send *robot* :angle-vector current-av)
            (set-drill-default-coords)
            (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects) (objects *drill*)))
            (send *robot* :angle-vector (get-watch-drill-pose))
            (send *robot* :fix-leg-to-coords (make-coords))
            (send *drill* :transform (make-coords :pos (float-vector (* width (- ix 0.5)) (* width (- iy 0.5)) (* width (- iz 0.5)))
                                                  :rpy (list (deg2rad (* 15 (- ir 1))) (deg2rad (* 45 0)) (deg2rad (* 15 (- 1 0))))))
            (if (and (fullbody-inverse-kinematics-for-drill-button :draw? draw? ;; :x-offset (* width (- ix 1)) :y-offset (* width (- iy 1)) :z-offset (* width (- iz 1))
                                                                   )) (not (send *robot* :self-collision-check)) (not (c-check-for-drill)))
                (setq success-num (+ 1 success-num))
              )
            )))
    success-num
    
    )
  )
  

(defun rotate-grasp-coords
  (deg)
  (send (send *drill* :get :grasp-coords) :rotate (deg2rad deg) :z)
  )

(defun full-check
  (deg &key (draw? nil))
  (load (format nil "/home/leus/hrp2_dataset/pre-angle~A.l" deg))
  (let ((max-num -1) (max-index-list nil))
    (rotate-grasp-coords deg)
    (dotimes ;(i 1)
        (length *answers*)
      )
      (let ((num (check-reachability-fot-button :angle-vector (elt *answers* i) :draw? draw?)))
        (cond (
               (> num max-num)
               (setq max-num num)
               (setq max-index-list (list i))
               )
              ((equal num max-num)
               (push i max-index-list)
               )
              ))
      )
    
    (format t "max-num ~A, max-index ~A" max-num max-index-list)
    (let ((true-answers nil))
      (dotimes (i (length max-index-list))
        (push (elt *answers* (elt max-index-list i)) true-answers)
        )
      (setq *true-answers-list* nil)
      (load "/home/leus/hrp2dataset/true-angle.l")
      (push (list deg max-num true-answers) *true-answers-list*) 
      (dump-loadable-structure (format nil "/home/leus/hrp2dataset/true-angle.l") *true-answers-list*)
      )
    (list max-num max-index-list)
    )
  )
(full-check angle)
