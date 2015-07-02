#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-motion-on-polaris-xp900.l")
(load "package://drc_task_common/euslisp/vehicle/staro-driving-motion-on-polaris-xp900.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsknt-driving-motion-on-polaris-xp900.l")

(setq *motion* nil)
(setq *ik-status* nil)
(setq *view-box-list** nil)

(defun init-jaxon
  ()
  (setq *motion* (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil))
  )

(defun init-hrp2jsknt
  ()
  (setq *motion* (instance hrp2jsknt-driving-motion-on-polaris-xp900 :init :real nil))
  )

(defun init-staro
  ()
  (setq *motion* (instance staro-driving-motion-on-polaris-xp900 :init :real nil))
  )

(defun evaluate
  (limb x y)
  (print (float-vector x y 0))
  (send *motion* :ride :init-pose t)
  (send *motion* :move-robot-position (float-vector x y 0))
  (send *motion* :drive-init-pose)
  (send *motion* :drive-init-pose-crank :limb limb)
  (send *motion* :car :reset)
  (let ((retval nil)
        (retval-vector (list nil nil nil nil))
        )
    (block evaluation
      (setq retval (send *motion* :approach-handle :limb limb))
      (if retval
          (setf (elt retval-vector 0) retval)
        (progn (warn "evaluate: failed to apprach~%") (return-from evaluation nil))
        )
      (setq retval (send *motion* :grasp-handle :limb limb))
      (if retval
          (setf (elt retval-vector 1) retval)
        (progn (warn "evaluate: failed to grasp~%") (return-from evaluation nil))
        )
      (setq retval (send *motion* :turn-handle 360 :limb limb))
      (setf (elt retval-vector 2) retval)
      (when (> (abs retval) 0.0)
        (send *motion* :turn-handle 0.0 :limb limb :relative nil)
        )
      (setq retval (send *motion* :turn-handle -360 :limb limb))
      (setf (elt retval-vector 3) retval)
      )
    retval-vector
    )
  )

(defun make-view-box
  (status-list &key (z-offset 0.0) (box-size 30)) ;; status-list assumed to be list of (approach grasp turn-plus turn-minus)
  (let* ((view-box (make-cube box-size box-size box-size))
         (robot-coords (send (car (send *motion* :robot :links)) :copy-worldcoords)) ;; use body link
         (robot-pos (send robot-coords :pos)) box-pos
         tmp-status (tmp-color (float-vector 0 0 0))
         )
    (setq box-pos robot-pos)
    (setf (elt box-pos 2) (+ (elt robot-pos 2) z-offset))
    (send view-box :newcoords (make-coords :pos box-pos))
    ;; set box color according to ik status
    (block coloring
      (setq tmp-status (pop status-list))
      (unless tmp-status
        (setf (elt tmp-color 0) 0.5)
        (send view-box :set-color tmp-color)
        (return-from coloring view-box)
        )
      (setq tmp-status (pop status-list))
      (unless tmp-status
        (setf (elt tmp-color 0) 1.0)
        (send view-box :set-color tmp-color)
        (return-from coloring view-box)
        )
      (setq tmp-status (pop status-list))
      (setf (elt tmp-color 1) (- 1.0 (/ tmp-status 360.0)))
      (setq tmp-status (pop status-list))
      (setf (elt tmp-color 2) (- 1.0 (/ tmp-status 360.0)))
      (send view-box :set-color tmp-color)
      )
    view-box
    )
  )

(defun main
  (&key (resolution 30.0) (times 8) (limb :rarm) (z-offset 0))
  (setq *ik-status* nil)
  (setq *view-box-list* nil)
  (let ((offset (* resolution (/ times 2)))
        tmp-status-list tmp-view-box)
    (dotimes (x times)
      (dotimes (y times)
        (setq tmp-status-list (evaluate limb (- (* x resolution) offset) (- (* y resolution) offset)))
        (push tmp-status-list *ik-status*)
        (setq tmp-view-box (make-view-box tmp-status-list :z-offset z-offset))
        (push tmp-view-box *view-box-list*)
        )
      )
    )
  (objects (append (list (send *motion* :car)) *view-box-list*))
  )


