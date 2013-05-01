#!/usr/bin/env roseus

(defun gripper-z-axis-inversed-p (arm)
  (let ((r (send (send *pr2* arm :end-coords) :rotate-vector #f(0 0 1))))
    (if (> (elt r 2) 0) t nil)))