#!/usr/bin/env roseus


(defun ir2ri (&key (time 5000) (wait-interpolation t))
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  (when wait-interpolation
    (send *ri* :wait-interpolation)))


(defun ri2ir (&key (time 5000))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
