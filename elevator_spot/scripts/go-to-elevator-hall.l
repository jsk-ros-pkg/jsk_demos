#!/usr/bin/env roseus
(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

(setq *go-to-elevator-hall* t)

(defun go-to-elevator-hall(floor)
  ;; Go to elevator hall
  (let autowalk-file-path
    (cond ((= floor 7))
          (setq autowalk-file-path "package://spot_elevator/autowalk/"))
    (send *ri* :navigate-to autowalk-file-path -1 :initial-localizaiton-fiducial t :initial-localization-waypoint 0)))

