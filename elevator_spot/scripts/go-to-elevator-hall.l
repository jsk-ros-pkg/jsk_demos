#!/usr/bin/env roseus
(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

(setq *go-to-elevator-hall* t)

(setq *autowalk-73b2-to-7f-elevator-hall* "package://spot_elevator/autowalk/73b2-to-7f-elevator-hall")

(defun go-to-elevator-hall(floor)
  ;; Go to elevator hall
  (let autowalk-file-path
    (cond ((= floor 7))
          (setq autowalk-file-path *autowalk-73b2-to-7f-elevator-hall*))
    (send *ri* :navigate-to autowalk-file-path -1 :initial-localizaiton-fiducial t :initial-localization-waypoint 0)))

