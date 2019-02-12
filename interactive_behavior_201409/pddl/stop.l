#!/usr/bin/env roseus
;; stop.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")


(pr2-init)

(setq cds nil)

(while (ros::ok)
  (ros::rate 1)
  (ros::sleep)
  (unless cds
    (setq cds (send *ri* :state :worldcoords)))
  (setq cds2 (send *ri* :state :worldcoords))
  (when (and cds cds2)
    (setq diff (norm (send cds :difference-position cds2)))
    (ros::ros-info "diff: ~A" diff)
    (when (> diff 1000)
      (send *ri* :preempt :speech)
      (exit 0))))
