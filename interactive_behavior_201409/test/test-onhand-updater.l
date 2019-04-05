#!/usr/bin/env roseus
;; test-onhand-updater.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "test_onhand_updater")

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")

(setq init '((at table)
             (onhand bottle larm)
             (onhand cup rarm)))
(pprint (update-condition init))
(setq u (get-updater onhand-updater))
