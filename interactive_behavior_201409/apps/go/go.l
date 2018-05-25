#!/usr/bin/env roseus
;; go.l

(ros::roseus "run_go")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

;; TODO: implement app

(exit)
