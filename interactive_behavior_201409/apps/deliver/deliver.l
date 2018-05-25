#!/usr/bin/env roseus
;; deliver.l

(ros::roseus "run_deliver")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

;; TODO: implement app

(exit)
