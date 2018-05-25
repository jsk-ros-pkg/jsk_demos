#!/usr/bin/env roseus
;; find_object.l

(ros::roseus "run_find_object")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

;; TODO: implement app

(exit)
