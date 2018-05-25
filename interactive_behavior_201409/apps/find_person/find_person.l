#!/usr/bin/env roseus
;; find_person.l

(ros::roseus "run_find_person")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

;; TODO: implement app

(exit)
