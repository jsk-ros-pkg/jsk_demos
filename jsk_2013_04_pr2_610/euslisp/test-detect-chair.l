#!/usr/bin/env roseus
(ros::roseus "detect_chair_test")


(load "models/room610-chair-object.l")
(setq *chair-object* (instance room610-chair-object :init))
(load "detect-footobject.l")

(start-detect-footobject :width 0.37 :height 0.43 :model *chair-object* :error-thred 0.02 :inner-threshold 0.2  :base-origin "/eng8/6f/610" :name-space "chair-center")