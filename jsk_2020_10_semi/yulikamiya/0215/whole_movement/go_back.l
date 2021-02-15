#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::roseus "go_back")
(ros::roseus-add-msgs "std_msgs")

(print "rosnode go_back init")

(defun go-back(msg)
    (print "func go_back started")
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :angle-vector #f(75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0))
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

    (send *ri* :wait-interpolation)
    (send *fetch* :torso :waist-z :joint-angle 20)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

    (send *ri* :wait-interpolation)
    (send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-door-front") :world)
    (send *fetch* :move-to (make-coords :pos (float-vector -600 -400 0)) :local)
    (send *fetch* :move-to (make-coords :rpy (float-vector 90 0 0)) :local)
    (send *irtviewer* :draw-objects)
    (send *ri* :move-to (send *fetch* :copy-worldcoords))
)

(ros::subscribe "go_back" std_msgs::string #'go-back)
(ros::spin)
(exit)