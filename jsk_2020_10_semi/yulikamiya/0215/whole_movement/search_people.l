#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::roseus "search_people")
(ros::roseus-add-msgs "std_msgs")
(ros::advertise "facial_recognition_start" std_msgs::string 1)

(print "rosnode search_people init")

(send *fetch* :head :neck-y :joint-angle 45)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :head :neck-y :joint-angle 0)
(send *fetch* :move-to (make-coords :pos #f(4700 7329 0) :rpy #f(1.6 1.614e-15 -6.387e-15)) :world)
(send *ri* :move-to (send *fetch* :copy-worldcoords))

(send *ri* :wait-interpolation)
(send *fetch* :torso :waist-z :joint-angle 180)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)


(send *ri* :wait-interpolation)
(send *fetch* :head :neck-p :joint-angle -20)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(setq msg (instance std_msgs::string :init))
(send msg :data "go")
(ros::publish "facial_recognition_start" msg)

(print "published")