#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::roseus "measure_temperature")
(ros::roseus-add-msgs "geometry_msgs")

(print "rosnode measure_temperature init")

(defun measure-temperature(msg)    
    (print "func measure-temperature started")
    (send *ri* :speak (format nil "You look like ~A, is that right?" (send msg :header :frame_id)))

    (send *fetch* :rarm :angle-vector #f(0.950514 78.0375 -175.645 128.973 -0.084058 55.6689 94.4493))
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

    (send *ri* :wait-interpolation)
    (send *fetch* :head :neck-p :joint-angle 0)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

    (send *ri* :wait-interpolation)
    (send *fetch* :start-grasp)
    (send *irtviewer* :draw-objects)
    (send *ri* :start-grasp :wait t)

    (send *ri* :wait-interpolation)
    (send *fetch* :stop-grasp)
    (send *irtviewer* :draw-objects)
    (send *ri* :stop-grasp :wait t)
)

(ros::subscribe "matched_face" geometry_msgs::vector3stamped #'measure-temperature)
(ros::spin)
(exit)