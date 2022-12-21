#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(spot-init nil)
(setq *image-source* "hand_color")

(ros::subscribe "/edgetpu_object_detector/output/rects" jsk_recognition_msgs::RectArray #'cb)

(defun pick-object-in-image (image-source center-x center-y)
  (send *ri* :pick-object-in-image image-source center-x center-y))

(defun cb (msg)
  (let ((x (send (send msg :rects) :x))
        (y (send (send msg :rects) :y))
        (width (send (send msg :rects) :width))
        (height (send (send msg :rects) :height))
        (center-x (+ x (/ width 2)))
        (center-y (+ y (/ height 2))))
    (pick-object-in-image *image-source* center-x center-y)
    (unix::sleep 5)
    (send *ri* :stow-arm)
    ;;(send *ri* :wait-interpolation)                                                                
    ))
