#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "person recognition")

(defclass human-recog-cb-class
  :super propertied-object
  :slots (label_name rect human_x human_y width_human human_height))
(defmethod human-recog-cb-class
  (:init ()
   (ros::subscribe "/spot_recognition/class"
                   jsk_recognition_msgs::ClassificationResult #'send self :human-recog-class-cb)
   (ros::subscribe "/spot_recognition/rects"
                   jsk_recognition_msgs::RectArray #'send self :human-rect-cb))

  (:human-recog-class-cb (msg)
                         (setq label_name (send msg :label_names))
                         (print (list 'cb self label_name)))
  (:human-rect-cb (msg) (setq rect (send msg :rects)))
  (:print-loop () ;; for debug
         (ros::rate 10)
         (while (ros::ok)
           (if rect
               (progn
                 (setq human_x (send (elt rect 0) :x))
                 (setq human_y (send (elt rect 0) :y))
                 (setq human_width (send (elt rect 0) :width))
                 (setq human_height (send (elt rect 0) :height))
                 (ros::ros-info "(x,y)=(~A,~A), area: ~A" human_x human_y (* human_width human_height))))
           (ros::spin-once)
           (ros::sleep))
         (exit)))
(setq human-recog (instance human-recog-cb-class :init))

;; (do-until-key
;;   (ros::spin-once))
