(ros::roseus "get_apriltag_corners")
(ros::load-ros-manifest "apriltag_ros")

(setq *tag-detection* nil)

(defun tag-detection-callback (msg)
  (setq *tag-detection* msg))


(ros::subscribe "/camera/color/tag_detections"
                apriltag_ros::AprilTagDetectionArray
                #'tag-detection-callback)
(ros::rate 10)

(do-until-key
    (ros::spin-once)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (dotimes (i (length (send *tag-detection* :detections)))
      (setq tag
            (ros::tf-pose->coords (send (send (send (elt (send *tag-detection* :detections) i) :pose) :pose) :pose)))
      (print tag)
      ))
  (ros::sleep))
