(load "package://jsk_perception/euslisp/detection-interface.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "fridge_marker")

(defun hoge ((timeout 10) ((:type atype) nil) (tf-force nil)
                             (diff-position 10.0)
                             (diff-rotation 1) ;; radian
                             (single nil) (speak t)
                             (publish-objectdetection-marker t)
                             (marker-life 30) (speak-name)
                             (detection-topic *detection-topic*)
                             ((:target-object target-obj) (make-cube 60 60 60)))

  (start-detection-interface-subscribe tf-force target-obj detection-topic)
  (while (ros::ok)
    (marker-publish target-obj marker-life))
)

