(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(load "package://drc_task_common/euslisp/primitive-marker-util.l")

(ros::roseus "koyama_primitive_marker")
(ros::roseus-add-msgs "posedetection_msgs")


(defun sub-object (msg)
  (setq *msg* msg)
  )


(ros::subscribe "/ObjectDetectionTrassPose"  geometry_msgs::PoseStamped #'sub-object)
(defun set-pose-10 ()
  (block
   :set-pose-loop
   (dotimes (i 10)
     (if (set-pose) (return-from :set-pose-loop nil))
     (unix::usleep 200000)
     )
   (ros::ros-info "recog failed 10 times")
   )
  )

(defun make-interactive-marker (&key (name "trass") (x 265) (y 1750) (z 265) (offset (make-coords)))
  (let (trans coords)
    (ros::spin-once)
    (if (or (not *msg*) (not (send *msg* :pose)))
        (progn
          (if (not *msg*)
              (ros::ros-info "msg not published")
            (ros::ros-info "recognized result is empty")
            )
          ;;(return-from set-pose nil)
          ))
    (setq coords (make-coords :pos (float-vector 300 0 (/ z 2))));;tekito-
    (when *msg*
      (setq trans (send *tf* :lookup-transform "ground"
                        (send *msg* :header :frame_id) (send *msg* :header :stamp)))
      (unless trans
        (setq trans (send *tf* :lookup-transform "ground"
                          (send *msg* :header :frame_id) (ros::time 0))))
      (if trans (setq coords (send trans :transform (ros::tf-pose->coords (send *msg* :pose)))))
      (send coords :transform offset))
    (insert-primitive-marker
     :name name
     )
    (set-primitive-marker-dimensions
     :x x :y y :z z :name name
     )
    (set-primitive-marker-pose
     coords "ground" :name name
     )
    (ros::ros-info "interactive maker has made!")
    )
  t
  )
(setq *msg* nil)
(setq *tf* (instance ros::transform-listener :init))

