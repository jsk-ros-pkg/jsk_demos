(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(load "package://drc_task_common/euslisp/primitive-marker-util.l")

(load "package://drc_task_common/euslisp/irex-large-box/apc-box-model.l")
(make-apc-box)
(setq *box-label-offset* (send (send *apc-box* :sift-label-coords) :transformation (make-coords)))

(ros::roseus "set_pose_recognition")
(setq *tf* (instance ros::transform-listener :init))
(ros::roseus-add-msgs "posedetection_msgs")

(setq *msg* nil)
(defun sub-object (msg)
  (setq *msg* msg)
  )

(ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'sub-object)

(defun set-pose ()
  (ros::spin-once)
  (if (or (not *msg*) (not (send *msg* :objects)))
      (progn
        (if (not *msg*)
            (ros::ros-info "msg not published")
          (ros::ros-info "recognized result is empty")
          )
        (return-from set-pose nil)
        )
    )
  (let ((trans (send *tf* :lookup-transform "ground"
                     (send *msg* :header :frame_id) (send *msg* :header :stamp)))
        crds)
    (unless trans
      (setq trans (send *tf* :lookup-transform "ground"
                        (send *msg* :header :frame_id) (ros::time 0))))
    (if trans
        (progn
          (setq crds (send trans :transform (ros::tf-pose->coords (send (elt (send *msg* :objects) 0) :pose))))
          ;; modify sift marker direction
          (send crds :rotate (deg2rad 180) :z)
          ;; modify label position offset for box model
          (send crds :transform *box-label-offset*)
          (insert-primitive-marker
           :name "apc_box"
           )
          (set-primitive-marker-dimensions
           :x 450.0 :y 545.0 :z 975.0 :name "apc_box"
           )
          (set-primitive-marker-pose
           crds "ground" :name "apc_box"
           )
          )
(progn
      (warn "trans and crds are NONE !!!~%")
      nil
)
      )
    )
;;  t
  )

(defun set-pose-10 ()
  (block
   :set-pose-loop
   (dotimes (i 300)
     (if (set-pose) (return-from :set-pose-loop nil)) ;; escape from :set-pose-loop block
;;     (unix::usleep 200000) ;; 0.2sec
     (unix::sleep 1) ;; 1sec
     )
   (ros::ros-info "recog failed 300 times")
   )
  )
