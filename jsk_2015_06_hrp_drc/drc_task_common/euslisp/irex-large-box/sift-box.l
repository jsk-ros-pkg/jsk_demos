(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(load "package://drc_task_common/euslisp/primitive-marker-util.l")

(load "package://drc_task_common/euslisp/irex-large-box/wooden-box-model.l")
(make-wooden-box)
(setq *box-label-offset* (send (send *wooden-box* :sift-label-coords) :transformation (make-coords)))

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
        (unless (not *msg*)
          (ros::ros-info "recognized result is empty"))
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
          (set-primitive-marker-dimensions
           :x 598.0 :y 480.0 :z 710.0 :name "wooden_box"
           )
          ;; (set-primitive-marker-pose
          ;;  (send (send crds :copy-worldcoords) :translate (float-vector 0 0 355)) "ground" :name "wooden_box"
          ;;  )
          (setq *box-coords* (send crds :copy-worldcoords))
          )
(progn
      (warn "trans and crds are NONE !!!~%")
      nil
)
      )
    )
;;  t
  )

#|
(defun set-pose-10 ()
  (block
   :set-pose-loop
   (dotimes (i 10)
     (if (set-pose) (return-from :set-pose-loop nil)) ;; escape from :set-pose-loop block
;;     (unix::usleep 200000) ;; 0.2sec
     (unix::sleep 1) ;; 1sec
     )
   (ros::ros-info "recog failed 300 times")
   )
  )
|#

(defun set-pose-10 (&optional (average 5))
  (let (cds-lst)
    (block
     :set-pose-loop
     (dotimes (i (* average 10))
       ;;(if (set-pose) 
       ;;     (unix::usleep 200000) ;; 0.2sec
       (if (set-pose)
           (push (send *box-coords* :copy-worldcoords) cds-lst))
       (setq *msg* nil)
       (if (>= (length cds-lst) average)
           (return-from :set-pose-loop nil)) ;; escape from :set-pose-loop block
       (unix::usleep (* 1000 500))
       )
     (ros::ros-info "recog failed 300 times")
     )
    (when (>= (length cds-lst) average)
      (let ((ave-pos (float-vector 0 0 0))
            (ave-yaw 0))
        (dolist (cds cds-lst)
          (v+ (send cds :worldpos) ave-pos ave-pos)
          (incf ave-yaw (caar (send cds :rpy-angle))))
        (scale (/ 1.0 (length cds-lst)) ave-pos ave-pos)
        (setq ave-yaw (/ ave-yaw (length cds-lst)))
        ;; (pprint (list ave-pos ave-yaw cds-lst))
        (send *box-coords* :reset-coords)
        (send *box-coords* :transform
              (make-coords :pos ave-pos :angle ave-yaw :axis :z))
        )
      )
    ))
