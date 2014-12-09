#!/usr/bin/env roseus

(ros::roseus "spacenav_tf_publisher")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_pcl_ros")

(defun init-valve-tf
  ()
  (setq *src-tf-name* "odom_on_ground")
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *control-coords* (make-coords))
  (setq *selected-box-coords* (make-coords))
  (ros::subscribe "/spacenav/joy" sensor_msgs::Joy #'spacenav-callback)
  (ros::subscribe "bounding_box_marker/selected_box" jsk_pcl_ros::BoundingBox #'selected-box-cb)
  (ros::advertise "/valve_model_pose" geometry_msgs::PoseStamped 1)
  (ros::advertise "/set_valve_model_radius" std_msgs::Float32 1)
  )

(defun main-valve-tf
  ()
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (publish-tf :coords *control-coords*)
   (publish-pose :coords *control-coords*)
   (unless (ros::ok) (return)))
  )

(defun selected-box-cb
  (msg)
  (let* ((box-origin-frame (send (send msg :header) :frame_id))
         (box-pose (send msg :pose))
         (box-dmnsn (send msg :dimensions))
         (box-dmnsn-x (send box-dmnsn :x))
         (box-dmnsn-y (send box-dmnsn :y))
         (valve-radius (* (* (min box-dmnsn-x box-dmnsn-y) 1000) 0.5))
         )
    (setq *selected-box-coords* (ros::tf-pose->coords box-pose))
    (setq *selected-box-coords*
          (send (send *tfl* :lookup-transform *src-tf-name* box-origin-frame (instance ros::time :init))
                :transform *selected-box-coords* :local))
    (ros::publish "/set_valve_model_radius" (instance std_msgs::Float32 :init :data valve-radius))
    (setq *control-coords* *selected-box-coords*)
    ))

(defun spacenav-callback
  (msg)
  ;; (format t "~A~%" (send msg :axes))
  (let* ((bs (send msg :buttons))
         (rotscl 1e-2)
         (posscl 10.0))
    (when (equal (elt bs 0) 1)
      ;; (let* ((control-coords-ret
      ;;         (send *tfl* :lookup-transform "track_result" *src-tf-name* (instance ros::time :init))))
      ;;   (when control-coords-ret
      ;;     (setq *control-coords* control-coords-ret)
      ;;     (setq *control-coords*
      ;;           (send *control-coords* :transformation (make-coords) :local)))
      ;;   (return-from spacenav-callback nil nil))
      )
    (cond
     ((> (norm bs) 1e-3)
      (unix:system
       (format nil
               (concatenate
                string
                "echo \"(setq *control-coords* "
                "(make-coords :pos ~A :rot ~A))\""
                "> camera-coords.l")
               (send *control-coords* :worldpos)
               (send *control-coords* :worldrot))))
     ((< (norm (send msg :axes)) 1e-3))
     (t
      (setq *control-coords*
            (make-coords
             :pos
             (v+ (send *control-coords* :worldpos)
                 (scale posscl (subseq (send msg :axes) 0 3)))
             :rot
             (send
              (send (make-coords
                     :rot (copy-object (send *control-coords* :worldrot)))
                    :transform
                    (make-coords
                     :rpy (scale rotscl (reverse (subseq (send msg :axes) 3 6))))
                    :world)
              :worldrot))))
     )
    ))

(defun publish-tf
  (&key
   (coords (make-coords))
   (from *src-tf-name*)
   (to "OBJ"))
  (format t "[tf_tranfrom] ~A~%"
          (concatenate
           float-vector
           (scale 1e-3 (send coords :worldpos))
           (let ((v (matrix2quaternion (send coords :worldrot))))
             (concatenate float-vector (subseq v 3 4) (subseq v 0 3)))
           ))
  (send *tfb* :send-transform coords from to))

(defun publish-pose
  (&key
   (coords (make-coords)))

  (let* ((valve-pose (ros::coords->tf-pose-stamped coords *src-tf-name*)))
    (format t "[pose] ~A~%" valve-pose)
    (ros::publish "/valve_model_pose" valve-pose)))


(init-valve-tf)
(main-valve-tf)
