(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")

;;;
(ros::roseus "rect_client")

(defun speak-hello (&optional (tm 5000))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :neck-p :joint-angle 60)
  (objects (list *fetch*))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "こんにちは")
  (send *fetch* :head :neck-p :joint-angle -60)
  (objects (list *fetch*))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm))
  (send *ri* :wait-interpolation)

(defun cb (msg)
  (setq find-human nil)
  (dolist (rect (send msg :rects))
    (ros::ros-info "x:~3A y:~3A w:~3A h:~3A ~5A" (send rect :x) (send rect :y)  (send rect :width)  (send rect :height) (* (send rect :width)  (send rect :height)))
    (if (> (* (send rect :width)  (send rect :height)) 60000)
	(setq find-human t))
    )

  (when (and (> (length (send msg :rects)) 0) find-human)
    (ros::ros-error "~A human found" (length (send msg :rects)))
    (speak-hello))
  )

(ros::subscribe "/edgetpu_human_pose_estimator/output/rects" jsk_recognition_msgs::rectarray #'cb)

(ros::spin)
