(load "package://naoeus/nao-interface.l") ;; load modules
;; (objects (list *nao*))
(nao-init)

;;(send *ri* :stop-grasp)

(defun update (angle)
  (ros::ros-info "get angle!")
  (send *nao* :angle-vector angle)
  (send *ri* :angle-vector (send *nao* :angle-vector) 1500)
  )

(ros::ros-info "start")
;; (init-node ("listener" :spin 10)
  (subscribe "nextnaoangle" "std_msgs/Float32MultiArray" #'update)
;;)
(ros::ros-info "end")
