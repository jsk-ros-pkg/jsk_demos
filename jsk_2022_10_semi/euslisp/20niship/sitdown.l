(load "package://naoeus/nao-interface.l") ;; load modules
(load "angle_settings.l") 
(nao-init)
;;現在のロボットの角度を取得
;; https://github.com/jsk-ros-pkg/jsk_robot/tree/acb5000cd23249b38ad69bb5df3ae593fabbb1d5/jsk_fetch_robot#common

(objects (list *nao*))

(setq TIMES (list 5000 5000 5000 5000 5000 5000))
(dotimes (i (length ANGLES))
  (setq angle (elt ANGLES i))
  (setq time (elt TIMES i))
  (ros::ros-info "set angle[~A] = ~A for ~A" i angle time)
  (send *nao* :angle-vector angle)
  (send *ri* :angle-vector (send *nao* :angle-vector) time)

  (objects (list *nao*))
  (unix:sleep 5)
)
