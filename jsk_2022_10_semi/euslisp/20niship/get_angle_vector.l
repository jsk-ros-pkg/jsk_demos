(load "package://naoeus/nao-interface.l") ;; load modules
(nao-init)
(ros::ros-info "get angle vector")
;;現在のロボットの角度を取得
;; https://github.com/jsk-ros-pkg/jsk_robot/tree/acb5000cd23249b38ad69bb5df3ae593fabbb1d5/jsk_fetch_robot#common
(loop
  (setq angles (send *ri* :state :potentio-vector))
  (ros::ros-info "angles = ~A" angles )
  (objects (list *nao*))
  (unix:sleep 1)
)
