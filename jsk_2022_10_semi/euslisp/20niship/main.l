(load "package://naoeus/nao-interface.l") ;; load modules
(objects (list *nao*))
(nao-init)

(ros::ros-info "AAAAAAA")
(unix:sleep 1)
(send *ri* :go-pos 0.5 0 0)
;;(send *ri* :angle-vector (send *nao* :angle-vector) 2000)
(unix:sleep 4)
(ros::ros-info "BBBB")

;; (objects (list *nao*)) 
;;(send *ni* :servo-on)
;;(dotimes (i 3)
;;  (send *nao* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
;;  (send *ri* :wait-interpolation)
;;)

;;現在のロボットの角度を取得
;; https://github.com/jsk-ros-pkg/jsk_robot/tree/acb5000cd23249b38ad69bb5df3ae593fabbb1d5/jsk_fetch_robot#common
;; (send *ri* :state :potentio-vector)

(send *ri* :stop-grasp)

