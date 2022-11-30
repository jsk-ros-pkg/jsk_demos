#! usr/bin/env roseus

;; ファイルの読み込み
(load "package://spoteus/spot-interface.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "walking")

;; spotオブジェクトの作成
(spot-init)
;;(setq *spot* (instance spot-robot :init))
(objects (list *spot*))

(send *ri* :speak-jp "傘を持ちます")

;; 傘を持つ姿勢
(send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 90 -100 70 0 30 -90 -80))
(send *ri* :angle-vector (send *spot* :angle-vector) 8000)
(send *ri* :wait-interpolation)

;; 手の開閉、傘をセット
(send *ri* :gripper-open)
(send *ri* :wait-interpolation)
(send *ri* :speak-jp "傘をセットしてください")
(unix::sleep 3)
(send *ri* :gripper-close)
(send *ri* :wait-interpolation)

(send *ri* :speak-jp "お散歩に行きましょう")

(ros::ros-info "start subscribing")

(defun sub (msg)
  (setq boxes (send msg :boxes))
  (if boxes
    (progn
      ;; (ros::ros-info "subscribe msg [boxes: ~A]" boxes)
      
      ;; 1人目の情報を取得
      (setq pose (send (elt boxes 0) :pose))
      ;; (ros::ros-info "subscribe msg [pose: ~A]" pose)
      (setq x (send pose :position :x))
      (setq y (send pose :position :y))
      (setq z (send pose :position :z))
      (ros::ros-info "subscribe msg [x: ~A, y: ~A, z: ~A]" x y z)

      ;; 人間とspotとの距離感の調整
      (send *ri* :go-velocity #横方向 x-基準値#)

      ;; 人間とspotとの前後関係の調整
      (send *ri* :go-velocity #前後方向 y-基準値#))
    (ros::ros-info "no person")))

(ros::subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray #'sub)

(ros::spin)
(exit)
