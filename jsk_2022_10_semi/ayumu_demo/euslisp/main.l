#! usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "umbrella")

(load "package://spoteus/spot-interface.l")

(spot-init)
;; (setq *spot* (instance spot-robot :init))
(objects (list *spot*))

;; 初期姿勢
(send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 0 -170 160 0 10 90 0))
(send *ri* :angle-vector (send *spot* :angle-vector) 3000)
(send *ri* :gripper-close)

;; ===============================
;; 傘を差し出し済みかどうか
(setq *umbrella_replace_done* nil)

;; 追従する人物の基準位置座標
(setq *ref_x* 100)
(setq *ref_y* 100)

(setq *counter* 0)
(setq *counter2* 0)

(setq *pre_position_number* 100)

(setq *sum_x* 0)
(setq *sum_y* 0)
;; ===============================

(defun sub (msg)
  (setq data (send msg :data))

  (setq position_number (elt data 0))
  (setq x (elt data 1))
  (setq y (elt data 2))
  (setq z (elt data 3))
  (setq dis (elt data 4))

  (ros::ros-info (format nil "x: ~A, y: ~A" x y))

  (if (not *umbrella_replace_done*)
      (load "set_umbrella.l")
    (load "go_walking.l")))

(ros::subscribe "/nearest_pos" std_msgs::float32multiarray #'sub)

(ros::rate 10)
(ros::spin)
