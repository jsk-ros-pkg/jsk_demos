(load "package://spoteus/spot-interface.l")

(spot-init)
;; (setq *spot* (instance spot-robot :init))
(objects (list *spot*))
(ros::ros-error "ok")

;; 初期姿勢
(send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 0 -170 160 0 10 90 0))
(send *ri* :angle-vector (send *spot* :angle-vector) 3000)
(send *ri* :gripper-close)

;; ===============================
;; 傘を差し出し済みかどうか
(setq *umbrella_replace_done* nil)

;; 前の人物の位置座標
(setq *pre_x* 100) 
(setq *pre_y* 100)
;; ===============================

;; 傘をセットする
(load "set_umbrella.l")

;; 散歩をする
;;(load "go_walking.l")

(ros::spin)
