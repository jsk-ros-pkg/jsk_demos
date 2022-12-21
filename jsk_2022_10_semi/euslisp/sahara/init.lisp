(load "package://spoteus/spot-interface.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(if t ;実機につなぐなら
    (spot-init)
    (setq *spot* (instance spot-robot :init))
)
;(send *ri* :speak-jp "こんにちは")
(send *ri* :speak "initialized")

(objects (list *spot*))