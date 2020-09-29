(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_maps/src/change-floor.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(fetch-init)
(ros::load-ros-manifest "speech_recognition_msgs")

(defun go-to-7f-elevator()
    (undock)
    (send *ri* :move-to (make-coords :pos #f(1850 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータのボタンを押してください" :wait t)
    )
    
(defun get-on-elevator()
    ;; (push-elevator-button())
    ;; (confirm-opened-door())
    (send *ri* :go-pos 3.0 0 180)
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータに乗りました" :wait t)
    )

(defun get-off-elevator()
    ;; (recognize-arrive-at-eng2-3f())
    (change-floor "/eng2/3f" :base-frame-name "/base_link")
    (send *ri* :speak-jp "マップを切り替えました" :wait t)
    (send *ri* :clear-costmap)
    ;; re-create 3f elevator map
    (send *ri* :go-pos 3.5 0 0)
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータから降りました" :wait t)
    )

(defun go-to-eng2-3f()
    (go-to-7f-elevator)
    (get-on-elevator)
    (get-off-elevator)
    )

;; jimusitu kyoin
;; (send *ri* :move-to (make-coords :pos #f(50000 -31276 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; jimusitu gakusei
;; (send *ri* :move-to (make-coords :pos #f(50000 -26576 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; elevator hall
;; (send *ri* :move-to (make-coords :pos #f(1850 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")
