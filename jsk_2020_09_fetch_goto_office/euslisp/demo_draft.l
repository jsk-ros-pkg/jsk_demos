(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_maps/src/change-floor.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(fetch-init)

;; subscribing
(ros::load-ros-manifest "speech_recognition_msgs")
(defvar *julius_msg* "/speech_to_text_julius")
(defvar *julius_service* "speech_recognition")

(defun register-julius-words()
    (ros::wait-for-service *julius_service*)
    (setq req (instance speech_recognition_msgs::SpeechRecognitionRequest :init))
    (send (send req :vocabulary) :words (list "さんがい" "にんげん" "けんくどう" "たんたんめん" "けいおかだ" "すすんで" "ふぇっち" "でこーど" "ぴーあーるーつー"))
)

(defun listen-julius()
    (setq res (ros::service-call "speech_recognition" req t))
    (ros::ros-info (format nil "~A :: ~A" (send (send res :result) :transcript) (send (send res :result) :confidence)))
    (car (send (send res :result) :transcript))
    )

(defun wait-for-julius-trigger(word)
    (while t 
        (if (equal word (listen-julius))
            (return t)
            (unix:sleep 1))
        ))

(defun go-to-7f-elevator()
    (undock)
    (send *ri* :move-to (make-coords :pos #f(1850 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータの下ボタンを押してください" :wait t)
    )
    
(defun get-on-elevator()
    ;; (push-elevator-button())
    ;; (confirm-opened-door())
    (send *ri* :clear-costmap)
    (send *ri* :go-pos 3.0 0 180)
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータに乗りました" :wait t)
    )

(defun get-off-elevator()
    (change-floor "/eng2/3f" :base-frame-name "/base_link")
    (send *ri* :speak-jp "マップを切り替えました" :wait t)
    ;; (recognize-arrive-at-eng2-3f())
    (send *ri* :clear-costmap)
    ;; re-create 3f elevator map
    (send *ri* :go-pos 3.5 0 0)
    (unix:sleep 1)
    (send *ri* :speak-jp "エレベータから降りました" :wait t)
    )

(defun go-to-eng2-3f()
    (register-julius-words)
    (go-to-7f-elevator)
    (wait-for-julius-trigger "すすんで")
    (get-on-elevator)
    (wait-for-julius-trigger "すすんで")
    (get-off-elevator)
    )

;; (defun go-to-front-of-auto-door1()
;;     (send *ri* :move-to (make-coords :pos #f(50000 -31276 0) :rpy (float-vector 0 0 0)) :frame-id "/map")
;;     )

;; (defun go-to-front-of-auto-door2())
    


;; jimusitu kyoin
;; (send *ri* :move-to (make-coords :pos #f(50000 -31276 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; jimusitu gakusei
;; (send *ri* :move-to (make-coords :pos #f(50000 -26576 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; elevator hall 7f
;; (send *ri* :move-to (make-coords :pos #f(1850 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")

;; elevator hall 3f
;; (send *ri* :move-to (make-coords :pos #f(737 -32134 -0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")

;; auto-door1
;; (send *ri* :move-to (make-coords :pos #f(9474 -29243 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; auto-door2
;; (send *ri* :move-to (make-coords :pos #f(19527 -34152 0) :rpy (float-vector 0 0 0)) :frame-id "/map")

;; auto-door3
;; (send *ri* :move-to (make-coords :pos #f(21527 -34152 0) :rpy (float-vector pi 0 0)) :frame-id "/map")

;; auto-door4
;; (send *ri* :move-to (make-coords :pos #f(11474 -29243 0) :rpy (float-vector pi 0 0)) :frame-id "/map")
