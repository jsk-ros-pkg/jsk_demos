;(setq pose (send *ri* :potentio-vector))

;初期姿勢
(send *ri* :speak "start")
(send *ri* :gripper-open)
(send *spot* :reset-pose)
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :gripper-open)

;手を地面付近に下ろす
(defun hand-down
    (height pitch)
    (send *spot* :reset-pose)
    (setq endcoords (send (send *spot* :arm :end-coords) :copy-worldcoords)) ;手先の座標系を取得し、コピーする
    (objects (list endcoords *spot*))   ;座標系を表示

    (send endcoords :rotate pitch :z)  ;座標系の回転移動
    (send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)

    (send endcoords :translate (float-vector 300 0 0) :world)  ;座標系の平行移動
    (send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)

    (send endcoords :translate (float-vector 0 0 height) :world)
    (send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)
)
(hand-down -100 (- (/ pi 2)))
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;; (exit)
(setq *moving* nil)

(defun cb (msg ball)
    (unless *moving*
        (setq *moving* t)
        (let (ballrects ballrect center-x center-y)
            (ros::ros-info ball)
            (send *ri* :speak ball)
            ;; (setq ballrects (one-shot-subscribe "/find_ball/ball_rect" jsk_recognition_msgs::RectArray))
            (setq ballrects msg)
            (setq ballrect (elt (send ballrects :rects) 0))
            (setq center-x (+ (send ballrect :x) (/ (send ballrect :width) 2)) )
            (setq center-y (+ (send ballrect :y) (/ (send ballrect :height) 2)) )
            (send *ri* :pick-object-in-image "hand_color" center-x center-y)
            (send *ri* :pick-object-in-image-wait-for-result)

            ;; (send *ri* :gripper-close)
            ;; (send *ri* :stow-arm)

            (cond 
                (
                    (string= ball "yellow")
                    (hand-down -300 (- (/ pi 3)))
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                    (hand-down -500 (- (/ pi 2)))
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                    (send *ri* :gripper-open)

                    (hand-down -100 (- (/ pi 2)))
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                )
                (
                    t
                    (send *spot* :reset-pose)
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                    (send *ri* :go-pos 0 0 -30)
                    (hand-down -100 0)
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                    (send *ri* :gripper-open)
                    (unix:sleep 5)
                    (hand-down -100 (- (/ pi 2)))
                    (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
                    (send *ri* :wait-interpolation)
                )
            )
            
            ;; (exit)
            (unix:sleep 5)
        )
        (setq *moving* nil)
    )
)
(defun cb_lined
    (msg)
    (cb msg "lined")
)
(defun cb_yellow
    (msg)
    (cb msg "yellow")
)
(defun cb_white
    (msg)
    (cb msg "white")
)
(ros::subscribe "/find_ball/lined_ball_rect" jsk_recognition_msgs::RectArray #'cb_lined)
(ros::subscribe "/find_ball/yellow_ball_rect" jsk_recognition_msgs::RectArray #'cb_yellow)
(ros::subscribe "/find_ball/white_ball_rect" jsk_recognition_msgs::RectArray #'cb_white)


(ros::rate 1)
(do-until-key
    (ros::spin-once)
    (ros::sleep)
)
;; (ros::spin)

;; ;手を対象物に持っていく
;; (setq endcoords (send (send *spot* :arm :end-coords) :copy-worldcoords)) ;手先の座標系を取得し、コピーする
;; (objects (list endcoords *spot*))   ;座標系を表示

;; (send endcoords :translate #f(0 0 -100) :world)
;; (send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis nil :revert-if-fail nil)

;; (send *ri* :wait-interpolation)
;; (send *ri* :angle-vector (send *spot* :angle-vector) 2000)

;; ;もとに戻る
;; (send *ri* :wait-interpolation)
;; (send *ri* :gripper-close)

;; (send *spot* :reset-pose)

;; (send *ri* :wait-interpolation)
;; (send *ri* :angle-vector (send *spot* :angle-vector) 2000)
