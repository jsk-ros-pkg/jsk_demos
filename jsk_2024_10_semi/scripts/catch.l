(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
(ros::load-ros-manifest "jsk_recognition_msgs")

;;pr2生成
(if (not (boundp '*pr2*)) (pr2-init))
(setq *ri* (instance pr2-interface :init))
;; 一辺600mmの立方体を出現させる
(setq *cube* (make-cube 600 600 600))
;; 立方体を(700, 0, 300)移動
(send *cube* :translate (float-vector 700 0 300))
;;boxを適当な初期位置で配置
(setq *target-box* (make-cube 20 20 20))
(objects (list *pr2* *target-box* *cube*))

;; chabudaiフラグ
(setq *chabudai* nil)
;; ボールを回収した回数
(setq *ball-count* 0)

(setq *tfl* (instance ros::transform-listener :init))
;;検知したboxとラベルを対応させるクラス
(defclass box-label-synchronizer
  :super exact-time-message-filter)

(defmethod box-label-synchronizer
  (:callback (box-msg label-msg)
    (print (list box-msg label-msg))
    (print (send-all (list box-msg label-msg) :header :stamp))
    (box-cb box-msg label-msg)
  )
)

;;コールバック関数
;;/docker/detic_segmentor/detected_classesというトピックにidの対応あり
(defun box-cb (box-msg label-msg)
  (ros::ros-info "received ~A boxes, ~A labels" (length (send box-msg :boxes)) (length (send label-msg :labels)))
  (dolist (msg-conbined (map cons #'(lambda (x y) (list  x y)) (send box-msg :boxes) (send label-msg :labels)))
    (let (box label)
      (setq box (car msg-conbined) label (cadr msg-conbined))
      (ros::ros-info "box ~A, label ~A" (send box :pose) (send label :name))
      (when (or (string= (send label :name) "ball") (string= (send label :name) "soccer_ball") (string= (send label :name) "tennis_ball") (string= (send label :name) "beachball") (string= (send label :name) "ping-pong_ball"))
        (setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
        (setq *target-dimensions* (send box :dimensions))
        (ros::ros-info "coords ~A, dimension ~A" (send *target-coords* :worldcoords) (* (send *target-dimensions* :x) (send *target-dimensions* :y) (send *target-dimensions* :z)))
        (send *target-box* :move-to *target-coords* :world)
        (ros::ros-info "update target position")
        ; )
      )
    )
  )
)

(defun chabudai-cb (msg)
    (ros::ros-info "robot_mood: ~A" (send msg :data))
    (ros::ros-info "ball_count: ~A" *ball-count*)
    (when (> (send msg :data) (- 9 *ball-count*))
        (setq *chabudai* t)
    )
)

(defun init-pose ()
  ; (send *ri* :speak-jp "初期姿勢に移動します" :wait t)
  (send *pr2* :reset-pose)
  (send *pr2* :torso :waist-z :joint-angle 150)
  (send *pr2* :larm :shoulder-p :joint-angle 0)
  (send *pr2* :larm :shoulder-r :joint-angle 90)
  (send *pr2* :head :neck-p :joint-angle 60)
  (send *ri* :stop-grasp :arms)
  (send *irtviewer* :draw-objects) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
)

(defun chabudai-gaeshi ()
    (send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 700 0 300))
       :parent *cube*)
    )

    (send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z
    )

    (send *pr2* :reset-pose)
    (send *pr2* :torso :waist-z :joint-angle 150)
    (send *pr2* :larm :collar-y :joint-angle 100)
    (send *pr2* :rarm :collar-y :joint-angle -100)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *pr2* :larm :shoulder-p :joint-angle 74)
    (send *pr2* :larm :elbow-p :joint-angle -50)
    (send *pr2* :rarm :shoulder-p :joint-angle 74)
    (send *pr2* :rarm :elbow-p :joint-angle -50)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ; (send *pr2* :larm :shoulder-p :joint-angle 70)
    (send *pr2* :larm :shoulder-r :joint-angle 90)
    (send *pr2* :larm :collar-y :joint-angle 60)
    (send *pr2* :larm :elbow-p :joint-angle -110)
    (send *pr2* :larm :elbow-r :joint-angle 180)
    (send *pr2* :larm :wrist-p :joint-angle -25)
    (send *pr2* :larm :wrist-r :joint-angle -15)
    ; (send *pr2* :rarm :shoulder-p :joint-angle 70)
    (send *pr2* :rarm :shoulder-r :joint-angle -90)
    (send *pr2* :rarm :collar-y :joint-angle -60)
    (send *pr2* :rarm :elbow-p :joint-angle -110)
    (send *pr2* :rarm :elbow-r :joint-angle 180)
    (send *pr2* :rarm :wrist-p :joint-angle -25)
    (send *pr2* :rarm :wrist-r :joint-angle 15)
    (send *pr2* :head :neck-p :joint-angle 50)
    (send *ri* :stop-grasp :arms)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ; (send *pr2*
    ;     :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
    ;                                     :translate (float-vector 0.0 0.5 0.0))
    ;     :move-target (send *cube* :get :left-coords)
    ;     :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
    ;     :rotation-axis t)
    ; (send *pr2*
    ;     :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
    ;                                     :translate (float-vector 1.0 -0.5 0.0))
    ;     :move-target (send *cube* :get :left-coords)
    ;     :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
    ;     :rotation-axis t)

    ; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    ; (send *ri* :wait-interpolation)
    ; (send *irtviewer* :draw-objects)

    ; (send *pr2*
    ;     :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
    ;                                     :translate (float-vector 0.0 -5.0 5.0))
    ;     :move-target (send *cube* :get :left-coords)
    ;     :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
    ;     :rotation-axis t)
    ; (send *pr2*
    ;     :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
    ;                                     :translate (float-vector 0.0 5.0 5.0))
    ;     :move-target (send *cube* :get :left-coords)
    ;     :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
    ;     :rotation-axis t)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ;; sleep 1秒
    (unix:sleep 1)

    (send *pr2* :larm :shoulder-p :joint-angle -20)
    (send *pr2* :larm :shoulder-r :joint-angle 20)
    (send *pr2* :larm :elbow-p :joint-angle -100)
    (send *pr2* :rarm :shoulder-p :joint-angle -20)
    (send *pr2* :rarm :shoulder-r :joint-angle -20)
    (send *pr2* :rarm :elbow-p :joint-angle -100)
    (send *pr2* :head :neck-p :joint-angle -10)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
)

(defun pick-up ()
  (setq is-pickup-successful nil)

  (while (not is-pickup-successful)
    (send *target-box* :newcoords (make-coords))
    ;;サブスクライブ
    ; (ros::subscribe "/pointcloud_screenpoint/output_point" geometry_msgs::PointStamped #'callback)
    (setq box-sync (instance box-label-synchronizer :init
          (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
          (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))

    ;;ボールが見つかるまでループ
    ; (send *ri* :speak-jp "ボールをさがしています" :wait t)
    (ros::rate 3) ;; 3Hz
    (while (< (elt (send *target-box* :worldpos) 2) 400)
      (ros::ros-info "waiting... ~A" (send *target-box* :worldpos))
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (ros::spin-once)
      (ros::sleep)
    )
    ; (send *ri* :speak-jp "ボールを見つけました" :wait t)
    (ros::ros-info "start grasping... ~A" (send *target-box* :worldpos))
    (send *pr2* :larm :shoulder-p :joint-angle 0)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;;ボールの上空へ移動
    (setq arm :larm)

    (send *pr2* arm :elbow-p :joint-angle -100)
    (send *pr2* arm :collar-y :joint-angle 20)

    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *pr2* arm :inverse-kinematics
      (send (send (send *target-box* :copy-worldcoords) :translate #f(30 0 150)) :rotate (deg2rad 90) :y)
      :rotation-axis t
    )

    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    ;;ボールへ移動
    (send *pr2* arm :inverse-kinematics
      (send (send (send *target-box* :copy-worldcoords) :translate #f(30 0 -20)) :rotate (deg2rad 90) :y)
      :rotation-axis t
    )

    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)


    ;;掴む
    (send *ri* :start-grasp arm :wait t)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    (setq is-pickup-successful (> (send *ri* :robot arm :gripper :joint-angle) 5))
    (when (not is-pickup-successful)
      (ros::ros-info "pickup failed ~%")

      ; (send *ri* :speak-jp "ボールを掴めませんでした" :wait t)
      (send *pr2* arm :inverse-kinematics
        (send (send (send *target-box* :copy-worldcoords) :translate #f(30 0 100)) :rotate (deg2rad 90) :y)
        :rotation-axis t
      )
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      (init-pose)
    )
  )

  (send *pr2* arm :inverse-kinematics
    (send (send (send *target-box* :copy-worldcoords) :translate #f(30 0 100)) :rotate (deg2rad 90) :y)
    :rotation-axis t
  )

  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (ros::ros-info "pickup successful ~%")
  ; (send *ri* :speak-jp "ボールを持ち上げました" :wait t)

  ;; 
  (send *pr2* :larm :collar-y :joint-angle 50)
  (send *pr2* :larm :shoulder-p :joint-angle 0)
  (send *pr2* :larm :shoulder-r :joint-angle 90)
  (send *pr2* :larm :elbow-p :joint-angle -30)
  (send *pr2* :larm :elbow-r :joint-angle 90)
  (send *pr2* :larm :wrist-p :joint-angle -10)
  (send *pr2* :larm :wrist-r :joint-angle 0)

  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;;離す
  (send *ri* :stop-grasp :arms)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
)


; (init-pose)
; (unix:sleep 1)
; (chabudai-gaeshi)
;;メイン処理の実行
(ros::advertise "ball_count" std_msgs::Int32 1)
(ros::subscribe "/robot_mood" std_msgs::Int32 #'chabudai-cb)

(send *ri* :speak-jp "ボールを片付けます" :wait t)
(while (not *chabudai*)
  (x::window-main-one)
  (ros::ros-info "init-pose")
  (init-pose)
  (unix:sleep 1)
  (ros::ros-info "pick up")
  (pick-up)
  
  (setq *ball-count* (+ *ball-count* 1))
  (setq msg (instance std_msgs::Int32 :init))
  (send msg :data *ball-count*)
  (ros::ros-info "ball_count: ~A" *ball-count*)
  (ros::publish "ball_count" msg)
  (ros::spin-once)

  (when *chabudai*
    (chabudai-gaeshi)
  )
)