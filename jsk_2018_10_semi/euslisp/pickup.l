(load "arm-movement-functions.l")
;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "fetch_driver_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
(defvar *bounding-box-topic* "/delivery_box_pickup/cluster_point_indices_decomposer_align_boxes_with_plane/boxes")
(defvar *box-address-topic* "/delivery_box_pickup/box_address")
(defvar *bounding-box-list* nil)
(defvar *bx-list* nil)
(defvar tmp nil)
(defvar *table* (make-cube 800 800 800))

;;(setq *co* (instance collision-object-publisher :init))
(setq *table* (make-cube 800 800 800))
;; ros::initする
(ros::roseus "pick-up-box")


;;*bounding-box-list*をsubscribeiして, 各boxの座標を計算する. 各座標は*bx-list*の中の, 各要素 :worldcoords になる?(
(send *fetch* :angle-vector (send *ri* :state :potentio-vector))
(defun detect-box-cb (msg)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (setq *bx-list* nil)
  ;; BoundingBoxがあれば表示する
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil)
    (mapcar #'(lambda (b)
                ;; BoundingBoxは、dimsntion(直方体の幅・奥行き・高さ)をもつ
                (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                       ;; (1) BoundingBoxのカメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *fetch* :head_camera_rgb_optical_frame_lk) :copy-worldcoords)))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの荷物を設置する
                  (send bx :newcoords (send cam-coords :transform cam->obj-coords))
                  (send bx :worldcoords)
		  (setq *bx-list* (append *bx-list* (list bx)))
                  (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
		  bx))
            *bounding-box-list*)
    (format t "# of box detecetd : ~A " (length *bx-list*))
    (send *irtviewer* :viewer :viewsurface :flush)
    )
  )

;;call back でsubscribeするbouding-box*に関する設定
(ros::subscribe *bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray #'detect-box-cb 1)


(defun reach-arm nil
  (if (= (length tmp) 1)
      (progn
	(print "picking up...")
	(send *co* :wipe-all)
	(setq table-coords (send (send (car tmp) :copy-worldcoords) :translate #f(0 0 -400)))
	(send *co* :add-object  *table* :frame-id "base_link" :relative-pose table-coords)
	(send *fetch* :reset-pose)
	(setq temae-coords (send (car tmp) :copy-worldcoords))
	(send temae-coords :rotate (/ pi -2) :y)
	(send temae-coords :translate #f(-300.0 0.0 0.0))
	(send *ri* :stop-grasp)
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics temae-coords)  10000)
	(send *ri* :wait-interpolation)
	(send temae-coords :translate #f(270.0 0.0 0.0))
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics temae-coords)  10000)
	(send *ri* :wait-interpolation)
	(send *ri* :start-grasp :wait t)
	)
    (print "more than 1 objects were detected")
    )
  )


;;荷物を取ることを試みる
(defun try-to-pickup nil
  (progn
    ;; (setq gripper-state (one-shot-subscribe "gripper_state" fetch_driver_msgs::GripperState))
    ;;gripper-state の状態をgripper-stateに入れる
    ;; (setq gripper-positon (elt (send gripper-state :joints) 0) :position)
    ;;gripper-position の状態をgripper-stateに入れる
    ;; (format t "gripper position :~A ~%" (send (elt b 0) :position))
    (reach-arm)
    )
  )

(setq address "red")
;
;;宛先に応じたカゴに荷物を入れ、カゴに対応するrosparamをincrementする
(defun store-box nil
  (let (address)
    (progn
      (print "cheking address and storing box ...")
      (check-address)
      ;; ;;addressはstringで"red", "green", "blue"で受け取る
      ;; (ros::subscribe *box-address-topic* std_msgs::string		      
      ;;                 #'(lambda (msg)
      ;; 			  (progn
      ;; 			    (ros::ros-info (format nil "box address ~A" (send msg :data)))
      ;; 			    (setq address (send msg :data)))))
      (setq address "red")
      (store-box-to-basket address)
      (ros::set-param (format nil "/box_number/~A" address) (+ (ros::get-param (format nil "/box_number/~A" address)) 1))
      (print "sotring box")
      )
    )
  )


;;荷物を全て拾い上げたことをpublishする
(defun publish-pickup-done nil
;;  (print "done")
  )



;;initialize
(setq pickup-status 1)

(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 (if *bx-list*
     (progn
       (setq tmp *bx-list*)
       (try-to-pickup)
       (if (= pickup-status 1)
	     (store-box)
	     )
       )
   (publish-pickup-done)
   )
   (ros::spin-once)
   (ros::sleep)
 )
