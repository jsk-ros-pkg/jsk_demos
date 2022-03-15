(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_2021_10_soup_from_boil/euslisp/model/my-eng2-scene.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/my-eng2-coords.l")
(load "models/arrow-object.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")

(defun init ()
  ;; init

  ;; init ROS Node
  (ros::roseus "pr2-move-with-maps")

  ;; init 73B2 room
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

  ;; init PR2
  (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (setq *arrow* (arrow))

  (objects (list *scene* *pr2*))

  ;; set global variables
  )

;; add dynamic parmas
;; c.f https://github.com/knorth55/jsk_demos/blob/5b2a3112cdefb1d53ae04ea0e06d345684386f21/elevator_move_base_pr2/src/utils.l#L1-L145
(defun call-service (service req)
  (if (ros::wait-for-service service 0)
      (ros::service-call service req)
    nil))

(defun change-dynm-params (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.10)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.087)  ;; 5deg
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding" :double 0.03)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding" :double 0.03)
  t)

(defun store-params (&rest args)
  (defparameter *xy-goal-tolerance*
    (ros::get-param "/move_base_node/DWAPlannerROS/xy_goal_tolerance"))
  (defparameter *yaw-goal-tolerance*
    (ros::get-param "/move_base_node/DWAPlannerROS/yaw_goal_tolerance"))
  (defparameter *global-footprint-padding*
    (ros::get-param "/move_base_node/global_costmap/footprint_padding"))
  (defparameter *local-footprint-padding*
    (ros::get-param "/move_base_node/local_costmap/footprint_padding"))
  (defparameter *look-at-human-enabled*
    (send (one-shot-subscribe "look_at_human/enabled" std_msgs::Bool) :data))
  (call-service "look_at_human/stop" (instance std_srvs::EmptyRequest :init))
  t)

(defun restore-params (&rest args)
  (when (boundp '*xy-goal-tolerance*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/DWAPlannerROS" "xy_goal_tolerance"
     :double *xy-goal-tolerance*))
  (when (boundp '*yaw-goal-tolerance*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance"
     :double *yaw-goal-tolerance*))
  (when (boundp '*global-footprint-padding*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/global_costmap" "footprint_padding"
     :double *global-footprint-padding*))
  (when (boundp '*local-footprint-padding*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/local_costmap" "footprint_padding"
     :double *local-footprint-padding*))
  (when (and (boundp '*look-at-human-enabled*) *look-at-human-enabled*)
    (call-service "look_at_human/start" (instance std_srvs::EmptyRequest :init)))
  t)

(defun set-defalut-params (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance"
   :double 0.2)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance"
   :double 0.17)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding"
   :double 0.1)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding"
   :double 0.1)
  t)


;; def utils
(defun update-view ()
  (send *pr2* :copy-worldcoords)
  (send *viewer* :draw-objects)
  (send *irtviewer* :look-all *pr2*)
  )

(defun update-scene ()
  (objects (list *scene* *pr2*))
  (send *irtviewer* :look-all *pr2*)
  )

(defun update-pos ()
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(init)
(update-view)

(warn "~% If you use real robot, run (update-pos) ~%")
(warn "please run (move-to-kitchen-ri) , (move-to-sink-ri) or (move-to-stove-ri) and (set-pose)~%")

;; add for grasp detection
(defun grasp-fail-detection (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (if (< (send *ri* :state :gripper :rarm :position) g-thre)
      (setq *fail-flag* t)))

(defun grasp-fail-detection-larm (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (if (< (send *ri* :state :gripper :larm :position) g-thre)
      (setq *fail-flag* t)))

;; release gripper
(defun release-object (&rest args &key (g-num 0.025) &allow-other-keys)
  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm
        (+ g-num (* 0.001 (send *ri* :state :gripper :rarm :position)))))

;; IRT上の移動
(defun move-to-kitchen-irt ()
  (send *pr2* :move-to (send *kitchen-spot* :copy-worldcoords) :world)
  (update-view)
  )

(defun move-to-shelf-irt ()
  (send *pr2* :move-to (send *shelf-spot* :copy-worldcoords) :world)
  (update-view)
  )

(defun move-to-sink-irt ()
  (send *pr2* :move-to (send *sink-spot* :copy-worldcoords) :world)
  (update-view)
  )
(defun move-to-stove-irt ()
  (send *pr2* :move-to (send *stove-spot* :copy-worldcoords) :world)
  (update-view)
  )
(defun move-to-microwave-irt ()
  (send *pr2* :move-to (send *microwave-spot* :copy-worldcoords) :world)
  (update-view)
  )
(defun move-to-seasoning-irt ()
  (send *pr2* :move-to (send *seasoning-spot* :copy-worldcoords) :world)
  (update-view)
  )
(defun move-to-arrange-irt ()
  (send *pr2* :move-to (send *arrange-spot* :copy-worldcoords) :world)
  (update-view)
  )
(defun move-to-garbage-irt ()
  (send *pr2* :move-to (send *garbage-spot* :copy-worldcoords) :world)
  (update-view)
  )

;; 実機の移動
(defun move-to-kitchen-ri ()
  (send *ri* :clear-costmap)
  (let ((local-flag (send *ri* :move-to (make-coords :pos #f(1400 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map")))
    (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.35 0 0)
        (warn "go-pos unsafe !! ~%"))
      (send *ri* :speak-jp "移動に失敗しました，アシストして下さい"))
    (print (send *ri* :state :worldcoords))))

(defun move-to-sink-ri ()
  ;; コンロまで移動する
  (send *ri* :clear-costmap)
  (let ((local-flag (send *ri* :move-to (make-coords :pos #f(1400 9100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")))
    (if (eql local-flag t)
      (send *ri* :go-pos-unsafe 0.29 0 0)
      (send *ri* :speak-jp "移動に失敗しました，アシストして下さい"))
    (print (send *ri* :state :worldcoords))))

(defun move-to-stove-ri ()
  (send *ri* :clear-costmap)
  (let ((local-flag (send *ri* :move-to (make-coords :pos #f(1400 8100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")))
    (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (warn "go-pos unsafe !! ~%"))
      (send *ri* :speak-jp "移動に失敗しました，アシストして下さい"))
    (print (send *ri* :state :worldcoords))))

(defun move-to-microwave-ri ()
  ;; 電子レンジまで移動する
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1600 9300 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-seasoning-ri ()
  ;; 調味料まで移動する
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1700 8900 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-arrange-ri ()
  (send *ri* :clear-costmap)
  (let ((local-flag (send *ri* :move-to (make-coords :pos #f(1400 8200 351) :rpy (float-vector pi 0 0)) :frame-id "/map")))
    (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (warn "go-pos unsafe !! ~%"))
      (send *ri* :speak-jp "移動に失敗しました，アシストして下さい"))
    (print (send *ri* :state :worldcoords))))

;; direct move funcs
(defun move-to-kitchen-ri-direct ()
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1200 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-shelf-ri-direct ()
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1110 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-sink-ri-direct ()
  ;; コンロまで移動する
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1110 9100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-stove-ri-direct ()
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1200 8100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-microwave-ri-direct ()
  ;; 電子レンジまで移動する
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1600 9300 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-seasoning-ri-direct ()
  ;; 調味料まで移動する
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1700 8900 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-arrange-ri-direct ()
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1200 8200 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-garbage-ri-direct ()
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1700 9000 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
  (print (send *ri* :state :worldcoords))
  )

;; kitchen spotとsink間の移動
(defun go-to-sink ()
  ;; kitchen spotからsinkに移動する
  (send *ri* :go-pos-unsafe 0 -0.6 0)
  ;; (send *ri* :go-pos-unsafe 0.09 0 0)
  (move-to-sink-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-kitchen-from-sink ()
  ;; sinkからkitchen spotに移動する
  (send *ri* :go-pos-unsafe -0.09 0 0)
  (send *ri* :go-pos-unsafe 0 0.6 0)
  (move-to-kitchen-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-kitchen-from-sink-without-map ()
  ;; sinkからkitchen spotに移動する
  (send *ri* :go-pos-unsafe -0.09 0 0)
  (send *ri* :go-pos-unsafe 0 0.6 0)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-stove ()
  ;; kitchen spotからstoveに移動する
  (send *ri* :go-pos-unsafe 0 0.4 0)
  (move-to-stove-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-kitchen-from-stove ()
  ;; stoveからkitchen spotに移動する
  (send *ri* :go-pos-unsafe 0 -0.4 0)
  (move-to-kitchen-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-sink-from-arrange ()
  ;; arrange spotからsinkに実行する
  (send *ri* :go-pos-unsafe 0 -0.9 0)
  (send *ri* :go-pos-unsafe 0.09 0 0)
  (move-to-sink-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-arrnage-from-sink ()
  ;; sinkからarrange spotに移動する
  (send *ri* :go-pos-unsafe -0.09 0 0)
  (send *ri* :go-pos-unsafe 0 0.9 0)
  (move-to-arrange-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-sink-from-stove ()
  ;; stoveからsinkに移動する ;; TODO 確認
  (send *ri* :go-pos-unsafe 0 -0.4 0)
  (send *ri* :go-pos-unsafe 0 -0.4 0)
  (move-to-sink-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

(defun go-to-stove-from-sink ()
  ;; sinkからstoveに移動する
  (send *ri* :go-pos-unsafe 0 0.4 0)
  (send *ri* :go-pos-unsafe 0 0.4 0)
  (move-to-stove-ri)
  (update-pos)
  (update-view)
  (print (send *ri* :state :worldcoords))
  )

;; def util-funcs
(defun set-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle 325)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun pre-ik-pose ()
  ;; set-pose without ri
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle 325)
  )

(defun set-pose-rec ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle 325)
  (send *pr2* :head_tilt_joint :joint-angle 0)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun set-head ()
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *pr2* :head :neck-y :joint-angle 0.0)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun after-move ()
  (send *pr2* :torso_lift_joint :joint-angle 325)
  (set-head)
  )

;; Motions with interaction
;; ケーブルの状態を確認し、キッチンに移動する。
(defun move-to-arrange-with-cable-check ()
  (send *ri* :speak-jp "キッチンに移動します。")
  (speech-before-move-with-rec)
  (unix:sleep 2)
  (move-to-arrange-ri)
  (speech-after-move-with-rec)
  (unix:sleep 2)
  (after-move)
  (update-pos)
  )

;; ケーブルの状態を確認し、キッチンに移動する。
(defun move-to-kitchen-with-cable-check ()
  (send *ri* :speak-jp "キッチンに移動します。")
  (speech-before-move-with-rec)
  (unix:sleep 2)
  (move-to-kitchen-ri)
  (speech-after-move-with-rec)
  (unix:sleep 2)
  (after-move)
  (update-pos)
  )
