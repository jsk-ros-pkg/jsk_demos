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
  ;; (send *irtviewer* :look-all *pr2*)
  (send *irtviewer* :viewer :viewing :look #f(3121.66 3235.13 33071.2) #f(842.417 8604.7 30832.0) #f(0 0 1))
  (send *viewer* :draw-objects)
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
(warn "please run (move-real-robot) and (set-pose)~%")

;; add for grasp detection
(defun grasp-fail-detection (arm &rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (if (send *ri* :simulation-modep)
    (setq *fail-flag* nil)
    (if (< (send *ri* :state :gripper arm :position) g-thre)
      (setq *fail-flag* t))))

;; release gripper
(defun release-object (&rest args &key (g-num 0.025) &allow-other-keys)
  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm
        (+ g-num (* 0.001 (send *ri* :state :gripper :rarm :position)))))

;; IRT上の移動
(defun move-on-irt (spot)
  "
Arg spot candidates:
:kitchen, :shelf, :sink, :stove, :microwave, :seasoning, :arrange, :garbage
"
  (let (spot-coords)
    (cond
     ((eq spot :kitchen) (setq spot-coords *kitchen-spot*))
     ((eq spot :shelf) (setq spot-coords *shelf-spot*))
     ((eq spot :sink) (setq spot-coords *sink-spot*))
     ((eq spot :stove) (setq spot-coords *stove-spot*))
     ((eq spot :microwave) (setq spot-coords *microwave-spot*))
     ((eq spot :seasoning) (setq spot-coords *seasoning-spot*))
     ((eq spot :arrange) (setq spot-coords *arrange-spot*))
     ((eq spot :garbage) (setq spot-coords *garbage-spot*))
     )
    (send *pr2* :move-to (send spot-coords :copy-worldcoords) :world))
  (update-view))

;; 実機の移動
(defun move-real-robot (&key (from nil) (to nil))
    "
Arg to candidates:
:kitchen, :sink, :stove, :microwave, :seasoning, :arrange
"
  (set-pose)
  (send *ri* :speak-jp "移動します。")
  (send *ri* :clear-costmap)
  (let (target-coords go-pos-unsafe-after go-pos-unsafe-before move-to-succeeded)
    (cond
     ((eq to :kitchen)
      (setq target-coords (make-coords :pos #f(1400 8500 351) :rpy (float-vector pi 0 0)))
      (setq go-pos-unsafe-after (list 0.35 0 0)))
     ((eq to :sink)
      (setq target-coords (make-coords :pos #f(1400 9100 351) :rpy (float-vector pi 0 0)))
      (setq go-pos-unsafe-after (list 0.29 0 0)))
     ((eq to :stove)
      (setq target-coords (make-coords :pos #f(1400 8100 351) :rpy (float-vector pi 0 0)))
      (setq go-pos-unsafe-after (list 0.2 0 0)))
     ((eq to :microwave)
      (setq target-coords (make-coords :pos #f(1600 9300 351) :rpy (float-vector 0 0 0)))
      (setq go-pos-unsafe-after (list 0 0 0)))
     ((eq to :seasoning)
      (setq target-coords (make-coords :pos #f(1700 8900 351) :rpy (float-vector 0 0 0)))
      (setq go-pos-unsafe-after (list 0 0 0)))
     ((eq to :arrange)
      (setq target-coords (make-coords :pos #f(1400 8200 351) :rpy (float-vector pi 0 0)))
      (setq go-pos-unsafe-after (list 0.2 0 0)))
     )
    ;; If arg from is given,
    ;; PR2 :go-pos-unsafe to the opposite direction of go-pos-unsafe-after before move-to
    (when from
      (setq go-pos-unsafe-before (mapcar #'(lambda (x) (* -1 x)) go-pos-unsafe-after))
      (send* *ri* :go-pos-unsafe go-pos-unsafe-before))
    ;; Move to
    ;; In simulation mode, move *ri* to z=30000
    (if (send *ri* :simulation-modep)
      (send target-coords :translate (float-vector 0 0 (- 30000 (elt (send target-coords :worldpos) 2)))))
    (setq move-to-succeeded (send *ri* :move-to target-coords :frame-id "/map"))
    ;; :go-pos-unsafe after move-to
    (if (eql move-to-succeeded t)
      (progn
        (send* *ri* :go-pos-unsafe go-pos-unsafe-after)
        (warn "go-pos unsafeを利用しています。注意してください。~%"))
      (send *ri* :speak-jp "移動に失敗しました，アシストして下さい" :wait t)))
  ;; Update position and IRT Viewer
  (print (format nil "Move to ~A~%" (send *ri* :state :worldcoords)))
  (update-pos)
  (update-view))

(defun move-real-robot-safe (&key (to nil))
  "
Arg to candidates:
:kitchen, :shelf, :sink, :stove, :microwave, :seasoning, :arrange, :garbage
"
  (send *ri* :speak-jp "移動します。")
  (send *ri* :clear-costmap)
  (let (target-coords)
    (cond
     ((eq to :kitchen) (setq target-coords (make-coords :pos #f(1200 8500 351) :rpy (float-vector pi 0 0))))
     ((eq to :shelf) (setq target-coords (make-coords :pos #f(1110 8500 351) :rpy (float-vector pi 0 0))))
     ((eq to :sink) (setq target-coords (make-coords :pos #f(1110 9100 351) :rpy (float-vector pi 0 0))))
     ((eq to :stove) (setq target-coords (make-coords :pos #f(1200 8100 351) :rpy (float-vector pi 0 0))))
     ((eq to :microwave) (setq target-coords (make-coords :pos #f(1600 9300 351) :rpy (float-vector 0 0 0))))
     ((eq to :seasoning) (setq target-coords (make-coords :pos #f(1700 8900 351) :rpy (float-vector 0 0 0))))
     ((eq to :arrange) (setq target-coords (make-coords :pos #f(1200 8200 351) :rpy (float-vector pi 0 0))))
     ((eq to :garbage) (setq target-coords (make-coords :pos #f(1700 9000 351) :rpy (float-vector 0 0 0))))
     )
    (send *ri* :move-to target-coords :frame-id "/map"))
  (print (format nil "Move to ~A~%" (send *ri* :state :worldcoords)))
  (update-pos)
  (update-view))

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
  (speech-before-move-with-rec)
  (move-real-robot :to :arrange)
  (speech-after-move-with-rec)
  (after-move)
  )

;; ケーブルの状態を確認し、キッチンに移動する。
(defun move-to-kitchen-with-cable-check ()
  (speech-before-move-with-rec)
  (move-real-robot :to :kitchen)
  (speech-after-move-with-rec)
  (after-move)
  )
