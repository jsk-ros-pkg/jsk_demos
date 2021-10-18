(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_2021_10_soup_from_boil/euslisp/model/my-eng2-scene.l")

(defun init ()
  ;; init
  (load "package://jsk_2021_10_soup_from_boil/euslisp/model/my-eng2-coords.l")
  
  ;; init ROS Node
  (ros::roseus "pr2-move-with-maps")
  
  ;; init 73B2 room
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  
  ;; init PR2
  (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (load "models/arrow-object.l")
  (setq *arrow* (arrow))
  
  (objects (list *scene* *pr2*))

  ;; set global variables
  (setq *s-time* 3)
  (setq *w-time* 2)
  (setq *debug-flag* nil)
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

;; (if (send *ri* :simulation-modep)
;;     (progn
;;       (update-view)
;;       )
;;   (progn
;;     (update-pos)
;;     (store-params)
;;     (change-dynm-params)
;;     )
;;   )

(warn "~% If you use real robot, run (update-pos) ~%")
(warn "please run (move-to-kitchen-ri) , (move-to-sink-ri) or (move-to-stove-ri) and (set-pose)~%")

;; add for voice recognition
(ros::load-ros-manifest "speech_recognition_msgs")
(setq *data-flag* nil)
(defun voice-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *data* (send *msg* :transcript))
    (setq *data-flag* t)
    (format t "data: ~A ~%" *data*)
    )
  )

(defun dialogue-func ()
  (ros::unsubscribe "/speech_to_text")
  (setq *data-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-cb)
  (until *data-flag*
         (ros::spin-once)
         (ros::sleep)
         )
  (send *ri* :speak-jp "ありがとうございます")
  (unix:sleep 1)
  )

(setq *ok-flag* nil)
(defun voice-ok-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *data* (send *msg* :transcript))
    (if (equal *data* '("OK"))
        (progn
          (format t "了解しました！ ~%" *data*)
          (setq *ok-flag* t)
          )
      (format t "data: ~A ~%" *data*)
      ) 
    )
  )

(defun ok-wait ()
  (ros::unsubscribe "/speech_to_text")
  (setq *ok-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-ok-cb)
  (until *ok-flag*
         (ros::spin-once)
         (ros::sleep)
         )  
  (send *ri* :speak-jp "ありがとうございます")
  (unix:sleep *w-time*)
  )

(defun ok-grasp-larm (&rest args &key (gain 0.01) &allow-other-keys)
  (ok-wait)
  (send *ri* :start-grasp :larm :wait t :gain gain)
  )

(defun ok-grasp (&rest args &key (gain 0.01) &allow-other-keys)
  (ok-wait)
  (send *ri* :start-grasp :rarm :wait t :gain gain)
  )

(defun receive-with-dialogue-larm (&rest args &key (gain 0.01) &allow-other-keys)
  ;; (send *ri* :speak-jp "包丁を渡す準備をして，合図をして下さい") ;; 喋らせている想定でスタート
  (setq *yes-flag* nil)
  (until *yes-flag*
         (send *ri* :stop-grasp :larm :wait t)
         (ok-grasp-larm :gain gain)
         (send *ri* :speak-jp "上手く掴めていますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           (progn
             (send *ri* :speak-jp "グリッパを開きます.もう一度お願いします")
             (unix:sleep *w-time*)
             )
           )
         )
  
  (send *ri* :speak-jp "次の行動に移ります")
  (unix:sleep *w-time*)
  )

(defun receive-with-dialogue (&rest args &key (gain 0.01) &allow-other-keys)
  ;; (send *ri* :speak-jp "包丁を渡す準備をして，合図をして下さい") ;; 喋らせている想定でスタート
  (setq *yes-flag* nil)
  (until *yes-flag*
         (send *ri* :stop-grasp :rarm :wait t)
         (ok-grasp :gain gain)
         (send *ri* :speak-jp "上手く掴めていますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           (progn
             (send *ri* :speak-jp "グリッパを開きます.もう一度お願いします")
             (unix:sleep *w-time*)
             )
           )
         )
  
  (send *ri* :speak-jp "次の行動に移ります")
  (unix:sleep *w-time*)
  )

;; add for grasp detection
(defun grasp-fail-detection (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (setq gripper-len (send *ri* :state :gripper :rarm :position))
  (if (< gripper-len g-thre)
      (setq *fail-flag* t))
  )

(defun grasp-fail-detection-larm (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (setq gripper-len (send *ri* :state :gripper :larm :position))
  (if (< gripper-len g-thre)
      (setq *fail-flag* t))
  )

;; release gripper
(defun release-object (&rest args &key (g-num 0.025) &allow-other-keys)
  (let (tmp-gripper)
    ;;グリッパを微妙に開く
    (setq tmp-gripper (* 0.001 (send *ri* :state :gripper :rarm :position)))
    (send *ri* :move-gripper :rarm (+ g-num tmp-gripper))
    ))

;; wait until movable
(ros::load-ros-manifest "pr2_msgs")

(defun wait-cable ()
  (setq *cable* t)
  (while *cable*
    ;; (let ((msg (cable-test)))
    ;;   (setq *cable* (< 0 (send msg :AC_present)))
    ;;   (unix::sleep 1)))
    (let ((msg (one-shot-subscribe "/power_state" pr2_msgs::PowerState)))
      (setq *cable* (< 0 (send msg :AC_present)))
      (unix::sleep 1)))
  )

(defun wait-cable-in ()
  (setq *cable* t)
  (while *cable*
    (let ((msg (one-shot-subscribe "/power_state" pr2_msgs::PowerState)))
      (setq *cable* (>= 0 (send msg :AC_present)))
      (unix::sleep 1)))
  )

(defun cable-test ()
  (let ((msg (one-shot-subscribe "/power_state" pr2_msgs::PowerState)))
    (format t "now AC_present is ~A ~%" (send msg :AC_present))
    msg))

(defun speech-before-move-with-rec ()
  (setq *msg* (cable-test))
  (if (< 0 (send *msg* :AC_present))
      (progn
        (send *ri* :speak-jp "移動するので電源ケーブルを抜いて下さい")
        (wait-cable)
        (send *ri* :speak-jp "ありがとうございます．移動を開始します")
        )
    (send *ri* :speak-jp "移動を開始します")
    )
  )

(defun speech-after-move-with-rec ()
  (setq *msg* (cable-test))
  (if (< 0 (send *msg* :AC_present))
      (send *ri* :speak-jp "調理を再開します")
    (progn
      (send *ri* :speak-jp "電源ケーブルを挿して下さい")
      (wait-cable-in)
      (send *ri* :speak-jp "ありがとうございます．調理を再開します")
      ))
  )


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
  ;; (send *ri* :move-to (make-coords :pos #f(1200 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (setq local-flag (send *ri* :move-to (make-coords :pos #f(1400 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map"))
  (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (warn "go-pos unsafe !! ~%")
        )
    (send *ri* :speak-jp "移動に失敗しました，アシストして下さい")
    )
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-sink-ri ()
  ;; コンロまで移動する
  (send *ri* :clear-costmap)
  ;; (send *ri* :move-to (make-coords :pos #f(1110 9100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (setq local-flag (send *ri* :move-to (make-coords :pos #f(1400 9100 351) :rpy (float-vector pi 0 0)) :frame-id "/map"))
  (if (eql local-flag t)
      (progn   (send *ri* :go-pos-unsafe 0.29 0 0)
               )
    (send *ri* :speak-jp "移動に失敗しました，アシストして下さい")
    )
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-stove-ri ()
  (send *ri* :clear-costmap)
  ;; (send *ri* :move-to (make-coords :pos #f(1200 8100 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (setq local-flag (send *ri* :move-to (make-coords :pos #f(1400 8100 351) :rpy (float-vector pi 0 0)) :frame-id "/map"))
  (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (warn "go-pos unsafe !! ~%")
        )
    (send *ri* :speak-jp "移動に失敗しました，アシストして下さい")
    )
  (print (send *ri* :state :worldcoords))
  )

(defun move-to-microwave-ri ()
  ;; 電子レンジまで移動する
  (send *ri* :clear-costmap)
  ;; (send *ri* :move-to (make-coords :pos #f(1700 9400 351) :rpy (float-vector 0 0 0)) :frame-id "/map")
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
  ;; (send *ri* :move-to (make-coords :pos #f(1200 8200 351) :rpy (float-vector pi 0 0)) :frame-id "/map")
  (setq local-flag (send *ri* :move-to (make-coords :pos #f(1400 8200 351) :rpy (float-vector pi 0 0)) :frame-id "/map"))
  (if (eql local-flag t)
      (progn
        (send *ri* :go-pos-unsafe 0.2 0 0)
        (warn "go-pos unsafe !! ~%")
        )
    (send *ri* :speak-jp "移動に失敗しました，アシストして下さい")
    )
  (print (send *ri* :state :worldcoords))
  )


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


(defun go-to-microwave () ;; TODO
  ;; kitchenからmicrowaveに移動する
  
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

;; move with speech
(defun speech-before-move ()
  (send *ri* :speak-jp "移動するので電源ケーブルを抜いて，合図して下さい")
  (ros::unsubscribe "/speech_to_text")
  (setq *ok-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-ok-cb)
  (until *ok-flag*
	 (ros::spin-once)
	 (ros::sleep)
	 )

  (send *ri* :speak-jp "ありがとうございます．移動を開始します")
  )

(defun speech-after-move ()
  (send *ri* :speak-jp "電源ケーブルを挿して，合図して下さい")
  
  (ros::unsubscribe "/speech_to_text")
  (setq *ok-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-ok-cb)
  (until *ok-flag*
	 (ros::spin-once)
	 (ros::sleep)
	 )  
  (send *ri* :speak-jp "ありがとうございます．調理を再開します")
  )


(defun go-to-stove-with-speech-before ()
  ;; kitchen spot(拠点)からstoveに移動する
  (speech-before-move)
  (unix:sleep *w-time*)
  (go-to-stove)
  )

(defun go-to-stove-with-speech-after ()
  ;; kitchen spotからstove(拠点)に移動する
  (go-to-stove)
  (speech-after-move)
  (unix:sleep *w-time*)
  )

(defun go-to-stove-with-speech-before-and-after ()
  ;; kitchen spotからstoveに移動し，拠点も移動させる  
  (speech-before-move)
  (unix:sleep *w-time*)
  (go-to-stove)
  (speech-after-move)
  (unix:sleep *w-time*)
  )

(defun go-to-kitchen-from-stove-with-speech-after ()
  ;; stoveからkitchen spot(拠点)に移動する
  (go-to-kitchen-from-stove)
  (speech-after-move)
  (unix:sleep *w-time*)
  )

(defun go-to-kitchen-from-stove-with-speech-before ()
  ;; stove(拠点)からkitchen spotに移動する
  (speech-before-move)
  (unix:sleep *w-time*)
  (go-to-kitchen-from-stove)
  )


(defun go-to-sink-with-speech-before ()
  ;; kitchen spot(拠点)からsinkに移動する
  (speech-before-move)
  (unix:sleep *w-time*)
  (go-to-sink)  
  )

(defun go-to-kitchen-from-sink-with-speech-after ()
  ;; sinkからkitchen spot(拠点)に移動する
  (go-to-kitchen-from-sink)
  (speech-after-move)
  (unix:sleep *w-time*)
  )

;; TODO stove→microwave， microwave→arrnage



;; move-to call codes
(defun move-to-kitchen-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-kitchen-ri-direct)
  (after-move)
  )

(defun move-to-kitchen-with-speech-after ()
  (move-to-kitchen-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-shelf-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-shelf-ri-direct)
  (after-move)
  )

(defun move-to-shelf-with-speech-after ()
  (move-to-shelf-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-sink-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-sink-ri-direct)
  (after-move)
  )

(defun move-to-sink-with-speech-after ()
  (move-to-sink-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-stove-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-stove-ri-direct)
  (after-move)
  )

(defun move-to-stove-with-speech-after ()
  (move-to-stove-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-stove-with-speech-before-and-after ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-stove-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )


(defun move-to-microwave-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-microwave-ri-direct)
  (after-move)
  )

(defun move-to-microwave-with-speech-after ()
  (move-to-microwave-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-arrange-with-speech-before ()
  (speech-before-move)
  (unix:sleep *w-time*)
  (move-to-arrange-ri-direct)
  (after-move)
  )

(defun move-to-arrange-with-speech-after ()
  (move-to-arrange-ri-direct)
  (speech-after-move)
  (unix:sleep *w-time*)
  (after-move)
  )

;; speech-with-rec
(defun move-to-kitchen-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-kitchen-ri-direct)
  (after-move)
  )

(defun move-to-kitchen-with-speech-after-with-rec ()
  (move-to-kitchen-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-shelf-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-shelf-ri-direct)
  (after-move)
  )

(defun move-to-shelf-with-speech-after-with-rec ()
  (move-to-shelf-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-sink-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-sink-ri-direct)
  (after-move)
  )

(defun move-to-sink-with-speech-after-with-rec ()
  (move-to-sink-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-stove-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-stove-ri-direct)
  (after-move)
  )

(defun move-to-stove-with-speech-after-with-rec ()
  (move-to-stove-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-stove-with-speech-before-and-after-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-stove-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )


(defun move-to-microwave-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-microwave-ri-direct)
  (after-move)
  )

(defun move-to-microwave-with-speech-after-with-rec ()
  (move-to-microwave-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-arrange-with-speech-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-arrange-ri-direct)
  (after-move)
  )

(defun move-to-arrange-with-speech-after-with-rec ()
  (move-to-arrange-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

(defun move-to-arrange-with-speech-before-and-before-with-rec ()
  (speech-before-move-with-rec)
  (unix:sleep *w-time*)
  (move-to-arrange-ri-direct)
  (speech-after-move-with-rec)
  (unix:sleep *w-time*)
  (after-move)
  )

