(defun ih-init ()
  ;; init
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/soup-from-boil/tool-use-arrange-codes.l")
  
  ;; set objects
  (load "package://jsk_2020_04_pr2_curry/euslisp/model/cooking-pot-73b2.l")
  (setq *pot* (cooking-pot))
  (setq *pot-coords* (make-coords :pos (float-vector 450 8030 30850) :rpy (float-vector 3.142 0 0))) ;; arrange original
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))
  
  
  ;; def global coords
  (setq *pot-larm-from-above* (make-coords :pos (float-vector 639 8109 30971) :rpy (float-vector -3.069 1.306 -0.087)))
  (setq *pot-larm-from-below* (make-coords :pos (float-vector 644 8104 30947) :rpy (float-vector 3.125 -0.909 3.119)))
  
  
  (setq *ih-panel-coords* (make-coords :pos (float-vector 563 7940 30865) :rpy (float-vector  3.044 1.407 2.87)))

  ;; arrange original
  (setq *ih-knob-coords* (make-coords :pos (float-vector 680 8110 30715) :rpy (float-vector -3.047 -0.011 1.512)))
  
  (move-to-arrange-irt)
  (objects (list *pr2* *scene* *spatula* *ladle* *pot*))
  (update-view)
  )

(ih-init)


(defun set-pose-3 ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )

(warn "If you use real robot, run (update-pos) ~%")


;; add for sense
(setq *x-thre* -15)

(ros::roseus "stove_rec_test" :anonymous t)
(setq *topic-name* "/left_endeffector/wrench")
(ros::load-ros-manifest "geometry_msgs")
(setq *cb-flag* nil)
(defun wrench-x-cb (msg)
  (when msg
    (let ((x nil))
      (setq x (send msg :wrench :force :x))
      (if (< x *x-thre*)
          (progn 
            (setq *cb-flag* t)
            (format t "OK!! x: ~A ~%" x)
            )
        (format t "x: ~A ~%" x))
      )))

;; add for panel rec
(ros::load-ros-package "roseus")

(defun panel-reader-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *data* (send *msg* :data))
    (format t "~A ~%" *data*)
    (if (ros::time> (send *msg* :header :stamp) *now*)
        (setq *cb-flag* t)
      )
    ))

(defun read-panel ()
  (setq *now* (ros::time-now))
  (ros::unsubscribe "/ih_stove_reader/result")
  (setq *cb-flag* nil)
  (ros::subscribe "/ih_stove_reader/result" roseus::StringStamped #'panel-reader-cb)
  (until *cb-flag*
         (ros::spin-once)
         (ros::sleep)
         )
  )

(defun string2num (string)
  (let (num)
    (cond
     ((equal string "1")
      (setq num 1))
     ((equal string "2")
      (setq num 2))
     ((equal string "3")
      (setq num 3))
     ((equal string "4")
      (setq num 4))
     ((equal string "5")
      (setq num 5))
     ((equal string "6")
      (setq num 6))
     ((equal string "7")
      (setq num 7))
     ((equal string "8")
      (setq num 8))
     ((equal string "9")
      (setq num 9))
     )
    num))

(defun read-data ()
  (let (num-data num)
    (setq num-data (string-trim "p" (string-trim "/" *data*)))
    (setq num (string2num num-data))
    num))


;; ;; add for rosservice
;; (ros::load-ros-manifest "std_srvs")
;; (ros::wait-for-service "after_stow_data_collection/save_request")
;; (setq req (instance std_srvs::TriggerRequest :init))


;; (defun rossrv-test ()
;;   ;; (ros::service-call "after_stow_data_collection_kinect/save_request" req t)
;;   (setq res (ros::service-call "after_stow_data_collection/save_request" req t))
;;   )

;; (defun rossrv-test-double ()
;;   (ros::service-call "after_stow_data_collection_kinect/save_request" req t)
;;   (setq res (ros::service-call "after_stow_data_collection/save_request" req t))
;;   )

(defun look-at-arrow ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send *arrow* :copy-worldcoords) :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )


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

(defun receive-with-dialogue-larm (&rest args &key (gain 0.01) &allow-other-keys)
  ;; (send *ri* :speak-jp "包丁を渡す準備をして，okと合図をして下さい") ;; 喋らせている想定でスタート
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

;; add for grasp detection
(defun grasp-fail-detection-larm (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (setq gripper-len (send *ri* :state :gripper :larm :position))
  (if (< gripper-len g-thre)
      (setq *fail-flag* t))
  )


;; def util-funcs
(defun reset-rarm ()
  ;; (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  ;; (send *pr2* :rarm :angle-vector *rav0*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8428.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  ;; (setq *rav2* #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  ;; (send *pr2* :rarm :angle-vector *rav2*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 800.186 8289.105 30866.677) :rpy (float-vector -2.341 -0.222 -0.482)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq *rav3* #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun reset-larm ()
  ;; 左手を少し上に
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 少し上にして水平に
  ;; (setq *lav1* (float-vector 16.6498 -20.1996 123.812 -51.9785 -42.6397 -65.2851 -0.059116))
  ;; (send *pr2* :larm :angle-vector *lav1*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 510.0 8090.0 31190.0) :rpy (float-vector 1.571 0.0 1.349)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 手前にする
  ;; (setq *lav2* (float-vector 41.8129 -20.2467 85.9536 -82.3235 -13.8272 -47.9804 -0.0))
  ;; (send *pr2* :larm :angle-vector *lav2*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 660.0 7990.0 31290.0) :rpy (float-vector 1.571 0.0 1.722)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; (setq *lav3* (float-vector 75.8217 52.0712 108.49 -109.796 -2.86542 -9.47056 -0.0))
  ;; (send *pr2* :larm :angle-vector *lav3*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 800.191 7910.896 30866.677) :rpy (float-vector 2.341 -0.222 0.133)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)  
  
  (setq *lav4* (float-vector 75.0 50.0 110.0 -110.0 -20.0 -10.0 -10.0))
  (send *pr2* :larm :angle-vector *lav4*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)  
  )

(defun reset-larm-3 ()
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )


;; IH funcs 
(defun push-stove-button ()
  ;; (send *arrow* :translate #f(130 0 0) :world) ;;をしている前提で始める
  
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 15)
    (send *pr2* :larm :move-end-pos #f(-10 0 0) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  (send *ri* :angle-vector-sequence avs tms)
  
  (ros::unsubscribe "/left_endeffector/wrench")
  (setq *topic-name* "/left_endeffector/wrench")
  (setq *cb-flag* nil)
  (ros::subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped #'wrench-x-cb)
  
  (while (some #'identity (send *ri* :wait-interpolation nil 0.1))
    (when (progn
            (ros::spin-once)
            (if *cb-flag* t)
            )
      (send *ri* :cancel-angle-vector)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (warn "cancel!!! ~%")
      )
    )
  
  (ros::unsubscribe "/left_endeffector/wrench")

  )

(defun on-stove ()
  (send *ri* :start-grasp :larm :wait t)

  ;; スイッチの位置
  (send *arrow* :newcoords (send *ih-switch-on-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (push-stove-button)

  (send *arrow* :newcoords (send *ih-switch-on-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun off-stove ()
  (send *ri* :start-grasp :larm :wait t)

  (send *arrow* :newcoords (send *ih-switch-off-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (send *arrow* :newcoords (send *ih-switch-off-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun push-knob ()
  (send *ri* :start-grasp :larm :wait t)
  
  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun grasp-knob ()
  (send *ri* :move-gripper :larm 0.08)
  
  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :translate #f(100 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(10 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (send *ri* :start-grasp :larm :wait t :gain 100)
  )

(defun rotate-knob (&rest args &key (deg 30) &allow-other-keys)
  ;; rotate
  (setq avs (list))
  (setq tms (list))

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  
  (if (> deg 0)
      (progn 
        (dotimes (i (/ deg 10))
          (send *arrow* :rotate (deg2rad 10) :x)
          (send *pr2* :larm :inverse-kinematics
                (send *arrow* :copy-worldcoords)
                :rotation-axis t
                :debug-view nil)
          (setq av (send *pr2* :angle-vector))
          (setq avs (append avs (list av)))
          (setq tms (append tms (list 200)))
          ))
    (dotimes (i (/ (* deg -1) 10))
      (send *arrow* :rotate (deg2rad -10) :x)
      (send *pr2* :larm :inverse-kinematics
            (send *arrow* :copy-worldcoords)
            :rotation-axis t
            :debug-view nil)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 200)))
      )
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  )


(defun rotate-knob-with-rec (&rest args &key (target "/7p") &allow-other-keys)
  (setq *target-flag* nil)
  (until *target-flag*
         (read-panel)
         (setq *read-data* (read-data))
         (cond           
          ;; TODO TODO
          ;; もし読めていなかったら
          ((equal *read-data* "")
           (send *ri* :speak-jp "パネルの認識に失敗しました，もう一度認識します") ;; fail countを入れても良いかも．
           )
          ;; もしtargetと一致していたら
          ((equal *read-data* target)
           (setq *target-flag* t))
          ;; もしtargetよりも小さかったら
          ((< *read-data* target)
           )
          ;; もしtargetよりも大きかったら
          ((> *read-data* target)
           )
          )
         ))


(defun release-knob ()
  (send *ri* :stop-grasp :larm :wait t)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(30 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :translate #f(100 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  )


;; pot funcs
(defun grasp-pot-from-above ()
  ;;少し上に
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 250) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis nil
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;;少し前に
  (send *arrow* :newcoords (make-coords :pos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :worldpos)))
  (send *arrow* :translate #f(-200 50 50) :world)
  (send *arrow* :rotate (deg2rad 90) :z :world)
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :rotate (deg2rad -60) :x :world)  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; potを上から掴む座標
  (send *arrow* :newcoords (send *pot-larm-from-above* :copy-worldcoords))

  (send *arrow* :translate #f(-100 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate #f(100 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)
  )

(defun grasp-pot-from-below ()
  ;;腕を引いた姿勢
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  
  ;; potを下から掴む座標
  (send *arrow* :newcoords (send *pot-larm-from-below* :copy-worldcoords))

  (send *arrow* :translate #f(60 0 -60) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate #f(-60 0 60) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)
  )

(defun grasp-pot-from-below-with-fail-detection ()
  (grasp-pot-from-below)
  
  ;; 失敗を検知する
  (grasp-fail-detection-larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-pot-from-below)
        (grasp-fail-detection-larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "鍋を掴むのが失敗しました，鍋の柄を掴ませる準備をしてokと合図して下さい")
              (ok-grasp-larm)
              ))
        ))
  )


(defun release-pot-from-below ()
  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :newcoords (send *pot-larm-from-below* :copy-worldcoords))
  (send *arrow* :translate #f(100 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  )

(defun release-pot-from-above ()
  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :newcoords (send *pot-larm-from-above* :copy-worldcoords))
  (send *arrow* :translate #f(-60 0 60) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (reset-larm)
  )

(defun grasp-pot-from-below-model ()
  ;;腕を引いた姿勢
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  
  ;; potを下から掴む座標
  (send *arrow* :newcoords (send (send *pot* :handle-handle0) :copy-worldcoords))
  
  (send *arrow* :translate #f(60 0 -60) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate #f(-60 0 60) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)
  )

(defun grasp-pot-from-below-model-with-fail-detection ()
  (grasp-pot-from-below-model)
  
  ;; 失敗を検知する
  (grasp-fail-detection-larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-pot-from-below-model)
        (grasp-fail-detection-larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "鍋を掴むのが失敗しました，鍋の柄を掴ませる準備をしてokと合図して下さい")
              (ok-grasp-larm)
              ))
        ))
  )


(defun release-pot-from-below-model ()
  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :newcoords (send (send *pot* :handle-handle0) :copy-worldcoords))
  (send *arrow* :translate #f(100 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;;腕を引いた姿勢
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )

(defun look-at-pot ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun look-at-ih-panel ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send *ih-panel-coords* :copy-worldcoords) :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

;; tool funcs
;; spatula
(defun mix-with-spatula-set ()
  (send *ri* :start-grasp :rarm :gain 30 :wait t)
  
  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 600 8550 31145) :rpy (float-vector -3.048 -0.062 -1.634)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 550 8400 31150) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *arrow* :newcoords (make-coords :pos (float-vector 600 8300 31200) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.03 -0.018))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (mix-with-spatula) ~%")
  )


(defun mix-with-spatula ()
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 427.739 8071.081 30846.132) :rpy (float-vector -2.267 0.887 0.091)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 509.423 8110.011 30860.304) :rpy (float-vector -2.038 1.081 0.261)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 486.298 8083.5 30893.89) :rpy (float-vector -1.987 0.871 0.095)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (mix-with-spatula) again or (wait-pose-spatula) ~%")
  )

(defun rotate-mix-with-spatula (&rest args &key (times 10) &allow-other-keys)
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 461.01 8107.529 30992.813) :rpy (float-vector -2.023 0.672 0.059)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 回転させる
  (setq *cds0* (make-cascoords :pos (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))) ;;中心の座標
  (setq *cds1* (make-cascoords :pos (send (send (send *cds0* :copy-worldcoords) :translate #f(45 0 5) :world) :worldpos))) ;;回転の初期位置 ;;半径 50だった
  
  ;; (setq *rpy1* (float-vector -2.267 0.887 0.091)) ;;回転中の目標姿勢
  ;; (setq *rpy1* (float-vector -2.406 1.193 0.156)) ;;回転中の目標姿勢 ;;IK解けなかった．
  ;; (setq *rpy1* (float-vector -2.253 0.974 0.102)) ;;回転中の目標姿勢
  
  (setq *rpy1* (float-vector -2.235 1.061 0.117)) ;;回転中の目標姿勢１
  (setq *rpy2* (float-vector -2.267 0.887 0.091)) ;;回転中の目標姿勢２
  
  (send *cds0* :assoc *cds1*)
  
  (send *cds0* :rotate (deg2rad 45) :z :world)
  (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
  
  (setq hoge (send *arrow* :copy-worldcoords))
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq *rs-avs* (list))
  (setq *rs-tms* (list))
  
  (setq cnt0 0)
  
  (dotimes (i 12)
    (send *cds0* :rotate (deg2rad 30) :z :world)
    (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
    
    (setq ik-flag (send *pr2* :rarm :inverse-kinematics
                        (send *arrow* :copy-worldcoords)
                        :move-target (car (send *spatula* :attention))
                        :rotation-axis t
                        :debug-view nil
                        :stop 10)
          )
    (if (eql ik-flag nil)
        (progn
          (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy2*))
          (setq ik-flag (send *pr2* :rarm :inverse-kinematics
                              (send *arrow* :copy-worldcoords)
                              :move-target (car (send *spatula* :attention))
                              :rotation-axis t
                              :debug-view nil
                              :stop 10
                              ))
          (if (eql ik-flag nil)
              (progn
                (send *pr2* :rarm :inverse-kinematics
                      (send *arrow* :copy-worldcoords)
                      :move-target (car (send *spatula* :attention))
                      :rotation-axis nil
                      :debug-view *debug-flag*)
                (setq cnt0 (+ cnt0 1))
                ))))
    
    (send *viewer* :draw-objects)
    (setq av (send *pr2* :angle-vector))
    (setq *rs-avs* (append *rs-avs* (list av)))
    (setq *rs-tms* (append *rs-tms* (list 400)))
    )

  (setq cnt1 0)
  (dotimes (i times)
    (send *ri* :angle-vector-sequence *rs-avs* *rs-tms*)
    (send *ri* :wait-interpolation)
    
    (setq cnt1 (+ cnt1 1))
    (if (oddp cnt1)
        (progn
          (squeeze-pot-test-1)
          (squeeze-pot-test-2)
          ))
    )
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 461.01 8107.529 30992.813) :rpy (float-vector -2.023 0.672 0.059)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (print cnt0)
  
  (warn "please run (mix-with-spatula) again or (wait-pose-spatula) ~%")
  )

(defun squeeze-pot-test-1 ()
  ;; ;; 最初の位置
  ;; (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; ;; 姿勢１
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 367.14 8060.574 30927.087) :rpy (float-vector -2.349 0.916 0.106)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 405.657 8052.333 30863.374) :rpy (float-vector -2.138 1.077 0.298)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)

  ;; (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 20) :world) :worldpos) :rpy (float-vector -2.356 1.396 0.0)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  (setq avs (list))
  (setq tms (list))
  
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  ;; 姿勢１
  (send *arrow* :newcoords (make-coords :pos (float-vector 367.14 8060.574 30927.087) :rpy (float-vector -2.349 0.916 0.106)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 405.657 8052.333 30863.374) :rpy (float-vector -2.138 1.077 0.298)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))

  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 20) :world) :worldpos) :rpy (float-vector -2.356 1.396 0.0)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  )

(defun squeeze-pot-test-2 ()
  ;; ;; 最初の位置
  ;; (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; ;; 姿勢１
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 478.279 8174.918 30891.519) :rpy (float-vector -1.979 1.493 0.313)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 479.457 8185.891 30850.187) :rpy (float-vector -1.642 1.542 0.658)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 20) :world) :worldpos) :rpy (float-vector -2.356 1.396 0.0)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  (setq avs (list))
  (setq tms (list))
  
  ;; todo
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  ;; 姿勢１
  (send *arrow* :newcoords (make-coords :pos (float-vector 478.279 8174.918 30891.519) :rpy (float-vector -1.979 1.493 0.313)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 479.457 8185.891 30850.187) :rpy (float-vector -1.642 1.542 0.658)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 20) :world) :worldpos) :rpy (float-vector -2.356 1.396 0.0)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 600)))

  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  )

(defun move-pot-test ()
  ;; 鍋を掴んでいる前提
  (send *pr2* :larm :move-end-pos #f(0 0 100) :world)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq avs (list))
  (setq tms (list))
  
  ;; 前後に動かす
  (dotimes (i 3)
    (send *pr2* :larm :move-end-pos #f(100 0 0) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    
    (send *pr2* :larm :move-end-pos #f(-100 0 0) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(0 0 -100) :world)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun rotate-mix-with-spatula-reverse (&rest args &key (times 10) &allow-other-keys)
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 461.01 8107.529 30992.813) :rpy (float-vector -2.023 0.672 0.059)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 回転させる
  (setq *cds0* (make-cascoords :pos (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))) ;;中心の座標
  (setq *cds1* (make-cascoords :pos (send (send (send *cds0* :copy-worldcoords) :translate #f(-45 0 5) :world) :worldpos))) ;;回転の初期位置 ;;半径 50だった
  
  ;; (setq *rpy1* (float-vector -2.267 0.887 0.091)) ;;回転中の目標姿勢
  ;; (setq *rpy1* (float-vector -2.406 1.193 0.156)) ;;回転中の目標姿勢 ;;IK解けなかった．
  ;; (setq *rpy1* (float-vector -2.253 0.974 0.102)) ;;回転中の目標姿勢
  
  (setq *rpy1* (float-vector -2.235 1.061 0.117)) ;;回転中の目標姿勢１
  (setq *rpy2* (float-vector -2.267 0.887 0.091)) ;;回転中の目標姿勢２
  
  (send *cds0* :assoc *cds1*)
  
  (send *cds0* :rotate (deg2rad -45) :z :world)
  (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
  
  (setq hoge (send *arrow* :copy-worldcoords))
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq avs (list))
  (setq tms (list))

  (setq cnt0 0)

  (dotimes (i 12)
    (send *cds0* :rotate (deg2rad -30) :z :world)
    (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
    
    (setq ik-flag (send *pr2* :rarm :inverse-kinematics
                        (send *arrow* :copy-worldcoords)
                        :move-target (car (send *spatula* :attention))
                        :rotation-axis t
                        :debug-view nil
                        :stop 10)
          )
    (if (eql ik-flag nil)
        (progn
          (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy2*))
          (setq ik-flag (send *pr2* :rarm :inverse-kinematics
                              (send *arrow* :copy-worldcoords)
                              :move-target (car (send *spatula* :attention))
                              :rotation-axis t
                              :debug-view nil
                              :stop 10
                              ))
          (if (eql ik-flag nil)
              (progn
                (send *pr2* :rarm :inverse-kinematics
                      (send *arrow* :copy-worldcoords)
                      :move-target (car (send *spatula* :attention))
                      :rotation-axis nil
                      :debug-view *debug-flag*)
                (setq cnt0 (+ cnt0 1))
                ))))
    
    (send *viewer* :draw-objects)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 600)))
    )

  (dotimes (i times)
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    )
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector -2.288 0.713 0.075)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 461.01 8107.529 30992.813) :rpy (float-vector -2.023 0.672 0.059)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (print cnt0)
  
  (warn "please run (mix-with-spatula) again or (wait-pose-spatula) ~%")
  )


(defun wait-pose-spatula ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.03 -0.018))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (mix-with-spatula) or (avoid-rarm-spatula) ~%")
  )

(defun avoid-rarm-spatula ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 470 8105 31070) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *spatula* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 550 8450 31150) :rpy (float-vector -1.603 0.03 -0.018)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :move-target (car (send *spatula* :attention))
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)

  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 600 8600 31144) :rpy (float-vector -3.048 -0.062 -1.634)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)
  ;; (send *viewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  ;; (send *ri* :wait-interpolation)
  
  ;; (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  ;; (send *pr2* :rarm :angle-vector *rav0*)  
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (after-transfer) ~%")
  )

;; ladle
(defun mix-with-ladle-set ()
  (send *ri* :start-grasp :rarm :gain 30 :wait t)

  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis nil
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 550 8250 31180) :rpy (float-vector -1.603 0.0 3.14)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (mix-with-ladle) ~%")
  )

(defun mix-with-ladle ()
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 417.495 8067.248 30861.184) :rpy (float-vector 1.772 1.446 1.068)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 540.829 8145.059 30854.487) :rpy (float-vector 0.509 0.96 -0.306)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 485.52 8113.658 30870.807) :rpy (float-vector 0.77 1.155 -0.115)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun mix-with-ladle-test-1 ()
  ;; 最初の位置  
  (send *arrow* :newcoords (make-coords :pos (float-vector 417.495 8067.248 30861.184) :rpy (float-vector 1.772 1.446 1.068)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq avs (list))
  (setq tms (list))

  (send *arrow* :newcoords (make-coords :pos (float-vector 417.495 8067.248 30861.184) :rpy (float-vector 1.772 1.446 1.068)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 400)))
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 540.829 8145.059 30854.487) :rpy (float-vector 0.509 0.96 -0.306)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 400)))
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 485.52 8113.658 30870.807) :rpy (float-vector 0.77 1.155 -0.115)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (setq av (send *pr2* :angle-vector))
  (setq avs (append avs (list av)))
  (setq tms (append tms (list 400)))

  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  (warn "please run (mix-with-ladle) again or (wait-pose-ladle) ~%")
  )

(defun rotate-mix-with-ladle (&rest args &key (times 10) &allow-other-keys)
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector 0.251 1.492 -0.625)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 回転させる
  (setq *cds0* (make-cascoords :pos (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))) ;;中心の座標
  (setq *cds1* (make-cascoords :pos (send (send (send *cds0* :copy-worldcoords) :translate #f(-50 0 30) :world) :worldpos))) ;;回転の初期位置
  
  (setq *rpy1* (float-vector 1.772 1.446 1.068)) ;;回転中の目標姿勢
  
  (send *cds0* :assoc *cds1*)
  
  (send *cds0* :rotate (deg2rad 45) :z :world)
  (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
  
  (setq hoge (send *arrow* :copy-worldcoords))
  
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)  
  
  (setq *rl-avs* (list))
  (setq *rl-tms* (list))

  (setq cnt0 0)
  
  (dotimes (i 12)
    (send *cds0* :rotate (deg2rad 30) :z :world)
    (send *arrow* :newcoords (make-coords :pos (send (send *cds1* :copy-worldcoords) :worldpos) :rpy *rpy1*))
    
    (setq ik-flag (send *pr2* :rarm :inverse-kinematics
                        (send *arrow* :copy-worldcoords)
                        :move-target (elt (send *ladle* :attention) 1)
                        :rotation-axis t
                        :debug-view nil))
    (if (eql ik-flag nil)
        (progn    
          (send *pr2* :rarm :inverse-kinematics
                (send *arrow* :copy-worldcoords)
                :move-target (elt (send *ladle* :attention) 1)
                :rotation-axis nil
                :debug-view *debug-flag*)
          (setq cnt0 (+ cnt0 1))
          ))
    (send *viewer* :draw-objects)
    (setq av (send *pr2* :angle-vector))
    (setq *rl-avs* (append *rl-avs* (list av)))
    (setq *rl-tms* (append *rl-tms* (list 400)))
    )

  (setq cnt1 0)
  (dotimes (i times)
    (send *ri* :angle-vector-sequence *rl-avs* *rl-tms*)
    (send *ri* :wait-interpolation)
    
    (setq cnt1 (+ cnt1 1))
    (if (oddp cnt1)
        (progn
	  (mix-with-ladle-test-1)
          ))
    )
  
  (send *arrow* :newcoords (make-coords :pos (send (send (send (send *pot* :attention-attention0) :copy-worldcoords) :translate #f(0 0 60) :world) :worldpos) :rpy (float-vector 0.251 1.492 -0.625)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 3)
    (send *pr2* :rarm :move-end-pos #f(0 0 30) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    
    (send *pr2* :rarm :move-end-pos #f(0 0 -30) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  (print cnt0)
  
  (warn "please run (mix-with-ladle) again or (wait-pose-ladle) ~%")
  )

(defun wait-pose-ladle ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun avoid-rarm-ladle ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 雫を落とす動作
  ;; WIP !!!!!
  
  ;;
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 550 8250 31180) :rpy (float-vector -1.603 0.0 3.14)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 600 8600 31144) :rpy (float-vector -3.048 -0.062 -1.634)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  ;; (send *pr2* :rarm :angle-vector *rav0*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (after-transfer) ~%")
  )


;; ih-with-rec codes
(defun ih-with-rec-init ()
  ;; point-pose-extractor
  (require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
  (defparameter *detection-topic* "/ObjectDetection")

  (setq local-cds (make-coords :pos (float-vector 527.349 424.972 760.996) :rpy (float-vector 3.129 0.857 -0.026)))
  (setq *s-panel* (make-cube 150 230 10))
  (send *s-panel* :move-to *pr2* :world)
  (send *s-panel* :transform (send local-cds :copy-worldcoords))
  (send *s-panel* :newcoords (make-coords :pos (send (send *s-panel* :copy-worldcoords) :worldpos)))
  (send *s-panel* :rotate (deg2rad 90) :y)
  (send *s-panel* :translate #f(75 115 0))

  (objects (list *pr2* *scene* *spatula* *ladle* *pot* *s-panel*))
  (update-view)
  (warn "please run (ih-recognition) ~%")
  )

(defun ih-recognition ()
  (set-pose-3)
  (setq s-pos (float-vector 0 0 0))
  (dotimes (i 10)
    (setq local-cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
    (send *s-panel* :move-to *pr2* :world)
    (send *s-panel* :transform (send local-cds :copy-worldcoords))
    (setq s-pos (v+ s-pos (send *s-panel* :pos)))
    )
  (send *s-panel* :newcoords (make-coords :pos (scale 0.1 s-pos)))
  (send *s-panel* :rotate (deg2rad 90) :y)
  (send *s-panel* :translate #f(75 115 0))
  
  (warn "please run (on-stove-with-rec) ~%")
  )

(defun on-stove-with-rec ()
  (send *ri* :start-grasp :larm :wait t)
  
  ;; スイッチの位置
  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 47) 30750) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(130 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 47) 30750) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(130 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (push-knob-with-rec) ~%")
  )

(defun off-stove-with-rec ()
  (send *ri* :start-grasp :larm :wait t)
  
  ;; スイッチの位置
  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 37) 30750) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(120 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (push-stove-button)

  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 37) 30750) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun push-knob-with-rec ()
  (send *ri* :start-grasp :larm :wait t)

  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 30) 30715) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(130 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 30) 30715) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun grasp-knob-with-rec ()
  (send *ri* :move-gripper :larm 0.08)
  
  (setq s-pos (send (send *s-panel* :copy-worldcoords) :worldpos))
  (send *arrow* :newcoords (make-coords :pos (float-vector (elt s-pos 0) (+ (elt s-pos 1) 30) 30715) :rpy (float-vector -3.047 -0.011 1.512)))
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :translate #f(130 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(10 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (send *ri* :start-grasp :larm :wait t :gain 100)
  
  (warn "please run (rotate-knob) ~%")
  )



;; now funcs
(defun ih-test ()
  (set-pose-3)
  (on-stove) ;;電源を入れる
  (push-knob) ;;つまみを押す
  
  (grasp-knob) ;;つまみを掴む
  (rotate-knob :deg 30) ;;時計回りに30度つまみを回す
  (release-knob) ;;つまみを離す
  
  (push-knob) ;;つまみを押す
  (off-stove) ;;電源を切る
  (set-pose-3)
  )

(defun ih-test-with-rec ()
  (set-pose-3)
  (ih-with-rec-init)
  (ih-recognition)
  
  (on-stove-with-rec) ;;電源を入れる
  (push-knob-with-rec) ;;つまみを押す

  (grasp-knob-with-rec) ;;つまみを掴む
  (rotate-knob :deg 30) ;;時計回りに30度つまみを回す
  (release-knob) ;;つまみを離す

  (push-knob-with-rec) ;;つまみを押す
  (off-stove-with-rec) ;;電源を切る
  (set-pose-3)
  )

(defun now-on-stove ()
  (set-pose-3)
  (on-stove) ;;電源を入れる
  )

(defun now-start-ih (&rest args &key (deg -10) &allow-other-keys)
  (set-pose-3)
  (push-knob)
  (grasp-knob)
  (rotate-knob :deg 30)
  (rotate-knob :deg deg)
  (release-knob)
  (set-pose-3)
  )

(defun now-start-ih-with-fail-detection (&rest args &key (deg -10) &allow-other-keys)
  (set-pose-3)
  (push-knob)

  (grasp-knob)
  ;; 失敗を検知する
  (grasp-fail-detection-larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (release-knob)
	(push-knob)
        (grasp-knob)
        (grasp-fail-detection-larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (release-knob)
              (set-pose-3)
              (send *ri* :speak-jp "IHコンロのつまみの操作に失敗しました，コンロをつけてokと合図をして下さい")
              (ok-wait)
              (return-from now-start-ih-with-fail-detection)))
        ))
  
  (rotate-knob :deg 30)
  (rotate-knob :deg deg)
  (release-knob)
  (set-pose-3)
  )


(defun knob-test (&rest args &key (deg 30) &allow-other-keys)
  ;;つまみが出ている状態で調節する
  (grasp-knob) ;;つまみを掴む
  (rotate-knob :deg deg) ;;時計回りにdeg度つまみを回す
  (release-knob) ;;つまみを離す
  )


(defun spatula-mix-test ()
  ;; ヘラでかき混ぜる動作のテスト
  (grasp-pot-from-below)
  (look-at-pot)
  (grasp-spatula)
  (mix-with-spatula-set)
  (mix-with-spatula)
  (wait-pose-spatula)
  (avoid-rarm-spatula)
  (release-pot-from-below)
  )

(defun ladle-mix-test ()
  ;; おたまでかき混ぜる動作のテスト
  (grasp-ladle)
  (grasp-pot-from-below)
  (look-at-pot)
  (mix-with-ladle-set)
  (mix-with-ladle)
  (wait-pose-ladle)
  (avoid-rarm-ladle)
  (release-pot-from-below)
  )


(defun now-spatula-mix ()
  ;; ヘラでかき混ぜる動作 ヘラは掴んでいる前提
  (grasp-pot-from-below)
  (look-at-pot)
  (set-spatula)
  (mix-with-spatula-set)
  (mix-with-spatula)
  (wait-pose-spatula)
  (avoid-rarm-spatula)
  (release-pot-from-below)
  )

(defun now-ladle-mix ()
  ;; おたまでかき混ぜる動作 おたまは掴んでいる前提
  (set-ladle)
  (grasp-pot-from-below)
  (look-at-pot)
  (mix-with-ladle-set)
  (mix-with-ladle)
  (wait-pose-ladle)
  (avoid-rarm-ladle)
  (release-pot-from-below)
  )


(defun spatula-mix-loop (&rest args &key (times 10) &allow-other-keys)
  (wait-pose-spatula)
  (dotimes (i times)
    (mix-with-spatula)
    )
  (wait-pose-spatula)
  )

(defun ladle-mix-loop (&rest args &key (times 10) &allow-other-keys)
  (wait-pose-ladle)
  (dotimes (i times)
    (mix-with-ladle)
    )
  (wait-pose-ladle)
  )

(defun spatula-mix-rotate-loop (&rest args &key (times 10) &allow-other-keys)
  (look-at-pot)
  ;; (rossrv-test)
  (wait-pose-spatula)
  (rotate-mix-with-spatula :times times)
  (wait-pose-spatula)
  ;; (rossrv-test)
  )

(defun ladle-mix-rotate-loop (&rest args &key (times 10) &allow-other-keys)
  (look-at-pot)
  ;; (rossrv-test)
  (wait-pose-ladle)
  (rotate-mix-with-ladle :times times)
  (wait-pose-ladle)
  ;; (rossrv-test)
  )

(defun now-start-spatula-mix ()
  (grasp-pot-from-below-model)
  (look-at-pot)
  (set-spatula)
  (mix-with-spatula-set)
  )

(defun after-spatula-mix ()
  (avoid-rarm-spatula)
  (release-pot-from-below-model)
  )

(defun now-start-ladle-mix ()
  (grasp-pot-from-below-model)
  (look-at-pot)
  (set-ladle)
  (mix-with-ladle-set)
  )

(defun after-ladle-mix ()
  (avoid-rarm-ladle)
  (release-pot-from-below-model)
  )



(defun now-knob (&rest args &key (deg -70) &allow-other-keys)
  ;;つまみが出ている状態で調節する
  (look-at-ih-panel)
  (push-knob) ;;つまみを押してonにする
  (grasp-knob) ;;つまみを掴む
  (rotate-knob :deg 30) ;;時計回りにdeg度つまみを回す
  (rotate-knob :deg deg) ;;時計回りにdeg度つまみを回す
  ;; (rossrv-test)
  (release-knob) ;;つまみを離す
  )

(defun dialogue-test ()
  (setq *yes-flag* nil)
  (until *yes-flag*
         (unix:sleep 1)
         (send *ri* :speak-jp "画像を記録します")
         ;; (rossrv-test)
         (send *ri* :speak-jp "記録しました")
         (unix:sleep 1)
         (send *ri* :speak-jp "具材は炒め終わっていますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           )
         )
  (send *ri* :speak-jp "次の工程に進みます")
  (unix:sleep *w-time*)
  )

(defun now-spatula-mix-rotate-loop-with-dialogue (&rest args &key (times 10) &allow-other-keys)
  (look-at-pot)
  ;; (rossrv-test)
  (wait-pose-spatula)
  (setq *yes-flag* nil)
  (until *yes-flag*
         (rotate-mix-with-spatula :times times)
         (wait-pose-spatula)
         (send *ri* :speak-jp "画像を記録します")
         ;; (rossrv-test)
         ;; (rossrv-test-double)
         (send *ri* :speak-jp "記録しました")
         (unix:sleep 1)
         (send *ri* :speak-jp "具材は炒め終わっていますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           )
         )
  (send *ri* :speak-jp "次の工程に進みます")  
  (unix:sleep *w-time*)
  )

(defun now-ladle-mix-rotate-loop-with-dialogue (&rest args &key (times 10) &allow-other-keys)
  (look-at-pot)
  ;; (rossrv-test)
  (wait-pose-ladle)
  (setq *yes-flag* nil)
  (until *yes-flag*
         (rotate-mix-with-ladle :times times)
         (wait-pose-ladle)
         (send *ri* :speak-jp "画像を記録します")
         ;; (rossrv-test)
         ;; (rossrv-test-double)
         (send *ri* :speak-jp "記録しました")
         (unix:sleep 1)
         (send *ri* :speak-jp "具材は上手く混ざっていますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           )
         )
  (send *ri* :speak-jp "次の工程に進みます")  
  (unix:sleep *w-time*)
  )

(defun now-boil-with-dialogue (&rest args &key (sleep-time 30) &allow-other-keys)
  (look-at-pot)
  ;; (rossrv-test)
  (setq *yes-flag* nil)
  (until *yes-flag*
         (unix:sleep sleep-time)
         (send *ri* :speak-jp "画像を記録します")
         ;; (rossrv-test)
         ;; (rossrv-test-double)
         (send *ri* :speak-jp "記録しました")
         (send *ri* :speak-jp "鍋の中身は沸騰していますか？")
         (dialogue-func)
         (if (equal *data* '("はい"))
             (setq *yes-flag* t)
           )
         )
  (send *ri* :speak-jp "次の工程に進みます")  
  )



(defun tmp-test ()
  (set-spatula)
  (rotate-mix-with-spatula :times 5)
  (squeeze-pot-test-1)
  (squeeze-pot-test-2)
  (wait-pose-spatula)
  (move-pot-test)
  )

(defun tmp-set ()
  (update-pos)
  (set-spatula)
  )


(defun tmp-pose ()
  (send *pr2* :rarm :angle-vector #f(-24.7324 5.33759 -49.5234 -9.68102 317.721 -106.132 -360.525))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  )

  
