;; remove interaction.l
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/interaction.l")
;; remove tool-at-arrange.l
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/tool-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")

(defun ih-init ()
  ;; set objects
  (setq *pot* (cooking-pot))
  (setq *pot-coords* (make-coords :pos (float-vector 450 8030 30850) :rpy (float-vector 3.142 0 0))) ;; arrange original ;;ここを変える
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))

  ;; def global coords
  (setq *pot-larm-from-above* (make-coords :pos (float-vector 639 8109 30971) :rpy (float-vector -3.069 1.306 -0.087)))
  (setq *pot-larm-from-below* (make-coords :pos (float-vector 644 8104 30947) :rpy (float-vector 3.125 -0.909 3.119)))

  (setq *ih-panel-coords* (make-coords :pos (float-vector 563 7940 30865) :rpy (float-vector  3.044 1.407 2.87)))

  ;; arrange original
  (setq *ih-knob-coords* (make-coords :pos (float-vector 680 8110 30715) :rpy (float-vector -3.047 -0.011 1.512))) ;; ここを変える

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

;; add for grasp detection
(defun grasp-fail-detection-larm (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (setq gripper-len (send *ri* :state :gripper :larm :position))
  (if (< gripper-len g-thre)
      (setq *fail-flag* t))
  )

;; Temperature recognition
(ros::load-ros-manifest "std_msgs")
(setq *max-temp* nil)
(setq *min-temp* nil)
(setq *center-temp* nil)
(defun max-temp-cb (msg)
  (setq *max-temp* (send msg :data)))
(defun min-temp-cb (msg)
  (setq *min-temp* (send msg :data)))
(defun center-temp-cb (msg)
  (setq *center-temp* (send msg :data)))
(ros::subscribe "/thermal/max_temp" std_msgs::Int16 #'max-temp-cb)
(ros::subscribe "/thermal/min_temp" std_msgs::Int16 #'min-temp-cb)
(ros::subscribe "/thermal/center_temp" std_msgs::Int16 #'center-temp-cb)

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
              (ok-grasp :arm larm)
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
              (ok-grasp :arm larm)
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

;; tool funcs
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

(defun ih-check ()
  ;; (update-pos)
  (set-pose)
  (now-start-ih-with-fail-detection :deg 10)
  (push-knob)
  (set-pose)
  )
