(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")

(ros::roseus "ih-at-arrange" :anonymous t)

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
  (objects (list *pr2* *scene* *ladle* *pot*))
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

;; add for grasp detection
(defun grasp-fail-detection-larm (&rest args &key (g-thre 10) &allow-other-keys)
  (setq *fail-flag* nil)
  (setq gripper-len (send *ri* :state :gripper :larm :position))
  (if (< gripper-len g-thre)
      (setq *fail-flag* t))
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

  (cancel-motion-when-touch :arm :larm :direction :x :threshold -15)
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
              (ok-grasp :arm :larm)
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
              (ok-grasp :arm :larm)
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

;; Motions with interation
;; IHコンロを使ってお湯を沸騰させる
(defun boil-soup ()
  (send *ri* :speak-jp "沸騰させる調理を開始します")

  ;; 着火する
  (now-start-ih-with-fail-detection :deg 10)

  ;; 沸騰するまで待つ
  (look-at-pot)
  (ros::spin-once)
  (let ((max-temp-msg (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000))
        max-temp)
    (if max-temp-msg
      ;; If PR2 have thermal sensor
      (progn
        (setq max-temp (send max-temp-msg :data))
        (while (< max-temp 75)
          (send *ri* :speak-jp "まだお湯は沸騰していません" :wait t)
          (unix:sleep 15)
          (setq max-temp
                (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data)))
        (send *ri* :speak-jp "お湯が沸騰しました。" :wait t))
      ;; If PR2 does not have thermal sensor
      (progn
        (look-at-pot)
        (now-boil-with-dialogue))))

  ;; 火を止める
  (push-knob)
  (set-pose-3)

  (send *ri* :speak-jp "沸騰させる調理を行いました．" :wait t)
  (send *ri* :speak-jp "次の工程に移ります" :wait t)
  )
