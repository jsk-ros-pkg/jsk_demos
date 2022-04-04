(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/test/pr2-random-coords.l")

(ros::roseus "ih-at-arrange" :anonymous t)

(defun ih-init ()
  ;; set objects
  (setq *pot* (cooking-pot))
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))

  ;; def global coords
  (setq *pot-larm-from-above* (make-coords :pos (float-vector 639 8109 30971) :rpy (float-vector -3.069 1.306 -0.087)))
  (setq *pot-larm-from-below* (make-coords :pos (float-vector 644 8104 30947) :rpy (float-vector 3.125 -0.909 3.119)))

  (setq *ih-panel-coords* (make-coords :pos (float-vector 563 7940 30865) :rpy (float-vector  3.044 1.407 2.87)))

  ;; arrange original
  (setq *ih-knob-coords* (make-coords :pos (float-vector 680 8110 30715) :rpy (float-vector -3.047 -0.011 1.512))) ;; ここを変える。(correct-ih-knob-coords)関数で自動的に変更可能

  (move-on-irt :arrange)
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

;; IH funcs
(defun push-stove-button ()
  (let (av avs tms)
    (dotimes (i 15)
      (send *pr2* :larm :move-end-pos #f(-10 0 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400)))
      )
    (send *ri* :angle-vector-sequence avs tms))

  (cancel-motion-when-touch :arm :larm :direction :x :threshold -15)
  )

(defun push-knob ()
  ;; If IK fail, try (set-pose)
  (send *ri* :start-grasp :larm :wait t)

  (let ((ik-target (send *ih-knob-coords* :copy-worldcoords)))
    (send ik-target :translate #f(100 0 0) :world)

    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (push-stove-button)

    (send ik-target :newcoords (send *ih-knob-coords* :copy-worldcoords))
    (send ik-target :translate #f(100 0 0) :world)

    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
  ))

(defun grasp-knob ()
  (send *ri* :move-gripper :larm 0.08)

  (let ((ik-target (send *ih-knob-coords* :copy-worldcoords)))
    (send ik-target :rotate (deg2rad 90) :x)
    (send ik-target :translate #f(100 0 0) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (push-stove-button)

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send ik-target :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(10 0 0) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))

  (send *ri* :start-grasp :larm :wait t :gain 100)
  )

(defun rotate-knob (&rest args &key (deg 30) &allow-other-keys)
  ;; rotate
  (let (av avs tms (ik-target (send (send *pr2* :larm :end-coords) :copy-worldcoords)))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (if (> deg 0)
      (progn
        (dotimes (i (/ deg 10))
          (send ik-target :rotate (deg2rad 10) :x)
          (send *pr2* :larm :inverse-kinematics
                (send ik-target :copy-worldcoords)
                :rotation-axis t
                :debug-view nil)
          (setq av (send *pr2* :angle-vector))
          (setq avs (append avs (list av)))
          (setq tms (append tms (list 200)))
          ))
      (dotimes (i (/ (* deg -1) 10))
        (send ik-target :rotate (deg2rad -10) :x)
        (send *pr2* :larm :inverse-kinematics
              (send ik-target :copy-worldcoords)
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
  )

(defun release-knob ()
  (send *ri* :stop-grasp :larm :wait t)

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((ik-target (send (send *pr2* :larm :end-coords) :copy-worldcoords)))
    (send ik-target :translate #f(30 0 0) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send ik-target :newcoords (send *ih-knob-coords* :copy-worldcoords))
    (send ik-target :rotate (deg2rad 90) :x)
    (send ik-target :translate #f(100 0 0) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
  )

;; pot funcs
(defun grasp-pot-from-below ()
  ;;腕を引いた姿勢
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; potを下から掴む座標
  (let ((ik-target (send *pot-larm-from-below* :copy-worldcoords)))
    (send ik-target :translate #f(60 0 -60) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *ri* :stop-grasp :larm :wait t)

    (send ik-target :translate #f(-60 0 60) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :larm :wait t))
  )

(defun grasp-pot-from-below-with-fail-detection ()
  (grasp-pot-from-below)

  ;; 失敗を検知する
  (grasp-fail-detection :larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-pot-from-below)
        (grasp-fail-detection :larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "鍋を掴むのが失敗しました，鍋の柄を掴ませる準備をしてokと合図して下さい")
              (ok-grasp :arm :larm)
              ))
        ))
  )

(defun release-pot-from-below ()
  (send *ri* :stop-grasp :larm :wait t)

  (let ((ik-target (send *pot-larm-from-below* :copy-worldcoords)))
    (send ik-target :translate #f(100 0 -100) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))
  )

(defun grasp-pot-from-below-model ()
  ;;腕を引いた姿勢
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; potを下から掴む座標
  (let ((ik-target (send (send *pot* :handle-handle0) :copy-worldcoords)))
    (send ik-target :translate #f(60 0 -60) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *ri* :stop-grasp :larm :wait t)

    (send ik-target :translate #f(-60 0 60) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))

  (send *ri* :start-grasp :larm :wait t)
  )

(defun grasp-pot-from-below-model-with-fail-detection ()
  (grasp-pot-from-below-model)

  ;; 失敗を検知する
  (grasp-fail-detection :larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-pot-from-below-model)
        (grasp-fail-detection :larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "鍋を掴むのが失敗しました，鍋の柄を掴ませる準備をしてokと合図して下さい")
              (ok-grasp :arm :larm)
              ))
        ))
  )

(defun release-pot-from-below-model ()
  (send *ri* :stop-grasp :larm :wait t)

  (let ((ik-target (send (send *pot* :handle-handle0) :copy-worldcoords)))
    (send ik-target :translate #f(100 0 -100) :world)
    (send *pr2* :larm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))

  ;;腕を引いた姿勢
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
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
(defun start-ih-with-fail-detection (&rest args &key (deg -10) &allow-other-keys)
  (set-pose-3)
  (push-knob)

  (grasp-knob)
  ;; 失敗を検知する
  (grasp-fail-detection :larm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (release-knob)
        (push-knob)
        (grasp-knob)
        (grasp-fail-detection :larm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (release-knob)
              (set-pose-3)
              (send *ri* :speak-jp "IHコンロのつまみの操作に失敗しました，コンロをつけてokと合図をして下さい")
              (ok-wait)
              (return-from start-ih-with-fail-detection)))
        ))

  (rotate-knob :deg 30)
  (rotate-knob :deg deg)
  (release-knob)
  (set-pose-3)
  )

;; correct position of IH by touch and wrench
;; BE CAREFUL that this function changes *ih-knob-coords* variable
(defun correct-ih-knob-coords ()
  (when (send *ri* :simulation-modep)
    (send *pr2* :move-to
          (make-coords :pos #f(1247.602 8254.205 30013.269) :rpy #f(3.132 0.018 -0.008)) :world)
    (return-from correct-ih-knob-coords))
  ;; ;; random coords when kinematics simulator
  ;; (if (send *ri* :simulation-modep)
  ;;   (send *pr2* :move-to (send (random-coords (send *arrange-spot* :copy-worldcoords) :x 10 :y 10 :theta 5) :copy-worldcoords) :world))

  (send *ri* :speak-jp "IHコンロの位置を触って確認します")
  (set-pose)
  (update-pos)
  (send *ri* :start-grasp :larm :wait t)

  ;; Mainly copied from (push-knob)
  ;; Pre pose
  (let (av1 av2 av3)
    (send *pr2* :rarm :move-end-pos #f(0 200 0) :world)
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :larm :inverse-kinematics
          (send *ih-knob-coords* :copy-worldcoords))
    (send *pr2* :larm :move-end-pos #f(100 200 250) :world)
    (send *pr2* :larm :move-end-rot 90 :x)
    (setq av2 (send *pr2* :angle-vector))
    (send *pr2* :larm :move-end-pos #f(-150 0 0) :world)
    (setq av3 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3) (list 1000 2000 2000))
    (send *ri* :wait-interpolation))
  (update-view)

  ;; Detect z direction
  (let (av avs tms)
    (dotimes (i 15)
      (send *pr2* :larm :move-end-pos #f(0 0 -10) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 200))))
    (send *ri* :angle-vector-sequence avs tms)
    (cancel-motion-when-touch :arm :larm :direction :z :threshold -15))
  (update-view)

  (let (av1 av2 av3)
    (send *pr2* :larm :move-end-pos #f(150 -30 100) :world)
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :larm :move-end-pos #f(0 -50 -150) :world)
    (setq av2 (send *pr2* :angle-vector))
    (send *pr2* :larm :move-end-pos #f(0 30 -160) :world)
    (send *pr2* :larm :move-end-rot -90 :x)
    (setq av3 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3) (list 1000 1000 1000))
    (send *ri* :wait-interpolation))
  (update-view)

  ;; Detect x direction for the next y ditection
  (let (av avs tms)
    (dotimes (i 15)
      (send *pr2* :larm :move-end-pos #f(-10 0 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 200))))
    (send *ri* :angle-vector-sequence avs tms)
    (cancel-motion-when-touch :arm :larm :direction :x :threshold -15)
    (send *pr2* :larm :move-end-pos #f(5 0 0) :world))
  (update-view)

  ;; Detect y direction of *ih-knob-coords*
  (let (av avs tms)
    (dotimes (i 15)
      (send *pr2* :larm :move-end-pos #f(0 -10 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms)
    (cancel-motion-when-touch :arm :larm :direction :y :threshold -6))
  (update-view)

  ;; Calculate *ih-knob-coords*
  (unix:usleep (* 100 1000))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((ih-knob-coords (copy-object *ih-knob-coords*))
         (fingertip (send (send *pr2* :larm :end-coords :copy-worldcoords)
                          :translate #f(20 0 0) :local))
         (new-ih-knob-coords (send fingertip :translate #f(0 -55 85) :world))
         diff)
    ;; Update *pr2* worldcoords
    (setq diff (v- (send *ih-knob-coords* :worldpos)
                   (send new-ih-knob-coords :worldpos)))
    (send *pr2* :translate diff :world)
    (format t "*pr2* coords is updated to ~A~%"
            (send *pr2* :worldcoords)))

  ;; Reset PR2 left arm
  (send *pr2* :larm :move-end-pos #f(100 0 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (update-view)
  )

(defun ih-check ()
  (send *ri* :speak-jp "IHコンロを操作できるかチェックします")
  (set-pose)
  (update-view)
  ;; Check if PR2 can handle IH kitchen
  (start-ih-with-fail-detection :deg 10)
  (update-view)
  (push-knob)
  (update-view)
  (set-pose)
  (update-view)
  )

;; Motions with interation
;; IHコンロを使ってお湯を沸騰させる
(defun boil-soup ()
  (send *ri* :speak-jp "沸騰させる調理を開始します")

  ;; 着火する
  (start-ih-with-fail-detection :deg 10)
  (update-view)

  ;; 沸騰するまで待つ
  (look-at-pot)
  (update-view)
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
        (boil-with-dialogue))))

  ;; 火を止める
  (push-knob)
  (update-view)
  (set-pose-3)
  (update-view)

  (send *ri* :speak-jp "沸騰させる調理を行いました．" :wait t)
  (send *ri* :speak-jp "次の工程に移ります" :wait t)
  )
