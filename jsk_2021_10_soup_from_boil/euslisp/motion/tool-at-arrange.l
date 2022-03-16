(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/knife-new-attention.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/potato-peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/shelf-door.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")

(setq *debug-flag* nil)

(defun tool-init ()
  ;; set tools
  (setq *knife* (knife))
  (send *knife* :newcoords (send *knife-coords* :copy-worldcoords))
  (setq *peeler* (peeler))
  (send *peeler* :newcoords (send *peeler-coords* :copy-worldcoords))
  (setq *potato-peeler* (peeler))
  (send *potato-peeler* :newcoords (send *potato-peeler-coords* :copy-worldcoords))
  (setq *ladle* (ladle))
  (send *ladle* :newcoords (send *ladle-coords-at-kitchen* :copy-worldcoords))
  (setq *pot* (cooking-pot))
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))
  ;; set equipment
  ;; *shelf-door*のcoordsは、(correct-shelf-door-coords)で自動調整可能
  (setq *shelf-door* (shelf-door))
  (send *shelf-door* :newcoords (send *shelf-door-coords* :copy-worldcoords))
  (setq *shelf-look-coords* (send (send *shelf-door* :handle-handle0) :copy-worldcoords))
  ;; arrange original
  (setq *ladle-coords-at-arrange* (make-coords :pos (float-vector 560 8700 30860) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，盛り付け時

  (move-to-arrange-irt)
  (objects (list *pr2* *peeler* *knife* *potato-peeler* *scene* *ladle* *shelf-door*))
  (update-view)
  )

(tool-init)

;; set codes
(warn "~%  If you use real robot, run (update-pos) ~%")
(warn "please run (now-set-tools) or (set-knife) ~%")

(defun look-at-shelf ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send *shelf-look-coords* :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun receive-ladle-with-dialogue ()
  (rarm-middle-pose)
  (set-ladle)
  (arrange-motion-set)
  (send *ri* :speak-jp "おたまを渡す準備をして，okと合図をして下さい")
  (receive-with-dialogue)
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  (set-ladle)
  (send *viewer* :draw-objects)

  (warn "please run (put-ladle-kitchen) or (put-ladle-at-shelf) ~%")
  )

;; rarm-middle-poseから、おたまをキッチンの上にセットする
(defun put-ladle-at-kitchen ()
  ;; キッチン台に手が当たるまで腕を下げる
  (let (av1 av2)
    (send *arrow* :newcoords (send *ladle-coords-at-kitchen* :copy-worldcoords))
    (send *arrow* :rotate (deg2rad -90) :x)
    (send *arrow* :translate #f(0 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
    (setq av1 (send *pr2* :angle-vector))
    (setq av2 (send *pr2* :rarm :move-end-pos #f(0 0 -200) :world :use-torso nil))
    (send *ri* :angle-vector-sequence
          (list av1 av2)
          (list 3000 3000))
    (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
    (send *pr2* :rarm :move-end-pos #f(0 0 10) :world :use-torso nil)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation))

  ;; おたまを置く
  (send *ri* :move-gripper :rarm 0.08)
  (send *pr2* :rarm :end-coords :dissoc *ladle*)

  ;; 手を上げる
  (let (av1 av2)
    (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send *arrow* :translate #f(0 0 100) :world)
    (setq av1 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send *arrow* :translate #f(60 100 170) :world)
    (setq av2 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *ri* :angle-vector-sequence
          (list av1 av2)
          (list 2000 2000))
    (send *ri* :wait-interpolation))
  )

(defun put-ladle-at-arrange ()
  (send *arrow* :newcoords (send *ladle-coords-at-arrange* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(-60 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; (send *ri* :stop-grasp :rarm :wait t)
  (send *ri* :move-gripper :rarm 0.08)
  (send *pr2* :rarm :end-coords :dissoc *ladle*)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(60 100 170) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun rarm-middle-pose ()
  ;; 中間姿勢になる
  (send *arrow* :newcoords (make-coords :pos (float-vector 570 8500 31100) :rpy (float-vector 3.14 0.0 0.0)))
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso :waist-z :joint-angle 325)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        ;; :rotation-axis t
        :rotation-axis :z ;; this change may affect many demos...
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun touch-shelf-knob ()
  ;; Mainly copied from (push-stove-button) in ih-at-arrange.l
  ;; The difference is left and right
  (let (avs tms)
    (dotimes (i 15)
      (send *pr2* :rarm :move-end-pos #f(-10 0 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))

  (cancel-motion-when-touch :arm :rarm :direction :x :threshold -15)
  )

(defun open-shelf-set ()
  ;; 中間姿勢になる
  (rarm-middle-pose)
  (look-at-shelf)

  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)

  ;; 棚のノブの位置
  ;; (send *arrow* :newcoords (send *shelf-knob-closed-coords* :copy-worldcoords))
  (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))

  (send *arrow* :translate (float-vector 100 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;; ノブのところまで手を伸ばす 力覚を入れても良い．
  (touch-shelf-knob)

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(10 0 0) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm :wait t)
  (send *ri* :start-grasp :rarm :wait t :gain 0.08)

  (warn "please run (open-shelf) ~%")
  )

;; (send *shelf-door* :angle-vector #f(90))によって、(send *shelf-door* :handle)の値が書き換わるので、注意
(defun open-shelf ()
  (let ((i 0)   ;; open
        (handle-coords (make-coords))
        av avs tms)
    (while (< i 99)
      (send *shelf-door* :angle-vector (float-vector i))
      (send handle-coords :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))

      (send *pr2* :rarm :inverse-kinematics (send handle-coords :copy-worldcoords) :rotation-axis t)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 600)))

      (setq i (+ i 10))
      )
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))
  (warn "please run (after-open) ~%")
  )

(defun after-open ()
  (send *ri* :stop-grasp :rarm :wait t)

  (send *pr2* :rarm :move-end-pos #f(0 20 -80) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :move-end-pos #f(0 -100 -100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; 中間姿勢になる
  (rarm-middle-pose)

  (warn "please run (grasp-spatula-at-shelf) ~%")
  )

(defun close-shelf-set ()
  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)
  ;; ドアを閉める前の姿勢になる
  (look-at-shelf)
  (let (av1 av2 av3 av4)
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 325)
    (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
    (setq av1 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis :z
          :debug-view *debug-flag*))
    (send *shelf-door* :angle-vector (float-vector 100))
    (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
    (send *arrow* :rotate (deg2rad -70) :y)
    (send *arrow* :translate #f(-150 0 150))
    (setq av2 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *arrow* :translate (float-vector 150 0 0))
    (setq av3 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *arrow* :translate (float-vector 0 0 -150))
    (setq av4 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3 av4)
          (list 2000 3000 1000 1000))
    (send *ri* :wait-interpolation))

  (warn "please run (close-shelf) ~%")
  )

(defun close-shelf ()
  (let ((i 100)   ;; close
        (handle-coords (make-coords))
        av avs tms)
    (while (>= i 0)
      (send *shelf-door* :angle-vector (float-vector i))
      (send handle-coords :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
      (send handle-coords :rotate (deg2rad -70) :y)

      (send *pr2* :rarm :inverse-kinematics (send handle-coords :copy-worldcoords) :rotation-axis t)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 1000)))

      (setq i (- i 10))
      )
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))
  (warn "please run (after-close) ~%")
  )

(defun after-close ()
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(80 0 -80) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 中間姿勢になる
  (rarm-middle-pose)

  (reset-rarm)
  )

;; 棚の中のおたまを掴む
(defun grasp-ladle-at-shelf ()
  (if (not (eql (send *ladle* :parent) nil))
      (send (send *ladle* :parent) :dissoc *ladle*))
  (send *ladle* :newcoords (send *ladle-coords-at-shelf* :copy-worldcoords))
  (send *arrow* :newcoords (send *ladle* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  ;; 棚の中に手を伸ばす
  (send *ri* :move-gripper :rarm 0.05)
  (move-rarm-to-ladle-at-shelf)

  ;; 右手をおたまの持ち手の高さに合わせる
  (send *arrow* :translate #f(0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
  (send *pr2* :rarm :move-end-pos #f(0 0 20) :world :use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *viewer* :draw-objects)
  (send *ri* :wait-interpolation)

  ;; おたまを掴む
  (send *ri* :start-grasp :rarm :wait t)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  (warn "please run (put-ladle-at-kitchen) ~%")
  )

(defun grasp-ladle-at-shelf-with-fail-detection ()
  (grasp-ladle-at-shelf)

  ;; 失敗を検知する
  (grasp-fail-detection :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
    (progn
        (grasp-ladle-at-shelf)
        (grasp-fail-detection :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "おたまの把持が失敗しました，渡す準備をしてokと合図をして下さい")
              (receive-with-dialogue)
              (send *ri* :start-grasp :rarm :wait t)
              ))
        ))
  )

(defun move-rarm-to-ladle-at-shelf ()
    ;; 中間姿勢になる
  (rarm-middle-pose)
  (look-at-shelf)

  ;; ladleを置く位置
  (let (av1 av2 av3)
    (send *arrow* :newcoords (send *ladle-coords-at-shelf* :copy-worldcoords))
    (send *arrow* :rotate (deg2rad -90) :x)
    (send *arrow* :translate #f(280 0 -70) :world)
    (setq av1 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis :z
                    :debug-view *debug-flag*))
    (send *arrow* :translate #f(50 -30 100) :world)
    (send *arrow* :rotate (deg2rad 20) :z)
    (setq av2 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *arrow* :rotate (deg2rad -20) :z)
    (send *arrow* :translate #f(-130 80 0) :world)
    (setq av3 (send *pr2* :rarm :inverse-kinematics
                    (send *arrow* :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*))
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3)
          (list 3000 3000 1000))
    (send *ri* :wait-interpolation))

  ;; 手をスライドさせる
  (let (av avs tms)
    (dotimes (i 28)
      (send *arrow* :translate (float-vector -10 0 0) :world)
      (send *pr2* :rarm :inverse-kinematics
            *arrow*
            :rotation-axis t
            :debug-view nil)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)))

;; This function is almost reverse motion of (move-rarm-to-ladle-at-shelf)
;; PR2の右手がladle-at-shelfの姿勢になっていることが前提
(defun rarm-middle-pose-from-ladle-at-shelf ()
  (let (av avs tms)
    (dotimes (i 28)
      (send *pr2* :rarm :move-end-pos #f(10 0 0) :world :use-torso nil)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))

  (let (av1 av2 av3 tmp-coords)
    (setq av1 (send *pr2* :rarm :move-end-pos #f(50 0 0) :world :use-torso nil))
    (setq tmp-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
    (send tmp-coords :translate #f(50 0 -50) :world)
    (setq av2 (send *pr2* :rarm :inverse-kinematics tmp-coords
                    :use-torso nil :rotation-axis :z))
    (send tmp-coords :translate #f(50 0 -100) :world)
    (send tmp-coords :rotate (deg2rad 90) :z)
    (setq av3 (send *pr2* :rarm :inverse-kinematics tmp-coords
                    :use-torso nil :rotation-axis :nil))
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3)
          (list 1000 1000 3000))
    (send *ri* :wait-interpolation))

  (rarm-middle-pose)
  )

;; おたまを棚の中に置く
(defun put-ladle-at-shelf ()
  (move-rarm-to-ladle-at-shelf)

  ;; おたまが棚に当たるまで腕を下ろす
  (send *arrow* :translate #f(0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
  (send *pr2* :rarm :move-end-pos #f(0 0 10) :world :use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *viewer* :draw-objects)
  (send *ri* :wait-interpolation)

  ;;グリッパを微妙に開く
  (let ((tmp-gripper (* 0.001 (send *ri* :state :gripper :rarm :position))))
    (send *ri* :move-gripper :rarm (+ 0.025 tmp-gripper)))

  (send *pr2* :rarm :end-coords :dissoc *ladle*)
  (send *arrow* :translate #f(60 0 30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(60 0 -30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 中間姿勢になる
  (rarm-middle-pose)
  )

(defun reset-rarm ()
  (let ((rav0 #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
        (rav1 #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
        (rav2 #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
        avs)
    (dolist (rav (list rav0 rav1 rav2))
      (send *pr2* :rarm :angle-vector rav)
      (setq avs (append avs (list (send *pr2* :angle-vector)))))
    (send *ri* :angle-vector-sequence
          avs
          (list 3000 3000 2000))
    (send *viewer* :draw-objects)
    (send *ri* :wait-interpolation)))

(defun up-rarm ()
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 50) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

;; grasp-codes
(defun grasp-tool-on-table ()
  ;; arrow　はセットしてある前提
  (send *ri* :stop-grasp :rarm :wait t)

  (send *arrow* :translate #f(50 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(-50 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm :wait t)
  )

(defun grasp-ladle ()
  (pre-ik-pose)
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (up-rarm)

  ;;掴む位置
  (send *arrow* :newcoords (send *ladle* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (send *ri* :stop-grasp :rarm :wait t)

  ;; つかむ
  (grasp-tool-on-table)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 50) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 待機姿勢になる
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun grasp-ladle-with-fail-detection ()
  (let ((tmp-coords (send *ladle* :copy-worldcoords)))
    (grasp-ladle)

    ;; 失敗を検知する
    (grasp-fail-detection :g-thre 10)
    (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (if (not (eql (send *ladle* :parent) nil))
          (send (send *ladle* :parent) :dissoc *ladle*))
        (send *ladle* :newcoords tmp-coords)
        (grasp-ladle)
        (grasp-fail-detection :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
          (progn
            (send *ri* :speak-jp "おたまの把持が失敗しました，渡す準備をしてokと合図をして下さい")
            (receive-with-dialogue)
            (send *ri* :start-grasp :rarm :wait t)
            ))
        ))
    ))

(defun set-ladle ()
  (if (not (eql (send *ladle* :parent) nil))
      (send (send *ladle* :parent) :dissoc *ladle*))
  (let (local-cds)
    (setq local-cds (make-cascoords))
    (send local-cds :newcoords (send *ladle* :copy-worldcoords))
    (send local-cds :assoc *ladle*)
    (send local-cds :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send local-cds :dissoc *ladle*)
    )
  (send *ladle* :rotate (deg2rad 90) :x)
  (send *viewer* :draw-objects)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  )


(defun arrange-motion-set ()
  ;; copy from (transfer-curry-set) in arrangement-codes.l
  (send *arrow* :newcoords (make-coords :pos (float-vector 504.422 8343.425 30917.724) :rpy (float-vector -1.367 0.118 -0.067)))
  (pre-ik-pose)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (arrange-motion) ~%")
  )

(defun arrange-motion ()
  ;; copy from (transfer-curry) in arrangement-codes.l
  (send *arrow* :newcoords (send (elt (send *ladle* :attention) 1) :copy-worldcoords))

  (dotimes (i 9)
    (send *arrow* :rotate (deg2rad 10) :y :world)
    (send *pr2* :rarm :inverse-kinematics
          (send *arrow* :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (warn "please run (have-the-plate-set-with-dialgue) ~%")
  )

(defun after-arrange-motion ()
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (let (av avs tms)
    (dotimes (i 3)
      (send *pr2* :rarm :move-end-pos #f(0 0 -40) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400)))

      (send *pr2* :rarm :move-end-pos #f(0 0 40) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))

  ;; avoid-pose的な姿勢になる．
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8528.539 31050.217) :rpy (float-vector -3.14 0.18 3.056)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun now-open-shelf-with-fail-detection ()
  (let ((tmp-coords (send *shelf-door* :copy-worldcoords)))
    (open-shelf-set)

    ;; 失敗を検知する
    (grasp-fail-detection :g-thre 10)
    (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (if (not (eql (send *shelf-door* :parent) nil))
          (send (send *shelf-door* :parent) :dissoc *shelf-door*))
        (send *shelf-door* :newcoords tmp-coords)
        (open-shelf-set)
        (grasp-fail-detection :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
          (progn
            (rarm-middle-pose) ;; 中間姿勢になる
            (send *ri* :speak-jp "棚の取手を掴むのに失敗しました，棚を開けてokと合図をして下さい")
            (ok-wait)
            (return-from now-open-shelf-with-fail-detection)))
        )))

    (open-shelf)
    (after-open)
  )

;; PR2がデモ前におたまなどを棚の中にセットするための関数
;; (now-set-from-shelf-with-failure-detection)の前に呼ぶ
(defun now-set-in-shelf-with-dialogue-and-fail-detection ()
  ;; 棚の中におたまとヘラをセットする．

  ;; 棚の扉を開ける
  (look-at-shelf)
  ;; Close *shelf-door* model
  (send *shelf-door* :angle-vector #f(0))
  (now-open-shelf-with-fail-detection)

  ;; おたまを人から受け取り、棚の中にセットする
  (set-head)
  (receive-ladle-with-dialogue)
  (put-ladle-at-shelf)

  ;; 棚の扉を閉める
  (close-shelf-set)
  (close-shelf)
  (after-close)

  (set-head)
  )

;; PR2がデモ本番に、棚を開けて道具を取り出してセットする関数
;; (now-set-in-shelf-with-dialogue-and-fail-detection)の後で呼ぶ
(defun now-set-from-shelf-with-failure-detection ()
  ;; 棚の扉を開ける
  (look-at-shelf)
  ;; Close *shelf-door* model
  (send *shelf-door* :angle-vector #f(0))
  (now-open-shelf-with-fail-detection)

  ;; おたまを棚から取り出し、キッチンにセットする
  (rarm-middle-pose)
  (grasp-ladle-at-shelf-with-fail-detection)
  (rarm-middle-pose-from-ladle-at-shelf)
  (put-ladle-at-kitchen)

  ;; 棚の扉を閉める
  (close-shelf-set)
  (close-shelf)
  (after-close)

  (set-head)
  )

;; Motions with interaction
(defun now-set-ladle-at-arrange-and-plate-with-dialogue ()
  ;; おたまを受け取る
  (set-pose)
  (up-rarm)
  (receive-ladle-with-dialogue)

  ;; 盛り付ける動作をして皿を人にセットしてもらう
  (arrange-motion-set)
  (arrange-motion)
  (have-the-plate-set-with-dialgue)
  (after-arrange-motion)

  ;; おたまをセットする
  (put-ladle-at-arrange)
  (reset-rarm)

  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)
  )

;; スープを注ぐ
(defun pour-soup ()
  ;; 盛り付けの位置に移動する
  ;; (move-to-arrange-with-speech-before-and-before-with-rec)

  ;; おたまを掴む
  (update-pos)
  (set-pose)
  (grasp-ladle-with-fail-detection)

  ;; 鍋を掴む
  (grasp-pot-from-below-model-with-fail-detection)

  ;; 止められるまでお湯を注ぐ
  (let ((pour-flag t))
    (until (not pour-flag)
           (now-arrange-curry)
           (send *ri* :speak-jp "まだお湯が必要ですか？" :wait t)
           (setq pour-flag (listen-speech "はい"))
           ))

  ;; 鍋とおたまを手から離す
  (release-pot-from-below-model)
  (put-ladle-at-arrange)
  (reset-rarm)
  (set-pose)

  ;;完成！！
  (send *ri* :speak-jp "お湯を注ぐ工程を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)

  (unix:sleep 2)
  )

;; スープを冷ます
(defun cool-soup ()
  (look-at-dish)
  (ros::spin-once)
  (let ((max-temp-msg (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000))
        max-temp)
    ;; If PR2 have thermal sensor
    (when max-temp-msg
      (setq max-temp (send max-temp-msg :data))
      (while (> max-temp 65)
        (send *ri* :speak-jp "まだスープが熱いです。もう少しお待ちください。" :wait t)
        (unix:sleep 15)
        (setq max-temp
              (send (one-shot-subscribe "/thermal/max_temp" std_msgs::Int16 :timeout 1000) :data))
        )))
  (send *ri* :speak-jp "スープが完成しました" :wait t)
  )

;; correct position of IH by touch and wrench
;; BE CAREFUL that this function changes *pr2* coords
(defun correct-pr2-coords-at-kitchen ()
  (send *ri* :speak-jp "ドアの取っ手の位置を触って確認します")
  (set-pose)
  (look-at-shelf)
  (update-pos)

  ;; 棚のノブの位置に手を伸ばす前のPre pose
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
  (send *arrow* :translate (float-vector 100 130 0) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; Detect *shelf-door-coords* by touch
  ;; Detect x direction for the next z ditection
  (let (av avs tms)
    (dotimes (i 20)
      (send *arrow* :translate (float-vector -10 0 0) :world)
      (send *pr2* :rarm :inverse-kinematics
            *arrow*
            :rotation-axis t
            :debug-view nil)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :x :threshold -15)
  (send *ri* :angle-vector-sequence
        (list (send *pr2* :rarm :move-end-pos #f(100 0 -100) :world)
              (send *pr2* :rarm :move-end-pos #f(-150 0 0) :world))
        (list 2000 2000))
  (send *ri* :wait-interpolation)

  ;; Detect z direction
  (let (av avs tms)
    (dotimes (i 10)
      (send *pr2* :rarm :move-end-pos #f(0 0 10) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :z :threshold -10)
  (send *ri* :angle-vector-sequence
        (list (send *pr2* :rarm :move-end-pos #f(150 0 -50) :world)
              (send *pr2* :rarm :move-end-pos #f(0 0 110) :world))
        (list 2000 2000))
  (send *ri* :wait-interpolation)

  ;; Detect x direction for the next y ditection
  (let (av avs tms)
    (dotimes (i 10)
      (send *pr2* :rarm :move-end-pos #f(-10 0 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :x :threshold -15)
  (send *ri* :angle-vector
        (send *pr2* :rarm :move-end-pos #f(15 0 0) :world)
        2000)
  (send *ri* :wait-interpolation)

  ;; Detect y direction
  (let (av avs tms)
    (dotimes (i 30)
      (send *pr2* :rarm :move-end-pos #f(0 -10 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :y :threshold -6)

  ;; Calculate *shelf-door-coords*
  (unix:usleep (* 500 1000))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((shelf-door-coords (copy-object *shelf-door-coords*))
         (fingertip (send (send *pr2* :rarm :end-coords :copy-worldcoords)
                          :translate #f(20 0 0) :local))
         (new-shelf-door-handle (send fingertip :translate #f(0 -20 0) :world))
         diff)
    ;; Update *pr2* worldcoords
    (setq diff (v- (car (send *shelf-door* :handle :worldpos))
                   (send new-shelf-door-handle :worldpos)))
    (send *pr2* :translate diff :world)
    (format t "*pr2* coords is updated to ~A~%"
            (send *pr2* :worldcoords)))

  ;; Reset PR2 rarm
  (send *pr2* :rarm :move-end-pos #f(100 0 -100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
)

(defun shelf-check ()
  (send *ri* :speak-jp "ドアの開閉が出来るかチェックします")
  (look-at-shelf)
  ;; Close *shelf-door* model
  (send *shelf-door* :angle-vector #f(0))

  ;; Check if PR2 can handle shelf
  (open-shelf-set)
  (open-shelf)
  (after-open)
  (close-shelf-set)
  (close-shelf)
  (after-close)
  )
