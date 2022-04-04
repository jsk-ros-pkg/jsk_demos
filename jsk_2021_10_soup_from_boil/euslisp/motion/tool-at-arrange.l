(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/knife-new-attention.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/potato-peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/shelf-door.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/test/pr2-random-coords.l")

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
  (setq *shelf-door* (shelf-door))
  (send *shelf-door* :newcoords (send *shelf-door-coords* :copy-worldcoords))
  (setq *shelf-look-coords* (send (send *shelf-door* :handle-handle0) :copy-worldcoords))
  ;; arrange original
  (setq *ladle-coords-at-arrange* (make-coords :pos (float-vector 560 8700 30860) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，盛り付け時

  (move-on-irt :arrange)
  (objects (list *pr2* *peeler* *knife* *potato-peeler* *scene* *ladle* *shelf-door*))
  (update-view)
  )

(tool-init)

;; set codes
(warn "~%  If you use real robot, run (update-pos) ~%")

(defun look-at-shelf ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send *shelf-look-coords* :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (update-view)
  )

(defun receive-ladle-with-dialogue ()
  (rarm-middle-pose)
  (set-ladle)
  ;; (arrange-motion-set)
  (transfer-curry-set);; from arrangement-at-arrange.l
  (send *ri* :speak-jp "おたまを渡す準備をして，okと合図をして下さい")
  (receive-with-dialogue)
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  (set-ladle)

  (warn "please run (put-ladle-kitchen) or (put-ladle-at-shelf) ~%")
  )

;; rarm-middle-poseから、おたまをキッチンの上にセットする
(defun put-ladle-at-kitchen ()
  ;; キッチン台に手が当たるまで腕を下げる
  (let (av1 av2 (ik-target (send *ladle-coords-at-kitchen* :copy-worldcoords)))
    (send ik-target :rotate (deg2rad -90) :x)
    (send ik-target :translate #f(0 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
    (update-view)
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :rarm :move-end-pos #f(0 0 -150) :world :use-torso nil)
    (update-view)
    (setq av2 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence
          (list av1 av2)
          (list 3000 3000))
    (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
    (send *viewer* :draw-objects))

  ;; おたまを置く
  (send *ri* :move-gripper :rarm 0.08)
  (send *pr2* :rarm :end-coords :dissoc *ladle*)

  ;; 手を上げる
  (let (av1 av2 (ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords)))
    (send ik-target :translate #f(0 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av1 (send *pr2* :angle-vector))
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(60 100 170) :world)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis :z
                    :debug-view *debug-flag*)
    (update-view)
    (setq av2 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence
          (list av1 av2)
          (list 2000 2000))
    (send *ri* :wait-interpolation))
  )

;; おたまをキッチンの上に置く
(defun put-ladle-at-arrange ()
  (let ((ik-target (send *ladle-coords-at-arrange* :copy-worldcoords)))
    (send ik-target :rotate (deg2rad -90) :x)

    (send ik-target :translate #f(60 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send ik-target :translate #f(-60 0 -100) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    ;; (send *ri* :stop-grasp :rarm :wait t)
    (send *ri* :move-gripper :rarm 0.08)
    (send *pr2* :rarm :end-coords :dissoc *ladle*)

    ;; 手を上げる
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(0 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    ;; 手を上げる
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(60 100 170) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))
  )

(defun rarm-middle-pose ()
  (let ((ik-target (make-coords :pos (float-vector 570 8500 31100) :rpy (float-vector 3.14 0.0 0.0))))
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 325)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          ;; :rotation-axis t
          :rotation-axis :z ;; this change may affect many demos...
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))
  (update-view)
  )

(defun touch-shelf-knob ()
  ;; Mainly copied from (push-stove-button) in ih-at-arrange.l
  ;; The difference is left and right
  (let (avs tms)
    (dotimes (i 15)
      (send *pr2* :rarm :move-end-pos #f(-10 0 0) :world)
      (update-view)
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
  ;; *shelf-door*は閉じている状態を仮定
  (send *shelf-door* :angle-vector #f(0))
  (let ((ik-target (send (car (send *shelf-door* :handle)) :copy-worldcoords)))
    (send ik-target :translate (float-vector 100 0 -100) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send ik-target :translate (float-vector 0 0 100) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;; ノブのところまで手を伸ばす
    (touch-shelf-knob)

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))

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
      (update-view)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 500)))

      (setq i (+ i 10))
      )
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))
  (warn "please run (after-open) ~%")
  )

(defun after-open ()
  (send *ri* :stop-grasp :rarm :wait t)

  (let (av1 av2 av3 av4 (shelf-handle (car (send *shelf-door* :handle))) ik-target)
    (send *pr2* :rarm :move-end-pos #f(0 20 -80) :world)
    (update-view)
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :rarm :move-end-pos #f(0 -100 -100) :world)
    (update-view)
    (setq av2 (send *pr2* :angle-vector))
    ;; *shelf-door*は開いている状態を仮定
    (send *shelf-door* :angle-vector #f(90))
    (setq ik-target (send shelf-handle :copy-worldcoords))
    (send ik-target :rotate (rad2deg 90) :y)
    (send ik-target :translate #f(0 -100 0) :world)
    (send *pr2* :rarm :inverse-kinematics ik-target)
    (update-view)
    (setq av3 (send *pr2* :angle-vector))
    (send ik-target :translate #f(0 80 0) :world)
    (send *pr2* :rarm :inverse-kinematics ik-target)
    (update-view)
    (setq av4 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3 av4)
          (list 1000 1000 2000 3000))
    (send *ri* :wait-interpolation))
  (update-view)

  ;; 中間姿勢になる
  (rarm-middle-pose)

  (warn "please run (grasp-spatula-at-shelf) ~%")
  )

(defun close-shelf-set ()
  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)
  ;; ドアを閉める前の姿勢になる
  (look-at-shelf)
  (let (av1 av2 av3 av4 (ik-target (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086))))
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 325)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis :z
                    :debug-view *debug-flag*)
    (update-view)
    (setq av1 (send *pr2* :angle-vector))
    (send *shelf-door* :angle-vector (float-vector 100))
    (setq ik-target (send (car (send *shelf-door* :handle)) :copy-worldcoords))
    (send ik-target :rotate (deg2rad -70) :y)
    (send ik-target :translate #f(-150 0 150))
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av2 (send *pr2* :angle-vector))
    (send ik-target :translate (float-vector 150 0 0))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (setq av3 (send *pr2* :angle-vector))
    (send ik-target :translate (float-vector 0 0 -150))
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av4 (send *pr2* :angle-vector))
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
      (update-view)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 800)))

      (setq i (- i 10))
      )
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))
  (warn "please run (after-close) ~%")
  )

(defun after-close ()
  (let ((ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords)))
    (send ik-target :translate #f(80 0 -80) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation))
  (update-view)

  ;; 中間姿勢になる
  (rarm-middle-pose)

  (reset-rarm)
  )

;; 棚の中のおたまを掴む
(defun grasp-ladle-at-shelf ()
  (if (not (eql (send *ladle* :parent) nil))
      (send (send *ladle* :parent) :dissoc *ladle*))
  ;; (send *ladle* :newcoords (send *ladle-coords-at-shelf* :copy-worldcoords))
  (let ((ik-target (send *ladle* :copy-worldcoords)) av avs tms)
    ;; 棚の中に手を伸ばす
    (send *ri* :move-gripper :rarm 0.08 :wait t)
    (move-rarm-to-ladle-at-shelf)
    (send *ri* :stop-grasp :rarm :wait t)
    (send ik-target :translate #f(0 0 50) :world)
    (send *pr2* :rarm :inverse-kinematics
          ik-target
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; 右手をおたまの持ち手の高さに合わせる
    (dotimes (i 10)
      (send ik-target :translate #f(0 0 -10) :world)
      (send *pr2* :rarm :inverse-kinematics
                     (send ik-target :copy-worldcoords)
                     :rotation-axis t
                     :debug-view *debug-flag*)
      (update-view)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms)
    (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
    (send *pr2* :rarm :move-end-pos #f(0 0 20) :world :use-torso nil)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *viewer* :draw-objects)
    (send *ri* :wait-interpolation))

  ;; おたまを掴む
  (send *ri* :start-grasp :rarm :wait t)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  (warn "please run (put-ladle-at-kitchen) ~%")
  )

(defun grasp-ladle-at-shelf-with-fail-detection ()
  (grasp-ladle-at-shelf)

  ;; 失敗を検知する
  (grasp-fail-detection :rarm :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
    (progn
        (grasp-ladle-at-shelf)
        (grasp-fail-detection :rarm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "おたまの把持が失敗しました，渡す準備をしてokと合図をして下さい")
              (receive-with-dialogue)
              (send *ri* :start-grasp :rarm :wait t)
              ))
        ))
  )

(defun move-rarm-to-ladle-at-shelf (&key (reverse nil))
  ;; ladleを置く位置
  ;; 中間姿勢を多めに入れないと、IKがすぐに失敗する
  (let (av0 av1 av2 av3 av4 ik-target av avs tms)
    ;; Pre pose
    (setq ik-target (make-coords :pos (float-vector 570 8500 31100) :rpy (float-vector 3.14 0.0 0.0)))
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 325)
    (send *pr2* :head :look-at (send *shelf-look-coords* :worldpos))
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis :z
                    :debug-view *debug-flag*)
    (update-view)
    (setq av0 (send *pr2* :angle-vector))
    ;; Set av1 ~ av4 おたまの先端を棚に入れるための姿勢
    (send (send *pr2* :rarm :end-coords) :dissoc *ladle*)
    (send *ladle* :newcoords (send *pr2* :rarm :end-coords :copy-worldcoords))
    (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
    (setq ik-target (send *ladle-coords-at-shelf* :copy-worldcoords))
    (send ik-target :rotate (deg2rad 45) :z)
    (send ik-target :rotate (deg2rad 45) :y)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :translate #f(200 0 50) :world)
                    :move-target (car (send *ladle* :attention))
                    :rotation-axis :t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av1 (send *pr2* :angle-vector))

    (send ik-target :rotate (deg2rad -30) :y)
    (send ik-target :rotate (deg2rad -30) :z)
    (send ik-target :translate #f(-50 0 50) :world)
    (send *pr2* :rarm :inverse-kinematics
                    ik-target
                    :move-target (car (send *ladle* :attention))
                    :rotation-axis :z
                    :debug-view *debug-flag*)
    (update-view)
    (setq av2 (send *pr2* :angle-vector))
    (setq ik-target (send *ladle-coords-at-shelf* :copy-worldcoords))
    (send ik-target :rotate (deg2rad -90) :x)
    (send ik-target :translate #f(200 70 30) :world)
    (send ik-target :rotate (deg2rad -20) :z)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av3 (send *pr2* :angle-vector))
    (send ik-target :rotate (deg2rad 20) :z)
    (send ik-target :translate #f(0 0 -10) :world)
    (send *pr2* :rarm :inverse-kinematics
                    (send ik-target :copy-worldcoords)
                    :rotation-axis t
                    :debug-view *debug-flag*)
    (update-view)
    (setq av4 (send *pr2* :angle-vector))
    (send *viewer* :draw-objects)
    (setq avs (list av0 av1 av2 av3 av4))
    (setq tms (list 2000 2000 2000 2000 1000))
    ;; 手を棚の奥にスライドさせる
    (dotimes (i 25)
      (send ik-target :translate (float-vector -10 0 0) :world)
      (send *pr2* :rarm :inverse-kinematics
            ik-target
            :rotation-axis t
            :debug-view nil)
      (update-view)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    ;; 実機を動かす。:reverseのキーワード引数がtになると、動作を逆再生する。
    (if reverse
      (send *ri* :angle-vector-sequence (reverse avs) (reverse tms))
      (send *ri* :angle-vector-sequence avs tms))
    (send *ri* :wait-interpolation)))

;; おたまを棚の中に置く
(defun put-ladle-at-shelf ()
  (move-rarm-to-ladle-at-shelf)

  ;; おたまが棚に当たるまで腕を下ろす
  (send *pr2* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (cancel-motion-when-touch :arm :rarm :direction :z :threshold 15)
  (update-view)

  ;;グリッパを微妙に開く
  (if (send *ri* :simulation-modep)
    (send *ri* :move-gripper :rarm 0.08 :wait t)
    (let ((tmp-gripper (* 0.001 (send *ri* :state :gripper :rarm :position))))
      (send *ri* :move-gripper :rarm (+ 0.025 tmp-gripper))))

  (send *pr2* :rarm :end-coords :dissoc *ladle*)
  (send *ladle* :newcoords (send *pr2* :rarm :end-coords :copy-worldcoords))
  (send *pr2* :rarm :move-end-pos #f(60 0 30) :world)
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :move-end-pos #f(60 0 -30) :world)
  (update-view)
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
      (update-view)
      (setq avs (append avs (list (send *pr2* :angle-vector)))))
    (send *ri* :angle-vector-sequence
          avs
          (list 3000 3000 2000))
    (send *ri* :wait-interpolation)))

;; grasp-codes
(defun grasp-tool-on-table (grasp-target)
  (send *ri* :stop-grasp :rarm :wait t)

  (let ((ik-target (send (send grasp-target :copy-worldcoords) :translate #f(50 0 100) :world)))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :inverse-kinematics
          (send grasp-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation))

  (send *ri* :start-grasp :rarm :wait t)
  )

(defun grasp-ladle ()
  (pre-ik-pose)
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)

  ;;掴む位置
  (let ((ik-target (send *ladle* :copy-worldcoords)))
    (send ik-target :rotate (deg2rad -90) :x)

    ;; つかむ
    (grasp-tool-on-table ik-target)
    (send (send *pr2* :rarm :end-coords) :assoc *ladle*)

    ;; 手を上げる
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(0 0 50) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    ;; 手を上げる
    (setq ik-target (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send ik-target :translate #f(40 0 150) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    ;; 待機姿勢になる
    (setq ik-target (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :rotation-axis t
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  )

;; キッチンの上のおたまを把持する
(defun grasp-ladle-with-fail-detection ()
  (let ((tmp-coords (send *ladle* :copy-worldcoords)))
    (grasp-ladle)

    ;; 失敗を検知する
    (grasp-fail-detection :rarm :g-thre 10)
    (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (if (not (eql (send *ladle* :parent) nil))
          (send (send *ladle* :parent) :dissoc *ladle*))
        (send *ladle* :newcoords tmp-coords)
        (grasp-ladle)
        (grasp-fail-detection :rarm :g-thre 10)
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
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  (update-view)
  )

(defun open-shelf-with-fail-detection ()
  (let ((tmp-coords (send *shelf-door* :copy-worldcoords)))
    (open-shelf-set)

    ;; 失敗を検知する
    (grasp-fail-detection :rarm :g-thre 10)
    (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (if (not (eql (send *shelf-door* :parent) nil))
          (send (send *shelf-door* :parent) :dissoc *shelf-door*))
        (send *shelf-door* :newcoords tmp-coords)
        (open-shelf-set)
        (grasp-fail-detection :rarm :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
          (progn
            (rarm-middle-pose) ;; 中間姿勢になる
            (send *ri* :speak-jp "棚の取手を掴むのに失敗しました，棚を開けてokと合図をして下さい")
            (ok-wait)
            (return-from open-shelf-with-fail-detection)))
        )))

    (open-shelf)
    (after-open)
  )

;; PR2がデモ前におたまなどを棚の中にセットするための関数
;; (set-from-shelf-with-failure-detection)の前に呼ぶ
(defun set-in-shelf-with-dialogue-and-fail-detection ()
  ;; 棚の中におたまとヘラをセットする．

  ;; 棚の扉を開ける
  (look-at-shelf)
  ;; Close *shelf-door* model
  (send *shelf-door* :angle-vector #f(0))
  (open-shelf-with-fail-detection)

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
;; (set-in-shelf-with-dialogue-and-fail-detection)の後で呼ぶ
(defun set-from-shelf-with-failure-detection ()
  ;; 棚の扉を開ける
  (look-at-shelf)
  ;; Close *shelf-door* model
  (send *shelf-door* :angle-vector #f(0))
  (open-shelf-with-fail-detection)

  ;; おたまを棚から取り出し、キッチンにセットする
  (rarm-middle-pose)
  (grasp-ladle-at-shelf-with-fail-detection)
  (move-rarm-to-ladle-at-shelf :reverse t)
  (put-ladle-at-kitchen)

  ;; 棚の扉を閉める
  (close-shelf-set)
  (close-shelf)
  (after-close)

  (set-head)
  )

(defun set-ladle-at-kitchen ()
  ;; おたまをセットする
  (put-ladle-at-kitchen)
  (reset-rarm)

  (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)
  )

;; (defun set-ladle-at-arrange ()
;;   ;; おたまをセットする
;;   (put-ladle-at-arrange)
;;   (reset-rarm)

;;   (send *ri* :speak-jp "準備を行いました．確認してokと合図をして下さい" :wait t)
;;   (ok-wait)
;;   )

;; スープを注ぐ
(defun pour-soup ()
  ;; 盛り付けの位置に移動する
  ;; (move-to-arrange-with-speech-before-and-before-with-rec)

  ;; おたまを掴む
  (set-pose)
  (grasp-ladle-with-fail-detection)

  ;; 鍋を掴む
  (grasp-pot-from-below-model-with-fail-detection)

  ;; 止められるまでお湯を注ぐ
  (let ((pour-flag t))
    (until (not pour-flag)
           (arrange-curry);; from arrangement-at-arrange.l
           (send *ri* :speak-jp "まだお湯が必要ですか？" :wait t)
           (if (send *ri* :simulation-modep)
             (setq pour-flag nil)
             (setq pour-flag (listen-speech "はい")))
           ))

  ;; 鍋とおたまを手から離す
  (release-pot-from-below-model)
  (put-ladle-at-arrange)
  (reset-rarm)
  (set-pose)

  ;;完成！！
  (send *ri* :speak-jp "お湯を注ぐ工程を行いました．確認してokと合図をして下さい" :wait t)
  (ok-wait)
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
  (when (send *ri* :simulation-modep)
    (send *pr2* :move-to
          (make-coords :pos #f(1081.642 8408.347 30038.963) :rpy #f(3.11 0.017 -0.009)) :world)
    (return-from correct-pr2-coords-at-kitchen))
  ;; ;; random coords when kinematics simulator
  ;; (if (send *ri* :simulation-modep)
  ;;   (send *pr2* :move-to (send (random-coords (make-coords :pos #f(1050 8500 30000) :rpy (float-vector pi 0 0)) :x 10 :y 10 :theta 5) :copy-worldcoords) :world))
  (send *ri* :speak-jp "ドアの取っ手の位置を触って確認します")
  (set-pose)
  (look-at-shelf)
  (update-pos)

  ;; 棚のノブの位置に手を伸ばす前のPre pose
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
  (send *arrow* :translate (float-vector -50 130 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (update-view)

  ;; Detect z direction
  (let (av avs tms)
    (dotimes (i 15)
      (send *pr2* :rarm :move-end-pos #f(0 0 10) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 200))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :z :threshold -10)
  (setq avs nil)
  (setq tms nil)
  (dolist (move-pos (list #f(150 0 -50) #f(0 0 110)))
    (send *pr2* :rarm :move-end-pos move-pos :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 1000))))
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  (update-view)

  ;; Detect x direction for the next y ditection
  (let (av avs tms)
    (dotimes (i 20)
      (send *pr2* :rarm :move-end-pos #f(-10 0 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 200))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :x :threshold -15)
  (send *ri* :angle-vector
        (send *pr2* :rarm :move-end-pos #f(15 0 0) :world)
        2000)
  (send *ri* :wait-interpolation)
  (update-view)

  ;; Detect y direction
  (let (av avs tms)
    (dotimes (i 30)
      (send *pr2* :rarm :move-end-pos #f(0 -10 0) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :rarm :direction :y :threshold -6)
  (update-view)

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
  (update-view)
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
