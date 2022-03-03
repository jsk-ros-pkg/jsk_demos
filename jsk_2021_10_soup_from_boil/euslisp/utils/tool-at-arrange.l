;; TODO: remove loading move-to-kitchen-with-map.l
(load "package://jsk_2021_10_soup_from_boil/euslisp/utils/move-to-kitchen-with-map.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/knife-new-attention.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/potato-peeler.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/spatula.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/cooking-pot-73b2.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/shelf-door.l")

(defun tool-init ()
  ;; set tools
  (setq *knife* (knife))
  (send *knife* :newcoords (send *knife-coords* :copy-worldcoords))
  (setq *peeler* (peeler))
  (send *peeler* :newcoords (send *peeler-coords* :copy-worldcoords))
  (setq *potato-peeler* (peeler))
  (send *potato-peeler* :newcoords (send *potato-peeler-coords* :copy-worldcoords))
  (setq *spatula* (spatula))
  (send *spatula* :newcoords (send *spatula-coords-1* :copy-worldcoords))
  (setq *ladle* (ladle))
  (send *ladle* :newcoords (send *ladle-coords-1* :copy-worldcoords))
  (setq *pot* (cooking-pot))
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))
  ;; set equipment
  (setq *shelf-door* (shelf-door))
  (send *shelf-door* :newcoords (send *shelf-door-coords* :copy-worldcoords))
  (setq *shelf-look-coords* (send (send *shelf-door* :handle-handle0) :copy-worldcoords))
  ;; arrange original
  (setq *ladle-coords-a* (make-coords :pos (float-vector 560 8700 30860) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，盛り付け時

  (move-to-arrange-irt)
  (objects (list *pr2* *peeler* *knife* *potato-peeler* *scene* *spatula* *ladle* *shelf-door*))
  (update-view)
  )

(tool-init)

;; add for sense
(setq *x-thre* -15)

(ros::roseus "stove_rec_test" :anonymous t)
(setq *topic-name* "/right_endeffector/wrench")
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

;; set codes
(warn "~%  If you use real robot, run (update-pos) ~%")
(warn "please run (now-set-tools) or (set-knife) ~%")

(defun set-shelf-door ()
  (if (not (eql (send *shelf-door* :parent) nil))
      (send (send *shelf-door* :parent) :dissoc *shelf-door*))
  (let (local-cds)
    (setq local-cds (make-cascoords))
    (send local-cds :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
    (send local-cds :assoc *shelf-door*)
    (send local-cds :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send local-cds :dissoc *shelf-door*)
    )
  (send *viewer* :draw-objects)
  )

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

  (warn "please run (put-ladle-1) or (put-ladle-0) ~%")
  )

(defun put-ladle-1 ()
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (send *ladle-coords-1* :copy-worldcoords))
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

(defun put-ladle-a ()  
  (send *arrow* :newcoords (send *ladle-coords-a* :copy-worldcoords))
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
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun open-shelf-set ()
  ;; 中間姿勢になる
  (rarm-middle-pose)
  
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
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 10)
    (send *arrow* :translate (float-vector -10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view nil)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 600)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  
  (ros::unsubscribe "/right_endeffector/wrench")
  (setq *topic-name* "/right_endeffector/wrench")
  (setq *cb-flag* nil)
  (ros::subscribe "/right_endeffector/wrench" geometry_msgs::WrenchStamped #'wrench-x-cb)
  
  (while (some #'identity (send *ri* :wait-interpolation nil 0.01))
    (when (progn
            (ros::spin-once)
            (if *cb-flag* t)
            )
      (send *ri* :cancel-angle-vector)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (warn "cancel!!! ~%")
      )
    )
  
  (ros::unsubscribe "/right_endeffector/wrench")
  
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
  
  (set-shelf-door)
  (warn "please run (open-shelf) ~%")
  )

(defun open-shelf ()
  (let ((i 0)   ;; open
        (handle-coords (make-coords))
        (av)
        (tms (list))
        (avs (list)))
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
    (send *ri* :wait-interpolation)
    )
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
  
  (warn "please run (grasp-spatula-0) ~%")
  )

(defun close-shelf-set ()
  ;; (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  ;; (send *pr2* :rarm :angle-vector *rav0*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)

  (send *shelf-door* :angle-vector (float-vector 100))

  (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -70) :y)
  (send *arrow* :translate #f(-60 0 60))

  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(60 0 0))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(0 0 -60))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (close-shelf) ~%")
  )

(defun close-shelf ()
  (let ((i 100)   ;; close
        (handle-coords (make-coords))
        (av)
        (tms (list))
        (avs (list)))
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
    (send *ri* :wait-interpolation)
    )
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



(defun grasp-tool-on-shelf ()
  ;; arrow　はセットしてある前提

  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)

  (send *arrow* :translate #f(120 0 0) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(-60 0 30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(-60 0 -30) :world)  
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  

  (send *ri* :start-grasp :rarm :wait t)
  )

(defun grasp-spatula-0 ()  
  (if (not (eql (send *spatula* :parent) nil))
      (send (send *spatula* :parent) :dissoc *spatula*))
  (send *spatula* :newcoords (send *spatula-coords-0* :copy-worldcoords))
  (send *arrow* :newcoords (send *spatula* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (grasp-tool-on-shelf)
  (send (send *pr2* :rarm :end-coords) :assoc *spatula*)
  
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(30 0 30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手をスライドさせる
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 35)
    (send *arrow* :translate (float-vector 10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view nil)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)

  ;; 手を下げる
  (send *pr2* :rarm :move-end-pos #f(0 0 -60) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (put-spatula-1) ~%")
  )

(defun grasp-spatula-0-with-fail-detection ()
  (grasp-spatula-0)
  
  ;; 失敗を検知する
  (grasp-fail-detection :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-spatula-0)
        (grasp-fail-detection :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "ヘラの把持が失敗しました，渡す準備をしてokと合図をして下さい")
              (receive-with-dialogue)
              (send *ri* :start-grasp :rarm :wait t)
              ))
        ))
  )


(defun grasp-ladle-0 ()
  (if (not (eql (send *ladle* :parent) nil))
      (send (send *ladle* :parent) :dissoc *ladle*))
  (send *ladle* :newcoords (send *ladle-coords-0* :copy-worldcoords))
  (send *arrow* :newcoords (send *ladle* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (grasp-tool-on-shelf)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(20 0 20) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手をスライドさせる
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 35)
    (send *arrow* :translate (float-vector 10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view nil)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  ;; 手を下げる
  (send *pr2* :rarm :move-end-pos #f(0 0 -60) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (put-ladle-1) ~%")  
  )

(defun grasp-ladle-0-with-fail-detection ()
  (grasp-ladle-0)
  
  ;; 失敗を検知する
  (grasp-fail-detection :g-thre 10)
  (if (equal *fail-flag* t) ;; 一度失敗したらもう一度挑戦する
      (progn
        (grasp-ladle-0)
        (grasp-fail-detection :g-thre 10)
        (if (equal *fail-flag* t) ;; 二度失敗したら人に助けてもらう
            (progn
              (send *ri* :speak-jp "おたまの把持が失敗しました，渡す準備をしてokと合図をして下さい")
              (receive-with-dialogue)
              (send *ri* :start-grasp :rarm :wait t)
              ))
        ))
  )

(defun put-spatula-0 ()
  ;; 中間姿勢になる
  (rarm-middle-pose)

  ;; spatulaを置く位置
  (send *arrow* :newcoords (send *spatula-coords-0* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  (send *arrow* :translate #f(30 0 30) :world)
  (send *arrow* :translate (float-vector 350 0 0) :world)
  (send *arrow* :translate #f(0 0 -100) :world)

  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手をスライドさせる
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 35)
    (send *arrow* :translate (float-vector -10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view nil)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(-30 0 -30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;グリッパを微妙に開く
  (release-object :g-num 0.025)
  ;; (setq tmp-gripper (* 0.001 (send *ri* :state :gripper :rarm :position)))
  ;; (send *ri* :move-gripper :rarm (+ 0.025 tmp-gripper))

  ;; (send *ri* :move-gripper :rarm 0.07)

  (send *pr2* :rarm :end-coords :dissoc *spatula*)

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

(defun put-ladle-0 ()
  ;; 中間姿勢になる
  (rarm-middle-pose)
  
  ;; ladleを置く位置
  (send *arrow* :newcoords (send *ladle-coords-0* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  (send *arrow* :translate #f(30 0 30) :world)
  (send *arrow* :translate (float-vector 350 0 0) :world)
  (send *arrow* :translate #f(0 0 -100) :world)

  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 手をスライドさせる
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 35)
    (send *arrow* :translate (float-vector -10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view nil)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(-30 0 -30) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;グリッパを微妙に開く
  (setq tmp-gripper (* 0.001 (send *ri* :state :gripper :rarm :position)))
  (send *ri* :move-gripper :rarm (+ 0.025 tmp-gripper))
  ;; (send *ri* :move-gripper :rarm 0.07)
  
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



(defun reset-rarm () ;; TODO (copy from arrangement-codes.l)
  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)  
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq *rav2* #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  (send *pr2* :rarm :angle-vector *rav2*)
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 800.186 8689.105 30866.677) :rpy (float-vector -2.341 -0.222 -0.482)))
  ;; (send *pr2* :rarm :inverse-kinematics
  ;;       (send *arrow* :copy-worldcoords)
  ;;       :rotation-axis t
  ;;       :debug-view *debug-flag*)  
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq *rav3* #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

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
  ;; (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  ;; (send *pr2* :rarm :angle-vector *rav0*)
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8828.539 31050.217) :rpy (float-vector -3.14 0.18 -0.086)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun grasp-ladle-with-fail-detection ()
  (setq tmp-coords (send *ladle* :copy-worldcoords))
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
  )

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

(defun now-set-ladle-1 ()
  (up-rarm)
  (receive-ladle)
  (put-ladle-1)
  )

(defun now-set-ladle-0-with-dialogue ()
  (up-rarm)
  (receive-ladle-with-dialogue)
  (put-ladle-0)
  )

(defun now-open-shelf-with-fail-detection ()
  (setq tmp-coords (send *shelf-door* :copy-worldcoords))
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
        ))
  
  (open-shelf)
  (after-open)
  )

(defun now-set-in-shelf-with-dialogue-and-fail-detection ()
  ;; 棚の中におたまとヘラをセットする．

  ;; 棚の扉を開ける
  (look-at-shelf)
  (now-open-shelf-with-fail-detection)

  ;; ヘラをセットする
  (set-head)
  (receive-spatula-with-dialogue)
  (look-at-shelf)
  (put-spatula-0)

  ;; おたまをセットする
  (set-head)
  (receive-ladle-with-dialogue)
  (look-at-shelf)
  (put-ladle-0)
  
  ;; 棚の扉を閉める
  (close-shelf-set)
  (close-shelf)
  (after-close)
  
  (set-head)
  )

(defun now-set-from-shelf-with-failure-detection ()
  ;; 棚の扉を開ける
  (look-at-shelf)
  (now-open-shelf-with-fail-detection)

  ;; ヘラをセットする
  (rarm-middle-pose)
  (grasp-spatula-0-with-fail-detection)
  (rarm-middle-pose)
  (put-spatula-1)

  ;; おたまをセットする
  (rarm-middle-pose)
  (grasp-ladle-0-with-fail-detection)
  (rarm-middle-pose)
  (put-ladle-1)

  ;; 棚の扉を閉める
  (close-shelf-set)
  (close-shelf)
  (after-close)

  (set-head)
  )

;; 盛り付けの位置での プログラム
(defun now-set-ladle-a-with-dialogue ()
  ;; おたまを受け取ってセットする
  (up-rarm)
  (receive-ladle-with-dialogue)
  (put-ladle-a)
  (reset-rarm)
  )
