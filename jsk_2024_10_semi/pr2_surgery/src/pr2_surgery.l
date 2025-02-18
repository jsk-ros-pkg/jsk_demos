#!/usr/bin/env roseus

#| DESCRIPTION
Okada-sensei seminar
Surgery-method dev3
Last editted Dec 19 by Michi-Tsubaki
GOAL: Trying to make unique motion for stitching sponge!

## Functions
- voice recognition (trriger)
- perception (recognize wound)
- sew sponge 

## For Future
- Dual Pandaでやってみたい．
- チート実験台ではなく，平らな傷口に上面からアプローチしたい
- そのために，class trajを継承して曲がった経路を生成する
|#

;;Load pkgs around PR2 Interface
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") ;;*ri*
(require "package://pr2eus/speak.l") ;;pkg for speaking
;;ros msgs
(ros::load-ros-package "jsk_recognition_msgs")
(ros::roseus-add-msgs "speech_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")


;;ここをかえるだけで調整できる！（外界の変化に合わせて）
;; Drawing Pram
(setq *draw-param-left* 0.2)
(setq *draw-param-right* 0.6)
;; Env Param
(setq *centerx* 700)
(setq *centery* 0)
(setq *centerz* 755)
(setq *deskw* 500)
;; Hampen Param
(setq *target-ypos* 0)
(setq *target-xpos* 0)
(setq *target-zpos* 30)
;; Traj Param
(setq *needle_len* 70)
(setq *interval* 10)
(setq *traj_num* 10)
(setq *traj_len* (* *interval* *traj_num*))
;; Set remain
(setq *remain* 450) ;;450 ;;how long remaining thread?
(setq *diff_remain* 55) ;;55
;; Offset
(setq *rarm_offset* 2)


;; Frag (DO NOT CHANGE)
(setq *start-flag* 0)
(setq *is_first* 1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Environment ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define set-environment method
(defun set-env()
  ;;Set center
  (setq *deskh* *centerz*)

  (setq *center* (make-cube 10 10 10))
  (send *center* :translate (float-vector *centerx* *centery* *centerz*))
  (send *center* :set-color :black)
  (setq *o* (send *center* :copy-worldcoords))

  ;;Set desk
  (setq *desk* (make-cube *deskw* *deskw* *deskh*))
  (send *desk* :translate (float-vector *centerx* *centery* (/ *centerz* 2))) 
  (send *desk* :set-color :brown)

  ;;Set needle
  (setq *needle* (make-cylinder 0.5 *needle_len*))
  (send *needle* :translate (v+ (float-vector -100 20 300) (send *center* :pos)))
  (send *needle* :set-color :yellow)
  (send *needle* :rotate #d90 :y)

  ;;Set hampen
  (setq *hampen* (make-cube 50 50 50))
  (send *hampen* :translate (v+ (float-vector 0 0 10) (send *center* :pos)))
  (send *hampen* :set-color :white)
  (send *hampen* :rotate #d45 :x)

  (send *desk* :put :left-coords
	(make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *pr2*))
  (send *needle* :put :left-coords
	(make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) 
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Sensing ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Voice Detection
(defun speech-cb(msg)
  (setq *ans* (elt (send msg :transcript) 0))
  (print *ans*)
  (if (equal *ans* "どうぞ")
      (setq *start-flag* 1)
    ;; 1回フラグが立てばそれでいい
    ))

(defun detect-voice () 
  (ros::ros-info "start waiting for call ~%")
  (ros::subscribe "speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-cb)
  (ros::rate 10)
  (while (not (eq *start-flag* 1))
    (ros::spin-once)
    (ros::sleep)
    (ros::ros-info "wait for voice command ...")
    )
  )

;; Perception
(defun cb-recog(msg)
  (ros::ros-info "boxes ~A" (length (send msg :boxes)))
  (if (and (> (length (send msg :boxes)) 0) (> (send (send (elt (send msg :boxes) 0) :dimensions) :x) 0)) 
      (setq *target-ypos-tmp* (send (send (send (elt (send msg :boxes) 0) :pose) :position) :y)) 
    )
  )

(defun detect-wound()
  (ros::subscribe "/recognize_wound/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'cb-recog)
  (ros::rate 10)
  (while (null *target-ypos-tmp*)
    (ros::ros-info "wait for target...")
    (ros::sleep)
    (ros::spin-once))
  (setq *target-ypos* *target-ypos-tmp*)
  )

(defun cb-rforce(msg)
  (ros::ros-info "checking right-virtual-force")
  (setq *rforce* (send (send (send msg :wrench) :force) :z))
  (format t "rforce: ~A ~%" *rforce*)
  (if (> (send (send (send msg :wrench) :force) :z) *draw-param-right*)
      (progn
	(setq *right-drawing* t)
	(format t "drawing~%")
	)
    )
  )

(defun cb-lforce(msg)
  (ros::ros-info "checking left-virtual-force")
  (setq *lforce* (send (send (send msg :wrench) :force) :z))
  (if (> (send (send (send msg :wrench) :force) :z) *draw-param-left*)
      (progn
	(setq *left-drawing* t)
	(format t "drawing~%")
	)
    )
  )
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Stitching ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Path ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define Trajectory Class
(defclass traj
  :super cascaded-coords
  :slots (points))

(defmethod traj
  (:init (&rest args)
	 (send-super* :init args)
    (dotimes (i *traj_num*)
      (push
       (make-cube 10 10 10 :pos (float-vector (* (- i 5) *interval*) 0 0))
       points))
    (dotimes (i (- (length points) 1))
      (send (elt points i) :assoc (elt points (+ 1 i))))
    (send self :assoc (car points))
    self)
  (:points () points)
  )


;; Define set-environment method
(defun set-env()
  ;;Set center
  (setq *deskh* *centerz*)

  (setq *center* (make-cube 10 10 10))
  (send *center* :translate (float-vector *centerx* *centery* *centerz*))
  (send *center* :set-color :black)
  (setq *o* (send *center* :copy-worldcoords))

  ;;Set desk
  (setq *desk* (make-cube *deskw* *deskw* *deskh*))
  (send *desk* :translate (float-vector *centerx* *centery* (/ *centerz* 2))) 
  (send *desk* :set-color :brown)

  ;;Set needle
  (setq *needle* (make-cylinder 0.5 *needle_len*))
  (send *needle* :translate (v+ (float-vector -100 20 300) (send *center* :pos)))
  (send *needle* :set-color :yellow)
  (send *needle* :rotate #d90 :y)

  ;;Set hampen
  (setq *hampen* (make-cube 50 50 50))
  (send *hampen* :translate (v+ (float-vector 0 0 10) (send *center* :pos)))
  (send *hampen* :set-color :white)
  (send *hampen* :rotate #d45 :x)

  (send *desk* :put :left-coords
	(make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *pr2*))
  (send *needle* :put :left-coords
	(make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) 
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Motion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;Define preparation task.
(defun preparation()
    (send *ri* :speak-jp "縫合を始めます．注意してください．" :wait t)
    (send *pr2* :reset-pose) ;;Set Initial Pose
    (send *pr2* :torso :waist-z :joint-angle 240)
    (send *ri* :start-grasp :arms)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *pr2* :head :look-at (send *center* :worldpos))

    (send *pr2* :larm :inverse-kinematics
	  (send (send (send *needle* :get :left-coords)
		      :copy-worldcoords)
		:rotate (deg2rad 90) :z)
	  :rotation-axis :z)  ;;IK to needle pose
    (send *pr2* :larm :end-coords :assoc *needle*)  ;;
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ;;Grasp needle (real)
    (send *ri* :speak-jp "左手に，針を持たせてください．" :wait t)
    (send *ri* :stop-grasp :larm :wait t)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    )


;;Define initial task.
(defun init-task()
    (send *ri* :speak-jp "手を閉じます．挟まれないように気をつけてください" :wait t)
    (send *ri* :start-grasp :larm :wait t)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *ri* :speak-jp "これから針を持って作業します．離れてください．" :wait t)
    (unix:sleep 2)

    ;;Grasp needle (dummy)
    (send *pr2* :larm :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords) :rotate (deg2rad 90) :z) :rotation-axis :z)
    (send *pr2* :larm :end-coords :assoc *needle*) ;;針を固定
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; others ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun stop()
  (send *ri* :stop-grasp)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;main.l;;;;;
;;Set PR2
(if (not (boundp '*pr2*)) (pr2-init)) 

;; Set the experimental environments.
(set-env)

;; Show all objects in irtviewer.
(objects (list *pr2* *center* *desk* *needle* *hampen*))

;; Human help Robot to grab needle.
(preparation)

;; The Nurse pass the needle saying "douzo!"
(if (send *ri* :simulation-modep)
    (unix:sleep 2)
  (detect-voice)
  )

;; Do initial task.
(init-task)

;; Dummy pos to be able to calculate IK
(setq current-coords (send (send *desk* :get :left-coords) :copy-worldcoords))
(setq new-coords (send current-coords :translate (float-vector 0 100 50))) 
(send *pr2* :larm :inverse-kinematics new-coords :rotation-axis :x)
(send *pr2* :larm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; Perception
;; Perception (dummy)
(ros::subscribe "/recognize_wound/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'cb-recog)
(if (send *ri* :simulation-modep)
    (setq *target-ypos-tmp* 0)
  (setq *target-ypos-tmp* nil)
  )
(detect-wound)

;; Stitch
(dotimes (i 4) ;;5は難しい。。。
  ;; Make trajectory points. 
  (setq r (instance traj :init))
  (send r :rotate pi/2 :z)
  (setq *target* (v+ (send *center* :pos) (float-vector (+ *target-xpos* (* i 3)) (+ *target-ypos* (* *traj_len* 0.12)) *target-zpos*))) ;; 0.2は実験則 ;; Trajectoryの中心座標をHampenにあわせている
  (send r :locate *target*  :world)
  (objects (append (list *pr2* *center*  *desk* *needle* *hampen*) (send r :points)))

  ;; Set start position.
  ;; Rarm
  (setq *r-pos* (send (elt (send r :points) 9) :copy-worldcoords))
  (send *r-pos* :rotate pi/2 :y :local)
  (send *r-pos* :rotate -pi/2 :z :local) ;; if -pi/2, rarm state is parallel to desk 
  (send *r-pos* :translate #f(-10 0 0) :local)
  (send *pr2* :rarm :inverse-kinematics *r-pos*  :rotation-axis t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  ;; Follow the trajectory points. Stitch.
  (setq tm 500)
  (setq path (list))
  (setq time-vec (list 4000))
  (dolist (e (send r :points))
    (let (ee) 
      (setq ee (send e :copy-worldcoords))
      (send ee :translate (float-vector (- *needle_len* *rarm_offset*) 0 0) :local)
      (send ee :rotate pi :z :local)
      (send ee :rotate pi/2 :y :local)
      (send ee :draw-on :flush t :size 100)
      (push (send *pr2* :larm :inverse-kinematics ee :rotation-axis t) path)
      (push tm time-vec)
      )
    )
  (pop time-vec)
  (nreverse time-vec)
  (nreverse path)
  (send *ri* :angle-vector-sequence path time-vec
	:default-controller 0.001
	:min-time 0.0001
	:minjerk-interpolation t
	)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)

  ;; Pass on the needle. (左手から右手に針を戻す)
  (send *ri* :start-grasp :rarm :wait t)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (send *ri* :stop-grasp :larm :wait t)
  (send *ri* :wait-interpolation)

  ;; Fix the right arm and neadle (針の固定を左から右に移す)
  (send *pr2* :larm :end-coords :dissoc *needle*)
  (send *pr2* :rarm :end-coords :assoc *needle*)
  (format t "is_first: ~A ~%" *is_first*)
  ;; Draw the needle from the object. (対象物から針を抜き出)
  (if (= *is_first* 1)
      (progn (send *pr2* :rarm :move-end-pos (float-vector 0 0 (- 0 *remain*)) :local)
	     (send *ri* :angle-vector-sequence (list (send *pr2* :angle-vector)) (list 10000)
		   :default-controller 0.001
		   :min-time 0.0001
		   :minjerk-interpolation t
		   )	     (send *ri* :wait-interpolation) 
	     (send *irtviewer* :draw-objects)
	     (setq *remain* (- *remain* *diff_remain*))
	     )
    (progn
      (setq remain_tmp (- *remain* *diff_remain*))
      (send *pr2* :rarm :move-end-pos (float-vector 0 0 -60) :local)
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation) 
      (send *irtviewer* :draw-objects)
      ;;(setq *remain* (- *remain* 60));; オリジナリティポイント: 1回縫うと針を引ける量が減るのを再現
      (format t "remaining ~A ~%" *remain*) ;; for debug! (*remain*が0になったらbreakするようにしたい)
      (if (> *remain*  remain_tmp)
	  (progn
	    (setq *right-drawing* nil)
	    (setq *position1* (send *ri* :state :angle-vector))
	    (ros::subscribe "/right_endeffector/wrench" geometry_msgs::WrenchStamped #'cb-rforce 1)
	    (send *pr2* :rarm :move-end-pos (float-vector 0 0 (- remain_tmp)) :local)
	    (setq *position2* (send *pr2* :angle-vector))
	    ;;(send *ri* :angle-vector (print (send *pr2* :angle-vector)) 10000) ;;bug
	    (send *ri* :angle-vector-sequence (list (send *pr2* :angle-vector)) (list 10000)
		  :default-controller 0.001
		  :min-time 0.0001
		  :minjerk-interpolation t
		  )
	    (send *irtviewer* :draw-objects)
	    (ros::duration-sleep 2.0)
	    (while (null *right-drawing*)
	      (ros::duration-sleep 0.05)
	      (ros::spin-once)
	      (if (not (send *ri* :interpolatingp))
		  (setq *right-drawing* t))
	      )
	    (send *ri* :cancel-angle-vector :wait t)
	    ;;(send *ri* :stop-motion)
	    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	    )
	)
      )
    )
 

  ;; Move left hand to the delivery point.
  (setq *pass-larm* (send (send (send (send (send *center* :copy-worldcoords)
					    :translate (float-vector 45 (* *needle_len* 0.5) 150))
				      :rotate pi/2 :z)
				:rotate pi/2 :y)
			  :rotate pi :x))
  (send *pass-larm* :draw-on :flush t :size 100) ;; for debug!
  (send *pr2* :larm :inverse-kinematics *pass-larm* :rotation-axis t :use-torso t) ;; Use torso! (腰を使わないとIKが解けないので注意)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)

  ;; Move right hand to the delivery point.
  (setq *pass-rarm* (send (send (send (send *center* :copy-worldcoords)
				      :translate (float-vector 45 (- 0 (* *needle_len* 0.5)) 150))
				:translate #f(-10 0 0))
			  :rotate -pi/2 :x))
  (send *pass-rarm* :draw-on :flush t :size 100) ;; for debug!
  (send *pr2* :rarm :inverse-kinematics *pass-rarm*  :rotation-axis t) ;; Do not use torso! (左手との相対座標が狂う)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)

  ;; Pass on the needle. (右手から左手に針を戻す)
  (send *ri* :start-grasp :larm)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)
  
  ;; Fix the left arm and needle again (針の固定を右から左に移す)
  (send *pr2* :rarm :end-coords :dissoc *needle*)
  (send *pr2* :larm :end-coords :assoc *needle*)
  (send *ri* :speak-jp "繰り返し，つぎの経路で縫います．注意してください．" :wait t)
  (unix:sleep 1)
  (send *pr2* :rarm :move-end-pos (float-vector 0 0 -50) :local)
  (send *ri* :wait-interpolation)
  (ros::rate 10)
  (setq *left-drawing* nil)
  ;;余っている糸を左に引く作業を知的化した
  (ros::subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped #'cb-lforce 1)
  (send *pr2* :larm :move-end-pos (float-vector 0 0 -240) :local)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 15000)
  (send *ri* :angle-vector-sequence (list (send *pr2* :angle-vector)) (list 15000)
	:default-controller 0.001
	:min-time 0.0001
	:end-coords-interpolation t
	)
  (while (null *left-drawing*)
    (ros::duration-sleep 0.02)
    (ros::spin-once)
    (if (not (send *ri* :interpolatingp))
	(setq *left-drawing* t))
    )
  (send *ri* :cancel-angle-vector :wait t)
  (setq *is_first* 0)
  )

(send *pr2* :reset-pose) ;;Set Initial Pose
(send *ri* :stop-grasp :arms)
(send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

(send *ri* :speak-jp "縫合がおわりました．お疲れ様でした．" :wait t)
