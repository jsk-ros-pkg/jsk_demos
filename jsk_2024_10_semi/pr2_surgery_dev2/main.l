#!/usr/bin/env roseus

;;(progn (ros::rate 5)(do-until-key (send *ri* :state :potentio-vector)(print (v- (send *ri* :robot :larm :end-coords :worldpos) (send *pr2* :larm :end-coords :worldpos))) (ros::sleep)))

#| DESCRIPTION
Okada-sensei seminar
Surgery-method dev2
Last editted Nov 23 by Michi-Tsubaki
GOAL: Trying to make unique motion for sewing sponge.
## Done
- voice recognition
- perception
- stitch sponge 

今後やること
・画像認識で位置決め(Perception)
・手術ロボットでしてみる
・曲がった経路にする(make-trajectory)
|#

;;Load pkgs around PR2 Interface
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") ;;*ri*
(require "package://pr2eus/speak.l") ;;pkg for speaking


(ros::load-ros-package "jsk_recognition_msgs")


#| UNDER CONSTRUCTION
;; For perception
(ros::load-ros-manifest "jsk_recognition_msgs")

;; For listening
(ros::roseus-add-msgs "speech_recognition_msgs")

;;set node
(ros::roseus "surgery")
|#

(ros::roseus-add-msgs "speech_recognition_msgs")

;;Load class related
(load "models/arrow-object.l") ;;*arrow*


;; Voice Detection
(defun speech-cb(msg)
  (setq *ans* (elt (send msg :transcript) 0))
  (print *ans*)
  (if (equal *ans* "メス")
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



(defun cb-recog(msg)
  (ros::ros-info "boxes ~A" (length (send msg :boxes)))
  (if (> (length (send msg :boxes)) 0)

      (setq *target-ypos-tmp* (send (send (send (elt (send msg :boxes) 0) :pose) :position) :y)) 
    )
  )

;;Global Param
(setq *start-flag* 0)
(setq *centerx* 700)
(setq *centery* 0)
(setq *centerz* 750)
(setq *deskw* 500)
(setq *needle_len* 70)
(setq *interval* 10)
(setq *traj_num* 10)
(setq *traj_len* (* *interval* *traj_num*))

;; Set remain
(setq *remain* 280) ;;how long remaining thread?
(setq *diff_remain* 100)

;; Offset
(setq *rarm_offset* 2)


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
    self) ;; :init
  (:points () points)
  ) ;; defmethod


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


;;Define preparation task.
(defun preparation()
    (require "package://pr2eus/pr2.l")
    (require "package://pr2eus/pr2-utils.l")
    (require "package://pr2eus/pr2-interface.l")
    (require "package://pr2eus/speak.l")
    
    (send *ri* :speak-jp "初期姿勢に戻ります．注意してください．" :wait t)
    (send *pr2* :reset-pose) ;;Set Initial Pose
    (send *ri* :start-grasp :arms)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
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
    (send *ri* :speak-jp "左手を開きます．針を持たせてください．" :wait t)
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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    )


;;;;main.l;;;;;

;;Set arrow (for debug)
(setq *arrow* (arrow))
(send *arrow* :copy-worldcoords)

;;Set PR2
(if (not (boundp '*pr2*)) (pr2-init)) 

;; Set the experimental environments.
(set-env)

;; Show all objects in irtviewer.
(objects (list *pr2* *center* *arrow* *desk* *needle* *hampen*))

;; Human help Robot to grab needle.
(preparation)

;; The Doctore declare the start of the surgery.
;;音声認識********************************************************************************************************
(detect-voice)
;;(unix:sleep 3) ;;dummy

;; Do initial task.
(init-task)

;; Perception (dummy)
(if (send *ri* :simulation-modep)
    (setq *target-ypos-tmp* 0)
  (setq *target-ypos-tmp* nil)
  )
(setq *target-ypos* 0)
(setq *target-xpos* 0)
(setq *target-zpos* 30)
(ros::subscribe "/recognize_wound/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'cb-recog)


(setq current-coords (send (send *desk* :get :left-coords) :copy-worldcoords))
(setq new-coords (send current-coords :translate (float-vector 0 100 50))) 
(send *pr2* :larm :inverse-kinematics new-coords :rotation-axis :x)
(send *pr2* :larm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;;

(ros::rate 10)
(while (null *target-ypos-tmp*)
  (ros::ros-info "wait for target...")
  ;;(ros::duration-sleep 0.1)
  (ros::sleep)
  (ros::spin-once))
(setq *target-ypos* *target-ypos-tmp*)

;; Stitch
(dotimes (i 5)
  ;; Make trajectory points. 
  (setq r (instance traj :init))
  (send r :rotate pi/2 :z)
  (setq *target* (v+ (send *center* :pos) (float-vector (+ *target-xpos* (* i 3)) (+ *target-ypos* (* *traj_len* 0.2)) *target-zpos*))) ;; 0.2は実験則 ;; Trajectoryの中心座標をHampenにあわせている
  (send r :locate *target*  :world)
  (objects (append (list *pr2* *center* *arrow* *desk* *needle* *hampen*) (send r :points)))

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
  (dolist (e (send r :points))
    (let (ee) 
      (setq ee (send e :copy-worldcoords))
      (send ee :translate (float-vector (- *needle_len* *rarm_offset*) 0 0) :local)
      (send ee :rotate pi :z :local)
      (send ee :rotate pi/2 :y :local)
      (send ee :draw-on :flush t :size 100)
      (send *pr2* :larm :inverse-kinematics ee :rotation-axis t)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
      (send *ri* :wait-interpolation)
      (send *irtviewer* :draw-objects)
      )
    )

  ;; Pass on the needle. (左手から右手に針を戻す)
  (send *ri* :start-grasp :rarm :wait t)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (send *ri* :stop-grasp :larm :wait t)
  (send *ri* :wait-interpolation)

  ;; Fix the right arm and neadle (針の固定を左から右に移す)
  (send *pr2* :larm :end-coords :dissoc *needle*)
  (send *pr2* :rarm :end-coords :assoc *needle*)
  
  ;; Draw the needle from the object. (対象物から針を抜き出)
  (send *pr2* :rarm :move-end-pos (float-vector 0 0 (- 0 *remain*)) :local)
  (- *remain* *diff_remain*) ;; オリジナリティポイント: 1回縫うと針を引ける量が減るのを再現
  (format t "remaining ~A ~%" *remain*) ;; for debug! (*remain*が0になったらbreakするようにしたい)********************************************************************************************
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation) 
  (send *irtviewer* :draw-objects)

  ;; Move left hand to the delivery point.
  (setq *pass-larm* (send (send (send (send (send *center* :copy-worldcoords)
					    :translate (float-vector 50 (* *needle_len* 0.5) 150))
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
				:translate (float-vector 50 (- 0 (* *needle_len* 0.5)) 150))
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
  (send *ri* :speak-jp "繰り返し次の経路を縫います．注意してください．" :wait t)
  (unix:sleep 1)
  (send *pr2* :rarm :move-end-pos (float-vector 0 0 -50) :local)
  (send *pr2* :larm :move-end-pos (float-vector 0 0 -100) :local)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (send *ri* :wait-interpolation) 
  (send *irtviewer* :draw-objects)
 
  )
