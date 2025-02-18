#!/usr/bin/env roseus

;;Okada-seisei seminar
;;Surgery-method dev2
;;last editted Nov 12 ;; Michi-Tsubaki

;;<DESCRIPTION>
;;This program is for trying to make unique motion for sewing sponge. Still Trying

;;Load packages for using PR2 model and PR2 robot in real.
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l") ;;pkg for speaking
(load "models/arrow-object.l")

;;Set PR2
(if (not (boundp '*pr2*)) (pr2-init)) 

;;Set Center of a Table
(setq *center* (make-cube 10 10 10))
(send *center* :translate (float-vector 700 0 750))
(send *center* :set-color :black)

;;Set cube as a desk for surgery. 
(setq *desk* (make-cube 500 500 750))
(send *desk* :translate (float-vector 700 0 750/2)) 
(send *desk* :set-color :brown)

;;Set cylinder as needle.
(setq *needle* (make-cylinder 0.5 70)) 
(send *needle* :translate (v+ (float-vector 0 20 100) (send *center* :pos)))
(send *needle* :set-color :yellow)
(send *needle* :rotate #d90 :x)

;;Set hampen
(setq *hampen* (make-cube 50 50 50))
(send *hampen* :translate (v+ (float-vector 0 0 10) (send *center* :pos)))
(send *hampen* :set-color :white)
(send *hampen* :rotate #d45 :x)




(defclass traj
  :super cascaded-coords
  :slots (points))
(defmethod traj
  (:init (&rest args)
	 (send-super* :init args)
    (dotimes (i 10)
      (push
       (make-cube 10 10 10 :pos (float-vector (* (- i 5) 20) 0 0))
       points))
    (dotimes (i (- (length points) 1))
      (send (elt points i) :assoc (elt points (+ 1 i))))
    (send self :assoc (car points))
    self) ;; :init
  (:points () points)
  ) ;; defmethod
;;This program is for trying to make unique motion for sewing sponge. Still Trying

;;Load pkgs for using PR2 model and PR2 robot in real.
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l") ;;pkg for speaking
(load "models/arrow-object.l")

;;Define Trajectory Point (7)
(defvar *tp1* nil)
(defvar *tp2* nil)
(defvar *tp3* nil)
(defvar *tp4* nil)
(defvar *tp5* nil)
(defvar *tp6* nil)
(defvar *tp7* nil)

;;Set PR2
(if (not (boundp '*pr2*)) (pr2-init)) 

;;Set Center of a Table
(setq *center* (make-cube 10 10 10))
(send *center* :translate (float-vector 700 0 750))
(send *center* :set-color :black)

;;Set cube as a desk for surgery. 
(setq *desk* (make-cube 500 500 750))
(send *desk* :translate (float-vector 700 0 750/2)) 
(send *desk* :set-color :brown)

;;Set cylinder as needle.
;;Set needle
(setq *needle* (make-cylinder 0.5 70)) 
(send *needle* :translate (v+ (float-vector 0 20 100) (send *center* :pos)))
(send *needle* :set-color :yellow)
(send *needle* :rotate #d90 :x)
  

;;Show Worldrecord
(setq *arrow* (arrow))
(send *arrow* :copy-worldcoords)

;;Show all objects in IRTVIEWER
(objects (list *pr2* *center* *arrow* *desk* *needle* *hampen*))


;;Set initial pose
(send *ri* :speak-jp "初期姿勢に戻ります。" :wait t)
(send *pr2* :reset-pose)
(send *ri* :start-grasp :arms)

(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

(send *ri* :speak-jp "左手を開きます。針を持たせてください。")
(send *ri* :stop-grasp :larm :wait t)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

(send *ri* :speak-jp "閉じます。" :wait t)
(send *ri* :start-grasp :larm :wait t)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

(send *ri* :speak-jp "これから針を持って作業します．離れてください．" :wait t)
(unix:sleep 2)


(setq r (instance traj :init))
(send r :rotate pi/2 :z)
(send r :locate #f(800 0 900) :world)
(objects (append (list *pr2* *center* *arrow* *desk* *needle* *hampen*) (send r :points)))

;;(objects (list *pr2* (elt (send r :points) 0) (elt (send r :points) 1) (elt (send r :points) 2)

(dolist (e (send r :points)
  (send *pr2* :larm :inverse-kinematics e
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (format t "debug~%")
  (unix:sleep 1)
  )
  
  )

