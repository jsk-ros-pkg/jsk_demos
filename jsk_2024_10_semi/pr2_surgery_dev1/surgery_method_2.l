#!/usr/bin/env roseus

;;Okada-seisei seminar
;;Surgery-method dev1
;;last editted Nov 12 ;; Michi-Tsubaki

;;<DESCRIPTION>
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
(setq *needle* (make-cylinder 0.5 70)) 
(send *needle* :translate (v+ (float-vector 0 20 100) (send *center* :pos)))
(send *needle* :set-color :yellow)
(send *needle* :rotate #d270 :x)

;;Set hampen
(setq *hampen* (make-cube 50 50 50))
(send *hampen* :translate (v+ (float-vector 0 0 10) (send *center* :pos)))
(send *hampen* :set-color :white)
(send *hampen* :rotate #d45 :x)



;;Define set-trajectory function
(defun set-trajectory (x)
  (let ((prev-pos *center*))  ; 最初の座標を原点に設定
    ;; tp1
    (setq *tp1* (make-cube 2 2 2))
    (send *tp1* :translate (v+ (float-vector (- x 15) 30 40) (send *center* :pos)))
    (send *tp1* :rotate #d0 :z)
    (send *tp1* :set-color :red)
    (send *tp1* :put :left-coords
      (make-cascoords
       :coords (send (send *tp1* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp1*))
    (setq prev-pos (send *tp1* :pos))
    ;; tp2
    (setq *tp2* (make-cube 2 2 2))
    (send *tp2* :translate (v+ (float-vector 0 -10 0) prev-pos))
    (send *tp2* :rotate #d90 :z)
    (send *tp2* :set-color :red)
    (send *tp2* :put :left-coords
      (make-cascoords
       :coords (send (send *tp2* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp1*))
    (setq prev-pos (send *tp2* :pos))
    ;; tp3
    (setq *tp3* (make-cube 2 2 2))
    (send *tp3* :translate (v+ (float-vector 0 -10 0) prev-pos))
    (send *tp3* :rotate #d270 :z)
    (send *tp3* :set-color :red)
    (send *tp3* :put :left-coords
      (make-cascoords
       :coords (send (send *tp3* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp2*))
    (setq prev-pos (send *tp3* :pos))
    ;; tp4
    (setq *tp4* (make-cube 2 2 2))
    (send *tp4* :translate (v+ (float-vector 0 -10 0) prev-pos))
    (send *tp4* :rotate #d90 :x)
    (send *tp4* :set-color :red)
    (send *tp4* :put :left-coords
      (make-cascoords
       :coords (send (send *tp3* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp3*))
    (setq prev-pos (send *tp4* :pos))
    ;; tp5
    (setq *tp5* (make-cube 2 2 2))
    (send *tp5* :translate (v+ (float-vector 0 10 0) prev-pos))
    (send *tp5* :rotate #d90 :x)
    (send *tp5* :set-color :red)
    (send *tp5* :put :left-coords
      (make-cascoords
       :coords (send (send *tp3* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp4*))
    (setq prev-pos (send *tp5* :pos))
    ;; tp6
    (setq *tp6* (make-cube 2 2 2))
    (send *tp6* :translate (v+ (float-vector 0 -10 0) prev-pos))
    (send *tp6* :rotate #d90 :x)
    (send *tp6* :set-color :red)
    (send *tp6* :put :left-coords
      (make-cascoords
       :coords (send (send *tp6* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp5*))
    (setq prev-pos (send *tp6* :pos))
    ;; tp7
    (setq *tp7* (make-cube 2 2 2))
    (send *tp7* :translate (v+ (float-vector 0 -70 0) prev-pos))
    (send *tp7* :rotate #d90 :x)
    (send *tp7* :set-color :red)
    (send *tp7* :put :left-coords
      (make-cascoords
       :coords (send (send *tp6* :copy-worldcoords) 
                    :translate (float-vector 1 0 0))
              :rot #2f((0 0 1)
                (-1 0 0)
                (0 -1 0))
       :parent *tp6*))
    (setq prev-pos (send *tp7* :pos))))


(set-trajectory 0)

;;Set coordination.
(send *desk* :put :left-coords
      (make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *pr2*))
(send *needle* :put :left-coords
      (make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) 

;;Show Worldrecord
(setq *arrow* (arrow))
(send *arrow* :copy-worldcoords)

;;Show all objects in IRTVIEWER
(objects (list *pr2* *center* *arrow* *desk* *needle* *hampen* *tp1* *tp2* *tp3* *tp4* *tp5* *tp6* *tp7*)) 

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


;;Grasping needle (dummy)
(send *pr2* :larm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)
(send *pr2* :larm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)


;;Set Start Position
;;Larm
(setq current-coords (send (send *desk* :get :left-coords) :copy-worldcoords))
(setq new-coords (send current-coords :translate (float-vector 0 100 50))) 
(send *pr2* :larm :inverse-kinematics new-coords :rotation-axis :x)
(send *pr2* :larm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;;Rarm
(setq current-coords (send (send *desk* :get :left-coords) :copy-worldcoords))
(setq new-coords (send current-coords :translate (float-vector 0 -100 100))) 
(send *pr2* :rarm :inverse-kinematics new-coords :rotation-axis :x)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)


(do ((i 0 (+ i 1)))                     ; i を 0 から 1 ずつ増加
    ((> i 3) (format t "done"))                ;; それぞれの位置と角度情報をリストに格納     ; i が 3 より大きくなったら終了
    (set-trajectory (- (* i 10) 3))         ; i の値に基づいて座標計算
    (format t "Trajectory tp1:~A tp2:~A tp3:~A tp4:~A tp5:~A tp6:~A tp7:~A. ~%" (send *tp1* :copy-worldcoords) (send *tp2* :copy-worldcoords) (send *tp3* :copy-worldcoords) (send *tp4* :copy-worldcoords) (send *tp5* :copy-worldcoords) (send *tp6* :copy-worldcoords) (send *tp7* :copy-worldcoords))
    (objects (list *pr2* *center* *arrow* *desk* *needle* *hampen* *tp1* *tp2* *tp3* *tp4* *tp5* *tp6* *tp7*))
    (send *irtviewer* :draw-objects)

  (send *ri* :start-grasp :larm :wait t)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp1* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0)) 
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp2* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))  ;; 5cm上、5cm手前
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp3* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp4* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp5* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp6* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  #|
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *tp7* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
|#



  ;;Rarm Swith Hands
  (send *ri* :stop-grasp :rarm :wait t)
  (send *irtviewer* :draw-objects)
  
  (send *pr2* :rarm :inverse-kinematics
	(send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 0.0 60.0)))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)

  (send *ri* :start-grasp :rarm :wait t)
  (send *irtviewer* :draw-objects)
  (send *pr2* :rarm
        :inverse-kinematics (send (send (send *tp7* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector 0 -7 0))
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis :z
        :debug-view t
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (send *ri* :stop-grasp :larm :wait t)
  (send *irtviewer* :draw-objects)


    
  (unix:sleep 2))                        ; 1秒間スリープ
