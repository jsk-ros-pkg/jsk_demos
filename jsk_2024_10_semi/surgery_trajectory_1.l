;;Okada-seisei seminar
;;Surgery-method
;;last editted Nov 12 ;; Michi-Tsubaki

;;<DESCRIPTION>
;;This program is for trying to make unique motion for sewing sponge. Still Trying


;;Load packages for using PR2 model and PR2 robot in real.
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l") ;;pkg for speaking


;;Define Trajectory Point (7)
(defvar *tp1* nil)
(defvar *tp2* nil)
(defvar *tp3* nil)
(defvar *tp4* nil)
(defvar *tp5* nil)
(defvar *tp6* nil)
(defvar *tp7* nil)

;;Define set-trajectory function
(defun set-trajectory (x)
  (let ((prev-pos (float-vector x -20 40)))  ; 最初の座標を原点に設定
    ;; tp1
    (setq *tp1* (make-cube 2 2 2))
    (send *tp1* :translate (v- (float-vector 0 5 0) (send *center* :pos)))
    (send *tp1* :rotate #d90 :x)
    (send *tp1* :set-color :red)
    (setq prev-pos (send *tp1* :pos))
    ;; tp2
    (setq *tp2* (make-cube 2 2 2))
    (send *tp2* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp2* :rotate #d90 :x)
    (send *tp2* :set-color :red)
    (setq prev-pos (send *tp2* :pos))
    ;; tp3
    (setq *tp3* (make-cube 2 2 2))
    (send *tp3* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp3* :rotate #d90 :x)
    (send *tp3* :set-color :red)
    (setq prev-pos (send *tp3* :pos))
    ;; tp4
    (setq *tp4* (make-cube 2 2 2))
    (send *tp4* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp4* :rotate #d90 :x)
    (send *tp4* :set-color :red)
    (setq prev-pos (send *tp4* :pos))
    ;; tp5
    (setq *tp5* (make-cube 2 2 2))
    (send *tp5* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp5* :rotate #d90 :x)
    (send *tp5* :set-color :red)
    (setq prev-pos (send *tp5* :pos))
    ;; tp6
    (setq *tp6* (make-cube 2 2 2))
    (send *tp6* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp6* :rotate #d90 :x)
    (send *tp6* :set-color :red)
    (setq prev-pos (send *tp6* :pos))
    ;; tp7
    (setq *tp7* (make-cube 2 2 2))
    (send *tp7* :translate (v- (float-vector 0 5 0) prev-pos))
    (send *tp7* :rotate #d90 :x)
    (send *tp7* :set-color :red)
    (setq prev-pos (send *tp7* :pos))))

(set-trajectory 0)

;;Set Center of a Table
(setq *center* (make-cube 10 10 10))
(send *center* :translate (float-vector 700 0 750))
(send *center* :set-color :black)




;;Set coordination.
(send *desk* :put :left-coords
      (make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *pr2*))
(send *needle* :put :left-coords
      (make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) 

;;Show all objects in IRTVIEWER
(objects (list *pr2* *center* *arrow* *desk* *needle* *hampen* *tp1* *tp2* *tp3* *tp4* *tp5* *tp6* *tp7*)) 


(do ((i 0 (+ i 1)))                     ; i を 0 から 1 ずつ増加
    ((> i 3) 'done)                ;; それぞれの位置と角度情報をリストに格納     ; i が 3 より大きくなったら終了
  (set-trajectory (- (* i 10) 3))         ; i の値に基づいて座標計算
  (send *irtviewer* :draw-objects)       ; 立方体を描画
  (send *needle* :larm :inverse-kinematics
      (send (send *tp1* :get :left-coords) :copy-worldcoords)
      :rotation-axis :x)
  (send *pr2* :larm :end-coords :assoc *needle*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (unix:sleep 1))                        ; 1秒間スリープ
