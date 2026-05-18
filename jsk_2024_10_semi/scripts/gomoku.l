(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") 
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

; (send *pr2* :reset-pose) ;; 初期姿勢

;; 一辺600mmの立方体を出現させる
(setq *cube* (make-cube 600 600 600))
;; 立方体を(700, 0, 300)移動
(send *cube* :translate (float-vector 700 0 300))
(objects (list *pr2* *cube*))

;; PR2の左腕を指定の角度に動かす
(send *pr2* :larm :shoulder-p :joint-angle 15)
(send *pr2* :larm :shoulder-r :joint-angle 90)
(send *pr2* :larm :collar-y :joint-angle 45)
(send *pr2* :larm :elbow-p :joint-angle -100)
(send *pr2* :larm :elbow-r :joint-angle 130)
(send *pr2* :larm :wrist-p :joint-angle -10)
(send *pr2* :larm :wrist-r :joint-angle 0)
(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)

;; inverse-kinematicsを使ってPR2の左腕を現在位置から相対的に(0, 0, 100)移動
; (let ((target-coords (make-coords :pos (float-vector 0 0 100))))
;   (send *pr2* :larm :inverse-kinematics target-coords))

;;Set coordination.
(send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 700 0 300))
       :parent *cube*))

(send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

; (send *ri* :stop-grasp :arms)

(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 0.0 -2.5))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

; (send *ri* :start-grasp :arms)

(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)

(send *pr2* :larm :elbow-r :joint-angle 120)

(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 0.0 2.5))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)

(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 5.0 -5.0 0.0))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

(send *pr2* :larm :wrist-r :joint-angle 0)

(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)

(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 0.0 -2.5))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

(send *irtviewer* :draw-objects)
