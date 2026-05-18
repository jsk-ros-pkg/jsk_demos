(require "package://pr2eus/pr2.l")  ;;import pr2 package
; (require "package://pr2eus/pr2-utils.l")
; (require "package://pr2eus/pr2-interface.l") 
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

;; 一辺600mmの立方体を出現させる
(setq *cube* (make-cube 100 100 100))
;; 立方体を(700, 0, 300)移動
(send *cube* :translate (float-vector 700 0 300))

;;Set coordination.
(send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 700 0 300))
       :parent *cube*))

(send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

; (send *pr2* :reset-pose) ;; 初期姿勢
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(objects (list *pr2* *cube*))


(send *pr2* :larm :shoulder-p :joint-angle 70)
(send *pr2* :larm :shoulder-r :joint-angle 60)
(send *pr2* :larm :collar-y :joint-angle 32)
(send *pr2* :larm :elbow-p :joint-angle -110)
(send *pr2* :larm :elbow-r :joint-angle 180)
(send *pr2* :larm :wrist-p :joint-angle -30)
(send *pr2* :larm :wrist-r :joint-angle -40)
(send *pr2* :rarm :shoulder-p :joint-angle 70)
(send *pr2* :rarm :shoulder-r :joint-angle -60)
(send *pr2* :rarm :collar-y :joint-angle -32)
(send *pr2* :rarm :elbow-p :joint-angle -110)
(send *pr2* :rarm :elbow-r :joint-angle 180)
(send *pr2* :rarm :wrist-p :joint-angle -30)
(send *pr2* :rarm :wrist-r :joint-angle 40)
(send *pr2* :head :neck-p :joint-angle 50)
; (send *ri* :stop-grasp :arms)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)


(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.5 0.0 0.0))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.5 0.0 0.0))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)

; (send *ri* :start-grasp :arms)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; sleep 2秒
(unix:sleep 2)

(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector -0.5 0.0 0.5))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector -0.5 0.0 0.5))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

;; sleep 1秒
(unix:sleep 1)


(send *pr2* :larm :shoulder-p :joint-angle 0)
(send *pr2* :larm :shoulder-r :joint-angle 5)
(send *pr2* :rarm :shoulder-p :joint-angle 0)
(send *pr2* :rarm :shoulder-r :joint-angle -5)
(send *pr2* :head :neck-p :joint-angle -10)
; (send *ri* :stop-grasp :arms)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 250)
; (send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
