(require "package://pr2eus/pr2.l")  ;;import pr2 package
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") 
(ros::load-ros-manifest "roseus")
(if (not (boundp '*pr2*)) (pr2-init)) 

;; 一辺600mmの立方体を出現させる
(setq *cube* (make-cube 600 600 600))
;; 立方体を(700, 0, 300)移動
(send *cube* :translate (float-vector 700 0 300))
;; chabudaiフラグ
(setq *chabudai* nil)

;;Set coordination.
(send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 700 0 300))
       :parent *cube*))

(send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

(send *pr2* :reset-pose) ;; 初期姿勢
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(objects (list *pr2* *cube*))


(defun callback (msg)
    (format t "robot_mood: ~A~%" (send msg :data))
    (when (< (send msg :data) 4)
        (setq *chabudai* t)
    )
)

(defun chabudai-gaeshi ()
    (send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 700 0 300))
       :parent *cube*)
    )

    (send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z
    )

    (send *pr2* :reset-pose)
    (send *pr2* :larm :collar-y :joint-angle 100)
    (send *pr2* :rarm :collar-y :joint-angle -100)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *pr2* :larm :shoulder-p :joint-angle 70)
    (send *pr2* :larm :elbow-p :joint-angle -50)
    (send *pr2* :rarm :shoulder-p :joint-angle 70)
    (send *pr2* :rarm :elbow-p :joint-angle -50)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ; (send *pr2* :larm :shoulder-p :joint-angle 70)
    (send *pr2* :larm :shoulder-r :joint-angle 60)
    (send *pr2* :larm :collar-y :joint-angle 32)
    (send *pr2* :larm :elbow-p :joint-angle -110)
    (send *pr2* :larm :elbow-r :joint-angle 180)
    (send *pr2* :larm :wrist-p :joint-angle -30)
    (send *pr2* :larm :wrist-r :joint-angle -30)
    ; (send *pr2* :rarm :shoulder-p :joint-angle 70)
    (send *pr2* :rarm :shoulder-r :joint-angle -60)
    (send *pr2* :rarm :collar-y :joint-angle -32)
    (send *pr2* :rarm :elbow-p :joint-angle -110)
    (send *pr2* :rarm :elbow-r :joint-angle 180)
    (send *pr2* :rarm :wrist-p :joint-angle -30)
    (send *pr2* :rarm :wrist-r :joint-angle 30)
    (send *pr2* :head :neck-p :joint-angle 50)
    (send *ri* :stop-grasp :arms)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *pr2*
        :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                        :translate (float-vector 1.0 0.5 0.0))
        :move-target (send *cube* :get :left-coords)
        :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t)
    (send *pr2*
        :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                        :translate (float-vector 1.0 -0.5 0.0))
        :move-target (send *cube* :get :left-coords)
        :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
        :rotation-axis t)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *pr2*
        :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                        :translate (float-vector 0.0 0.0 2.0))
        :move-target (send *cube* :get :left-coords)
        :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t)
    (send *pr2*
        :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                        :translate (float-vector 0.0 0.0 2.0))
        :move-target (send *cube* :get :left-coords)
        :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
        :rotation-axis t)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    ;; sleep 1秒
    (unix:sleep 1)

    (send *pr2* :larm :shoulder-p :joint-angle -20)
    (send *pr2* :larm :shoulder-r :joint-angle 20)
    (send *pr2* :larm :elbow-p :joint-angle -100)
    (send *pr2* :rarm :shoulder-p :joint-angle -20)
    (send *pr2* :rarm :shoulder-r :joint-angle -20)
    (send *pr2* :rarm :elbow-p :joint-angle -100)
    (send *pr2* :head :neck-p :joint-angle -10)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
)

(ros::subscribe "/robot_mood" std_msgs::Int32 #'callback)

(do-until-key
 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
 (send *irtviewer* :draw-objects)
 (x::window-main-one)
 (ros::spin-once)
    (when *chabudai*
        (chabudai_gaeshi)
        (setq *chabudai* nil)
    )
)