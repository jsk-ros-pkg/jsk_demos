#!/usr/bin/env roseus

;; Today's review
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; Spawn the dummy cube
;;(setq *cube* (make-cube 200 200 200))
;; move the cube (400, 0, 800)
;;(send *cube* :translate (float-vector 400 0 800))

(ros::roseus "motomochi")
(ros::load-ros-manifest "jsk_recognition_msgs")
(defvar *topic-name* "/HSI_color_filter/boxes")
(setq *box* (one-shot-subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray))

(setq *cube* (make-cube 200 200 200))
(setq *x* (send (car (send *box* :boxes)) :pose :position :x))
(setq *y* (send (car (send *box* :boxes)) :pose :position :y))
(setq *z* (send (car (send *box* :boxes)) :pose :position :z))

(send *cube* :translate (float-vector (* *x* 1000) (* *y* 1000) (* *z* 1000)))

(objects (list *pr2* *cube*))

;; Add coordinate to the left side of the cube
(send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 0 100 0))
       :parent *cube*))

;; Inverse kinematics (IK) is the mathematical process of calculating the variable joint parameters
;; With the PR2 left arm, solve the inverse kinematics to the cube side. 
(send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

;; Lift the cube with PR2's left arm and thrust it forward
;; Contact the left arm of PR2 to the left side of the cube
(send *pr2* :larm :end-coords :assoc *cube*)
;; Solve the inverse kinematics so that the cube moves to the position (100, 0, 200) relative to its current position
(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 100.0 0.0 100.0))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

;; If you want to send the current angles to the real robot
;; (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
