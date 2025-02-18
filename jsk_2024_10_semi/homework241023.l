#!/usr/bin/env roseus

;chmod u+x ./surgery_motion_1.l --> ./surgery_motion_1.l ;; change from ki00119 Michi-Tsubaki

;;Okada-seisei seminar homework achievement ;;last editted oct 29 ;; Michi-Tsubaki

;;<description>
;;Thiscode is for trying to make unique motion for sewing something. Still Trying


;;Load packages for using PR2 model and PR2 robot in real.
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") 


;;Making PR2 object
(if (not (boundp '*pr2*)) (pr2-init)) 


;;Setting cube as a desk for surgery. 
(setq *desk* (make-cube 500 500 600))
(send *desk* :translate (float-vector 700 0 350)) ;; 質問：世界座標の原点がわからないので，机を床に設置(接地)できない．
(send *desk* :set-color :brown)
;[memo] (send *desk* :translate (float-vector 500 0 80)) ;;CAUTION!!(by Michi-Tsubaki)<-parameter that make robot motion NOT converge


;;Setting cylinder as needle.
(setq *needle* (make-cylinder 2 40)) 
(send *needle* :translate (float-vector 700 0 700))
(send *needle* :set-color :yellow)
(send *needle* :rotate #d90 :x)


;;Setting hampen
(setq *hampen* (make-cube 60 60 20))
(send *hampen* :translate (float-vector 700 0 660)) ; 質問：座標じゃなくてdeskの上面に設置させたい
(send *hampen* :set-color :white)

;;Setting coordination.
(send *desk* :put :left-coords
      (make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *desk*))
;;Setting coordination.
(send *needle* :put :left-coords
      (make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) ;;質問：parent座標をpr2に設定したら収束しなくなった

(send *pr2* :larm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

;;Show all objects in viewer. 
(objects (list *pr2* *desk* *needle* *hampen*)) ;;Dont forget to add all items


;Setting Initial Pose
(send *pr2* :reset-pose)
(send *ri* :stop-grasp :arms)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


;Putting arms above each side of the desk
;;Setting the right arm and the left arm to the initial position.
;;USING INVERSE KINEMATICS
(send *pr2* :larm :inverse-kinematics
      (send (send *desk* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)
;(send *pr2* :rarm :inverse-kinematics
;      (send (send *desk* :get :left-coords) :copy-worldcoords)
;      :rotation-axis :z)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


;;Grasping needle
;;1. Reach
(send *pr2* :larm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)
(send *pr2* :larm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)

;;2. Graspe
(send *ri* :start-grasp :arms)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)



;;Stitching with the left hand
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 -10.0 -10.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


(send *pr2* :larm :wrist-p :joint-angle -100)
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 50.0 20.0 0.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 -12.0 20.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


(send *pr2* :larm :wrist-p :joint-angle -80)
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector -35.0 -10.0 5.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


(send *pr2* :larm :elbow-r :joint-angle 95)
(send *pr2* :larm :shoulder-r :joint-angle 95)
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 2.0 2.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)


;;Switching hands
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :translate (float-vector -10.0 0 0)
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
      :rotation-axis :x))
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
;(unix:sleep 1)





;;質問：ここでikが上手く行かなくなった。左では上手く行っているのに、右では下からアプローチする。assocの切り替えもわからない。あと，オフセット
;;Grasping needle with right hand
(send *pr2* :rarm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)
(send *pr2* :rarm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
