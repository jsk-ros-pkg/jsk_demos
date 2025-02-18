#!/usr/bin/env roseus
;chmod u+x ./surgery_motion_1.l --> ./surgery_motion_1.l ;; change from ki00119 Michi-Tsubaki

;;Okada-seisei seminar homework achievement ;;last editted oct 29 ;; Michi-Tsubaki

;;<description>
;;Thiscode is for trying to make unique motion for sewing something. Still Trying


;;Load packages for using PR2 model and PR2 robot in real.
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l") ;;use :speak-jp ;;https://github.com/wkentaro/jsk_pr2eus/blob/fad0691e0131275b76568a68096c6c9f6eead13e/pr2eus/speak.l#L4
(load "models/arrow-object.l")


;;Making PR2 object
(if (not (boundp '*pr2*)) (pr2-init)) 


;;Setting cube as a desk for surgery. 
(setq *desk* (make-cube 500 500 800))
(send *desk* :translate (float-vector 700 0 400)) 
(send *desk* :set-color :brown)
;[memo] (send *desk* :translate (float-vector 500 0 80)) ;;CAUTION!!(by Michi-Tsubaki)<-parameter that make robot motion NOT converge

;;Setting cylinder as needle.
(setq *needle* (make-cylinder 2 40)) 
(send *needle* :translate (float-vector 700 0 850))
(send *needle* :set-color :yellow)
(send *needle* :rotate #d90 :x)

;;Set hampen
(setq *hampen* (make-cube 60 60 30))
(send *hampen* :translate (float-vector 700 0 830)) 
;(send *hampen* :translate (v+ (float-vector 0 0 310) (send *desk* :pos))
(send *hampen* :set-color :white)

;;Set coordination.
(send *desk* :put :left-coords
      (make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *desk*))
;;Set coordination.
(send *needle* :put :left-coords
      (make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) ;;質問：parent座標をpr2に設定したら収束しなくなった ;;still confusing 

(send *pr2* :larm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)


;;Show Worldrecord
(setq *arrow* (arrow))
(send *arrow* :copy-worldcoords)

;;Show all objects in viewer. 
(objects (list *pr2* *desk* *needle* *hampen*)) ;;Dont forget to add all items



;Setting Initial Pose
(send *ri* :speak-jp "初期姿勢に戻ります" :wait t)
(send *ri* :start-grasp :arms :wait t)
(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
(unix:sleep 1)
(send *ri* :speak-jp "右手を開きます。スポンジをもたせてください。")
(send *ri* :stop-grasp :rarm :wait t)
(send *irtviewer* :draw-objects)
(unix:sleep 1)
(send *ri* :start-grasp :rarm :wait t)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
(unix:sleep 1)
(send *ri* :speak-jp "左手を開きます。針を持たせてください。")
(send *ri* :stop-grasp :larm :wait t)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
(unix:sleep 1)
(send *ri* :start-grasp :larm :wait t)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
(unix:sleep 1)
(send *ri* :speak-jp "これから針を持って作業します．離れてください．")
(unix:sleep 2)

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


;;Graspe needle
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
;(send *ri* :start-grasp :larm)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;(send *ri* :wait-interpolation)
;(send *irtviewer* :draw-objects)
;(unix:sleep 1)



;;Stitch with the left hand
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 0.0 -10.0 -10.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
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
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
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
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)



(send *pr2* :larm :wrist-p :joint-angle -80)
(send *pr2*
      :inverse-kinematics (send (send (send *needle* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector -35.0 -10.0 5.0))
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
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
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)



;;Switching hands
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)


(send *pr2* :rarm :inverse-kinematics
      (send (send *needle* :get :left-coords) :copy-worldcoords)
      :translate (float-vector -10.0 0 0)
      :move-target (send *needle* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
      :rotation-axis :x))
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)


;(send *ri* :stop-grasp :rarm)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;(send *ri* :wait-interpolation)
;(send *irtviewer* :draw-objects)


;;Grasping needle with right hand
(progn (send (send *needle* :copy-worldcoords) :draw-on :size 50) ;;debug
       (setq *next-needle-coords* (send (send *needle* :copy-worldcoords) :transform (make-coords :pos #f(0 0 80) :rpy (float-vector 0 0 0))))
       (send *next-needle-coords* :draw-on :flush t :size 100) ;; debug
       (send *pr2* :rarm :inverse-kinematics (send *next-needle-coords* :copy-worldcoords) :rotation-axis t :look-at-target t :debug-view t)
       )
;(send *pr2* :rarm :inverse-kinematics
;      (send (send *needle* :get :left-coords) :copy-worldcoords)
;      :rotation-axis :y) ;;yだと向きが違うき
(send (send *needle* :parent) :dissoc *needle*)
(send *pr2* :rarm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)

(progn (send (send *needle* :copy-worldcoords) :draw-on :size 50) ;;debug
       (setq *next-needle-coords* (send (send *needle* :copy-worldcoords) :transform (make-coords :pos #f(0 0 40) :rpy (float-vector 0 0 0))))
       (send *next-needle-coords* :draw-on :flush t :size 100) ;; debug
       (send *pr2* :rarm :inverse-kinematics (send *next-needle-coords* :copy-worldcoords) :rotation-axis t :look-at-target t :debug-view t)
       )
;(send *pr2* :rarm :inverse-kinematics
;      (send (send *needle* :get :left-coords) :copy-worldcoords)
;      :rotation-axis :y) ;;yだと向きが違うき
(send (send *needle* :parent) :dissoc *needle*)
(send *pr2* :rarm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)



(send *ri* :stop-grasp :larm)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)



;(send *pr2* :larm :inverse-kinematics
;      (send (send *desk* :get :left-coords) :copy-worldcoords)
;      :rotation-axis t)
;(send (send *needle* :parent) :dissoc *needle*)
;(send *pr2* :rarm :end-coords :assoc *needle*)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;(send *ri* :wait-interpolation)
;(send *irtviewer* :draw-objects)

(send *pr2* :rarm :inverse-kinematics
      (send (send *desk* :get :left-coords) :copy-worldcoords)
      :rotation-axis t)
(send (send *needle* :parent) :dissoc *needle*)
(send *pr2* :rarm :end-coords :assoc *needle*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)
;(send *irtviewer* :draw-objects)




;(send *pr2* :larm :inverse-kinematics
;      (send (send *needle* :get :left-coords) :copy-worldcoords)
;      :rotation-axis :z)
;(send *pr2* :larm :end-coords :assoc *needle*)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;(send *ri* :wait-interpolation)
;(send *irtviewer* :draw-objects)


;;how to dwaw coordination arrow
;(send (send (send *needle* :get :left-coords) :copy-worldcoords) :draw-on :flush t)

