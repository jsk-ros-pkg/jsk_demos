#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-broom-object.l")
(load "models/room610-scene.l")

(setq debug nil)

(room610)
(pr2)
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))
(if (not (boundp '*tfl*)) (setq *tfl* (instance transform-listener :init)))

(objects (list *room610* *pr2*))

(setq *room610-origin* "/eng8/6f/610")
(setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))

(setq *tray* (send *room610* :object "room610-tray"))

;; spots
(setq *tray-spot* (make-cascoords :pos #f(4650 1000 0) :rpy (list -pi/2 0 0)))
(send *room610* :assoc *tray-spot*)

(defmethod room610-scene
  (:spot-tray-spot () *tray-spot*)
  (:spots () (list *tray-spot*)))

(setq *world-to-610* (send *tfl* :lookup-transform "/world" *room610-origin* (ros::time 0)))

(defun y-tanaka-move-to-table ()
  (speak-jp "トレイの前に移動します。")
  (send *pr2* :reset-pose)
  (pr2-tuckarm-pose)
  (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
  (send *ri* :move-to *tray-spot* :frame-id *room610-origin*)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
)

(defun y-tanaka-pick-tray ()
  (warn "pick-tray (tanaka)~%")

  ; Reset
  (send *pr2* :reset-pose)
  ; (pr2-tuckarm-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ; (pick-tray)
  (speak-jp "トレイを持ちます。")
  
  ; Open gripper
  (send *ri* :stop-grasp :arms)
  (send *ri* :wait-interpolation)

  ; Rise his elbows prior to IK
  ; Use torso
  (send *pr2* :torso :waist-z :joint-angle 150)

  ; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)
  ; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)
  ; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ; IK
  (send *pr2* :inverse-kinematics
      ; (make-coords :pos #f(735.006 251.506 746.315))
      (send *tray* :handle-larm-handle)
      :rotation-axis nil
      :move-target (send *pr2* :larm :end-coords)
      :link-list (send *pr2* :link-list
                       (send *pr2* :larm :end-coords :parent)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
  (send *pr2* :inverse-kinematics
        ; (make-coords :pos #f(742.159 -220.170 758.206))
        (send *tray* :handle-rarm-handle)
        :rotation-axis nil
        :move-target (send *pr2* :rarm :end-coords)
        :link-list (send *pr2* :link-list
                         (send *pr2* :rarm :end-coords :parent)
                         (send *pr2* :torso_lift_link))
        :debug-view t)
  (send *pr2* :head :look-at 
        (apply 'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos))) 
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ; Grasp it
  (send *ri* :start-grasp :arms)
  (send *ri* :wait-interpolation)

  ; Hold it up!
  (send *tray* :translate #f(0 0 100) :world)
  (send *pr2* :inverse-kinematics
      ; (make-coords :pos #f(735.006 251.506 746.315))
      (send *tray* :handle-larm-handle)
      :rotation-axis nil
      :move-target (send *pr2* :larm :end-coords)
      :link-list (send *pr2* :link-list
                       (send *pr2* :larm :end-coords :parent)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
  (send *pr2* :inverse-kinematics
        ; (make-coords :pos #f(742.159 -220.170 758.206))
        (send *tray* :handle-rarm-handle)
        :rotation-axis nil
        :move-target (send *pr2* :rarm :end-coords)
        :link-list (send *pr2* :link-list
                         (send *pr2* :rarm :end-coords :parent)
                         (send *pr2* :torso_lift_link))
        :debug-view t)
  (send *pr2* :head :look-at
        (apply 'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ; Move it close to PR2
  (send *tray* :translate #f(0 250 0) :world)
    
    (send *pr2* :inverse-kinematics
      (send *tray* :handle-larm-handle)
      :rotation-axis nil
      :move-target (send *pr2* :larm :end-coords)
      :link-list (send *pr2* :link-list
                       ; (send *pr2* :larm :end-coords :parent)
                       (send *pr2* :l_elbow_flex_link)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
    (send *pr2* :inverse-kinematics
      (send *tray* :handle-rarm-handle)
      :rotation-axis nil
      :move-target (send *pr2* :rarm :end-coords)
      :link-list (send *pr2* :link-list
                       ; (send *pr2* :rarm :end-coords :parent)
                       (send *pr2* :r_elbow_flex_link)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
  (send *pr2* :head :look-at
        (apply 'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ; Speak
  (speak-jp "コーヒーをお持ちします。")
  ; (speak-jp "コーヒーをお持ちしますわ、ご主人様。")
)



