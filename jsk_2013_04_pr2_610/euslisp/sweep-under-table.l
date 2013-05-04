(defun sweep-under-table ()
  (warn "sweep-under-table (yukizaki)~%")
  (send *ri* :stop-grasp :rarm :wait t)
  (let (z-axis)
    (when nil ;; ?? debug
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq z-axis (if (gripper-z-axis-inversed-p :larm) 0 180))
      (send *ri* :stop-grasp :rarm)
      (send *pr2* :reset-pose)
      (send *pr2* :inverse-kinematics
            (make-coords :pos (send *pr2* :transform-vector #f(800 600 900)) :rpy (float-vector 0 0 (deg2rad z-axis)))
            :rotation-axis t
            :move-target (send *pr2* :larm :end-coords)
            :use-torso t
            :debug-view nil)
      (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
      (send *ri* :wait-interpolation)


      (send *ri* :stop-grasp :larm)
      (unix::sleep 2)
      (send *ri* :wait-interpolation)
      (send *pr2* :larm :move-end-pos #f(-500 0 0))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
      (send *ri* :wait-interpolation)
      (send *ri* :start-grasp :larm)
      (send *ri* :wait-interpolation)
      )

    (setq z-axis (if (gripper-z-axis-inversed-p :larm) 0 180))



#|
(setq debug nil)


 (warn "right-hand~%")
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))    
(send *pr2* :rarm :inverse-kinematics
          (make-coords :pos (send *pr2* :transform-vector #f(330 -260 550))
                       :rpy (list (deg2rad 30) 0 0))
          :rotation-axis :z
	  :revert-if-fail nil
          :debug-view debug)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)


    (send *pr2* :rarm :inverse-kinematics
          (make-coords :pos (send *pr2* :transform-vector #f(530 60 850))
                       :rpy (list (deg2rad 70) 0 0))
          :rotation-axis :z
	  :revert-if-fail nil
          :debug-view debug)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm)
    (send *ri* :stop-grasp :larm)
    (send *pr2* :larm :end-coords :dissoc *broom*)

    (warn "calib-pick-height~%")
    (let ((offset-l (send *pr2* :rotate-vector #f(0 -30 -500)))
          (offset-r #f(0 0 -500))
          (handle-larm (send (send *broom* :handle-larm) :copy-worldcoords))
          (handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)))
      (print offset-l)
      (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l :world)
            :look-at-target t
            :rotation-axis :z
            :debug-view debug)
      (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r :world)
            :look-at-target nil
            :rotation-axis :z
            :debug-view debug))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
                                        ;    (send *pr2* :rarm :move-end-pos #f(20 0 0))
                                        ;    (send *ri* :angle-vector (send *pr2* :angle-vector))
                                        ;    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)

;    (send *pr2* :torso :waist-z :joint-angle 100)
    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos (send *pr2* :rotate-vector #f(0 0 -230)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm)
|#



    ;;sweep under table
    (dotimes (i 3)
      ;;   (send *pr2* :angle-vector start-sweep)
      (send *pr2* :inverse-kinematics
            (make-coords :pos (send *pr2* :transform-vector #f(140 600 700)) :rpy (float-vector -0.2 -0.4 (deg2rad z-axis)))
            :rotation-axis t
            :move-target (send *pr2* :larm :end-coords)
            :link-list (send *pr2* :link-list
                             (send *pr2* :larm :end-coords :parent)
                             (send *pr2* :torso_lift_link))
            :debug-view nil)
      (send *pr2* :look-at-hand :larm)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
      (send *ri* :wait-interpolation)
      ;;   (send *pr2* :angle-vector end-sweep)
      (send *pr2* :inverse-kinematics
            (make-coords :pos (send *pr2* :transform-vector (float-vector 640 (- 500 (* 150 i)) 500)) :rpy (float-vector 0 -0.7 (deg2rad z-axis)))
            :rotation-axis t
            :move-target (send *pr2* :larm :end-coords)
            :link-list (send *pr2* :link-list
                             (send *pr2* :larm :end-coords :parent)
                             (send *pr2* :torso_lift_link))
            :debug-view nil)
      (send *pr2* :look-at-hand :larm)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
      (send *ri* :wait-interpolation)
      )

    (return-from sweep-under-table)

    ;;両手で
    ;;make broom model
    (setq *broom2* (room73b2-broom))
    (objects (list *pr2* *broom2*))
    (send *pr2* :assoc *broom2*)
    (send *broom2* :locate #f(600 0 500) :parent)
    (send *broom2* :orient (deg2rad 0) :y :parent)
    ;;手首を正しく
    (setq *handle-l* (make-coords :coords (send (car (send *broom2* :handle)) :worldcoords)))
    (send *handle-l* :rotate (deg2rad z-axis) :x)

    ;;左手を前に
    (send *pr2* :reset-pose)
    (send *pr2* :inverse-kinematics
          *handle-l*
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)

    (send *broom2* :locate #f(600 0 0) :parent)
    (setq *handle-l* (make-coords :coords (send (car (send *broom2* :handle)) :worldcoords)))
    (send *handle-l* :rotate (deg2rad z-axis) :x)
    (send *pr2* :reset-pose)
    (send *pr2* :inverse-kinematics
          *handle-l*
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    ;;右手を開いて前に
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)
    (unix::sleep 2)
    (send *pr2* :inverse-kinematics
          (car (cdr (send *broom2* :handle)))
          :rotation-axis t
          :move-target (send *pr2* :rarm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *pr2* :rarm :move-end-pos #f(60 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (unix:sleep 2)

    ;;右手でつかむ
    (send *ri* :start-grasp :rarm)
    (unix:sleep 5)

    ;;左手掴み直し
    (send *ri* :stop-grasp :larm)
    (send *pr2* :larm :move-end-pos #f(-200 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (unix:sleep 2)
    (send *pr2* :inverse-kinematics
          (car (send *broom2* :handle))
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (unix:sleep 2)
    (send *ri* :start-grasp :larm)
    (unix:sleep 5)

    (send *broom2* :locate #f(900 0 0) :parent)
    (send *broom2* :orient (deg2rad -30) :y :parent)
    ;;sweep start
    (send *pr2* :reset-pose)
    (send *pr2* :inverse-kinematics
          (car (cdr (send *broom2* :handle)))
          :rotation-axis t
          :move-target (send *pr2* :rarm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *pr2* :inverse-kinematics
          (car (send *broom2* :handle))
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)

    (send *broom2* :locate #f(1200 0 -150) :parent)
    (send *broom2* :orient (deg2rad -55) :y :parent)
    ;;sweep end
    (send *pr2* :reset-pose)
    (send *pr2* :inverse-kinematics
          (car (cdr (send *broom2* :handle)))
          :rotation-axis t
          :move-target (send *pr2* :rarm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *pr2* :inverse-kinematics
          (car (send *broom2* :handle))
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)

    ))
