;; (progn (load "sweep-under-table.l")(sweep-under-table-init)(sweep-under-table))
(defun sweep-under-table-init ()
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :angle-vector #f(100.0 10.6748 2.30296 30.0196 -97.4113 107.346 -62.9762 171.355 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 -3.07712 54.4601))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  )



(defun sweep-under-table-init-slim-old ()
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :angle-vector #f(126.803 -0.739814 -20.2449 11.8343 -69.8237 85.3014 -69.1725 178.026 -6.41069 -11.7687 -5.12628 -78.6576 -75.8367 -92.5412 1.35363 0 12.2682))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  )




(defun sweep-under-table-init-slim ()
  (send *pr2* :angle-vector #f(126.795 -1.00107 -20.2206 11.9353 -71.4577 93.1963 -72.7922 176.55 -2.71504 -5.92811 -3.83082 -83.875 -79.9001 -86.0572 -191.652 0 12.2682))
  (send *pr2* :rarm :move-end-pos #f(-100 0 0))
;;  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 1000 :larm :rotation-axis :z)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :rarm :move-end-pos #f(100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  )



(defun rehold-mop ()
  ;;

  ;; open grasp
  (send *ri* :move-gripper :rarm 0.03 :effort 20)
  (send *ri* :move-gripper :larm 0.03 :effort 20)
  (send *ri* :wait-interpolation)

  ;; down 200 mm
#|
  (send *pr2* :inverse-kinematics (list 
                                   (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 -200) (send *pr2* :worldcoords))
                                   (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 -200) (send *pr2* :worldcoords))
                                   )
        :link-list (list
                    (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                    (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                    )
        :rotation-axis (list t t)
        :move-target (list (send *pr2* :rarm :end-coords) (send *pr2* :larm :end-coords))
        :revert-if-fail nil
        :look-at-target t
        :debug-view nil
        )
|#

  ;; down 200 mm
  (setq av0 (send *pr2* :angle-vector))
  (send *pr2* :torso :waist-z :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 50))
  (send *pr2* :arms :move-end-pos #f(0 0 -600) :world)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
#|
  (send *pr2* :inverse-kinematics (list 
                                   (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 -200) (send *pr2* :worldcoords))
                                   (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 -200) (send *pr2* :worldcoords))
                                   )
        :link-list (list
                    (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                    (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                    )
        :rotation-axis (list t t)
        :move-target (list (send *pr2* :rarm :end-coords) (send *pr2* :larm :end-coords))
        :revert-if-fail nil
        :look-at-target t
        :debug-view nil
        )
  (setq av1 (send *pr2* :angle-vector))
  (send *ri* :angle-vector-sequence (list av0 av1) (list 1200 1200))
  (send *ri* :wait-interpolation)
|#
  (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :rarm :rotation-axis :z)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp)
  (send *ri* :start-grasp) ;; make sure to grasp

#|
  (send *pr2* :inverse-kinematics (list 
                                   (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 200) (send *pr2* :worldcoords))
                                   (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 200) (send *pr2* :worldcoords))
                                   )
        :link-list (list
                    (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                    (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                    )
        :rotation-axis (list t t)
        :move-target (list (send *pr2* :rarm :end-coords) (send *pr2* :larm :end-coords))
        :revert-if-fail nil
        :look-at-target t
        :debug-view nil
        )
  (setq av0 (send *pr2* :angle-vector))
  (send *pr2* :inverse-kinematics (list 
                                   (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 200) (send *pr2* :worldcoords))
                                   (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector  0 0 200) (send *pr2* :worldcoords))
                                   )
        :link-list (list
                    (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                    (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                    )
        :rotation-axis (list t t)
        :move-target (list (send *pr2* :rarm :end-coords) (send *pr2* :larm :end-coords))
        :revert-if-fail nil
        :look-at-target t
        :debug-view nil
        )
  (setq av1 (send *pr2* :angle-vector))
  (send *ri* :angle-vector-sequence (list av0 av1) (list 1200 1200))
  (send *ri* :wait-interpolation)
|#
  (send *pr2* :arms :move-end-pos #f(0 0 600) :world)
  (send *pr2* :torso :waist-z :joint-angle 150)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :rarm :rotation-axis :z)
  ;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)


  )


(defun sweep-under-table ()
  (let (c0 c1 av-grasp av-sweep0 av-sweep1)
    (warn "sweep-under-table (yukizaki)~%")
    (send *ri* :stop-grasp :rarm :wait t)

    ;; make sure that we start from same situation
    (send *pr2* :angle-vector #f(126.795 -1.00107 -20.2206 11.9353 -71.4577 93.1963 -72.7922 176.55 -2.71504 -5.92811 -3.83082 -83.875 -79.9001 -86.0572 -191.652 0 12.2682))
    ;;(setq before-sweep-left-hand (send (send *pr2* :larm :end-coords) :copy-worldcoords))

    ;; grasp pose
    (setq c0 (make-coords :pos (send *pr2* :transform-vector #f(700 600 1200)) :rpy (float-vector 0 0 0)))
    (send *pr2* :inverse-kinematics
          c0
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :look-at-target t
          :debug-view nil)
    (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 -300 0) :world)
          :revert-if-fail nil
          :rotation-axis t
          :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
          :move-target (list (send *pr2* :rarm :end-coords))
          :debug-view nil
          )

    (setq av-grasp (send *pr2* :angle-vector))
    (send *ri* :angle-vector av-grasp 1000)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-rarm-free*))

    ;; special command for casing grasp
    (send *ri* :move-gripper :larm 0.02 :effort 20 :wait t)

    ;; grasp pose
    ;; tmp code until https://sourceforge.net/p/jsk-ros-pkg/tickets/91/ fixed
    ;;(send *pr2* :larm :move-end-pos #f(0 0 -500))
    ;;(send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *pr2* :larm :move-end-pos #f(0 0 -500))
    (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2600 :larm :rotation-axis :z)
    (send *ri* :wait-interpolation)
    ;;
#|
    (send c0 :translate #f(0 0 -250) :world)
    (send *pr2* :inverse-kinematics
          c0
          :rotation-axis t
          :move-targe (send *pr2* :larm :end-coords)
          :use-torso t
          :look-at-target t
          :debug-view nil)
    (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    (setq av0 (send *pr2* :angle-vector))
    (send c0 :translate #f(0 0 -250) :world)
    (send *pr2* :inverse-kinematics
          c0
          :rotation-axis t
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :look-at-target t
          :debug-view nil)
    (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    (setq av1 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence (list av0 av1) (list 800 800))
    (send *ri* :wait-interpolation)
|#

    ;;sweep under table
    ;; ready pose
    (setq c0 (make-coords :pos (send *pr2* :transform-vector #f(140 600 700)) :rpy (float-vector -0.2 -0.4 0)))
    (setq c1 (make-coords :pos (send *pr2* :transform-vector (float-vector 250 500 500)) :rpy (float-vector 0 -0.7 0)))
    (send *pr2* :inverse-kinematics
          c0
          :rotation-axis t
          :revert-if-fail nil
          :move-target (send *pr2* :larm :end-coords)
          :use-torso t
          :look-at-target t
          :debug-view nil)
    (setq av-sweep0 (send *pr2* :angle-vector))

    ;; sweep pose
    (send *pr2* :inverse-kinematics
          c1
          :rotation-axis t
          :revert-if-fail nil
          :move-target (send *pr2* :larm :end-coords)
          :use-torso nil
          :look-at-target t
          :debug-view nil)
    (setq av-sweep1 (send *pr2* :angle-vector))

    ;; ready and grasp bloom
    ;;(send *pr2* :angle-vector av0)
    ;;(send *ri* :angle-vector av0 2500)
    ;; tmp code until https://sourceforge.net/p/jsk-ros-pkg/tickets/91/ fixed
    (let (tmp-av)
      (send *pr2* :inverse-kinematics
            (midcoords 0.5 c0 c1)
            :rotation-axis t
            :revert-if-fail nil
            :move-target (send *pr2* :larm :end-coords)
            :use-torso nil
            :look-at-target t
            :debug-view nil)
      (setq tmp-av (send *pr2* :angle-vector))
      (send *ri* :angle-vector-sequence (list tmp-av av-sweep0) (list 1250 1250)))
    (send *ri* :wait-interpolation)
    ;;(send *ri* :start-grasp :larm)
    ;; grsip tight
    (send *ri* :move-gripper :larm 0.0 :effort 40 :wait t) ;; default effor is 20
    (send *ri* :move-gripper :larm 0.0 :effort 40 :wait t) ;; make sure to grasp tight

    ;; start sweep
    (dotimes (i 3)
      ;;(send *ri* :angle-vector-with-constraint av-sweep1 1200 :larm :rotation-axis :z) (send *ri* :wait-interpolation)
      ;;(send *ri* :angle-vector-with-constraint av-sweep0 1200 :larm :rotation-axis :z)
      (send *ri* :angle-vector-sequence (list av-sweep1 av-sweep0) (list 1200 1200))
      (send *ri* :wait-interpolation))

    (send *pr2* :angle-vector av-grasp)
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 1800 :larm :rotation-axis :z)
    (send *ri* :wait-interpolation)

    (sweep-under-table-init-slim)
    (rehold-mop)
    ))

(defun sweep-under-table-dual-arm ()
  ;; we assume the robot aready in broon grasp pose, see (sweep-under-table-init)
  (let (av0 av1
            handle-larm handle-rarm offset-l offset-r
            av-grasp av-sweep0 av-sweep1)
  (use-tilt-laser-obstacle-cloud nil)

    (send *ri* :go-pos-unsafe -1.5 0 90)
    (setq av-grasp (send *pr2* :angle-vector))
    ;; 両手を前に
    (send *pr2* :larm :move-end-pos #f(200 0 0) *pr2*)
    (send *pr2* :rarm :move-end-pos #f(200 0 0) *pr2*)
    (setq av0 (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
    (send *ri* :wait-interpolation)

    ;; down both hand
    (send *broom* :move-to (make-coords :pos (transform #2f((1 0 0)(0 1 0)(0 0 0)) (send *pr2* :larm :end-coords :worldpos)) :rot (send *pr2* :worldrot)) :world)

    (setq handle-larm (send (send *broom* :handle-larm) :copy-worldcoords)
          handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)
          offset-l #f(0 0 -200)
          offset-r #f(0 0 -200))
    (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l)
          :look-at-target t
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r)
          :look-at-target nil
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    ;; special command for casing grasp
    (send *ri* :move-gripper :arms 0.02 :effort 20)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)

    ;; ready pose
    (send *broom* :orient (deg2rad -20) :y) ;; -40
    (send *broom* :orient (deg2rad -20) :x)
    (setq handle-larm (send (send *broom* :handle-larm) :copy-worldcoords)
          handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)
          offset-l #f(0 0 -200)
          offset-r #f(0 0 -200))

    (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l)
          :look-at-target *broom*
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r)
          :look-at-target nil
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (setq av-sweep0 (send *pr2* :angle-vector))
    ;; tmp code until https://sourceforge.net/p/jsk-ros-pkg/tickets/91/ fixed
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
                                        ;(setq av1 (send *pr2* :angle-vector))
                                        ;(dotimes (i 50) (send *pr2* :angle-vector (midpoint (/ i 50.0) av0 av1))(send *irtviewer* :draw-objects))    (send *ri* :angle-vector-sequence (list (midpoint 0.5 av0 av1) av1) (list 2000 2000))
    (send *ri* :wait-interpolation)
    ;; grsip tight
    (send *ri* :move-gripper :rarm 0.0 :effort 40 :wait t) ;; default effor is 20


    ;; sweep motion
    (send *broom* :translate #f(300 -300 0) *pr2*)
    (setq handle-larm (send (send *broom* :handle-larm) :copy-worldcoords)
          handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)
          offset-l #f(0 0 -100)
          offset-r #f(0 0 -200))

    (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l)
          :look-at-target nil
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r)
          :look-at-target *broom*
          :rotation-axis :z
          :rotation-axis t
          :debug-view nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (setq av-sweep1 (send *pr2* :angle-vector))
    ;; special command for casing grasp
;;    (send *ri* :angle-vector av-sweep0 2000)
;;    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector av-sweep1 2000)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(   0 0 -100))
    (send *pr2* :rarm :move-end-pos #f(   0 0 -100))
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 800 :arms :rotation-axis :z)
    (send *ri* :wait-interpolation)


    ;;go to  kitchen
    (setq goal_coords (make-coords :pos #f(5050 1000 0)))
    (setq pr2_now_coords (send *ri* :state :worldcoords *room610-origin*))
    (setq move-diff (send pr2_now_coords :transformation goal_coords))
    (send *ri* :go-pos-unsafe (* 0.001 (elt (send move-diff :pos) 0)) (* 0.001 (elt (send move-diff :pos) 1)) (rad2deg (elt (car (send move-diff :rpy-angle)) 0)))

    ;;go to  kitchen
    (setq goal_coords (make-coords :pos #f(6600 1000 0)))
    (setq pr2_now_coords (send *ri* :state :worldcoords *room610-origin*))
    (setq move-diff (send pr2_now_coords :transformation goal_coords))
    (send *ri* :go-pos-unsafe (* 0.001 (elt (send move-diff :pos) 0)) (* 0.001 (elt (send move-diff :pos) 1)) (rad2deg (elt (car (send move-diff :rpy-angle)) 0)))

    ;;go to  kitchen
    (setq goal_coords (make-coords :pos #f(6600 1000 0) :rpy #f(-2.4 0 0)))
    (setq pr2_now_coords (send *ri* :state :worldcoords *room610-origin*))
    (setq move-diff (send pr2_now_coords :transformation goal_coords))
    (send *ri* :go-pos-unsafe (* 0.001 (elt (send move-diff :pos) 0)) (* 0.001 (elt (send move-diff :pos) 1)) (rad2deg (elt (car (send move-diff :rpy-angle)) 0)))

    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms)
    (send *ri* :wait-interpolation)

    (speak-jp "部屋の掃除が終わりました")


   ;; (dotimes (i 5)
;;      (send *ri* :angle-vector-sequence (list av-sweep1 av-sweep0) (list 1200 1200))
;;      (send *ri* :wait-interpolation))
;;    (send *ri* :angle-vector av-grasp 1000)
;;    (send *ri* :wait-interpolation)
    ))
