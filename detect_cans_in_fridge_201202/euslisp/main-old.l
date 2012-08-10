;; open fridge and pick a can (and close the door)

(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")
(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  (setq *obj* (make-sphere 100)) ;; dummy object
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  ;;(objects (list *pr2*))
  )

(defun demo (&key (type "georgia"))
  ;;(init)
  (pr2-tuckarm-pose :rarm)
  (setq *opose* (send *ri* :state :worldcoords))
  ;; move-to front of fridge
  (setq co (send *scene* :spot "/eng2/7f/room73B2-fridge-front"))
  (send co :translate #f(50 0 0) :world) ;; for opening door
  (send *ri* :move-to co)
  ;;
  (send *pr2* :head :angle-vector #f(0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)

  ;; open door
  (let (ret)
    (dotimes (i 3 nil)
      ;; finding handle position
      (check :type "fridge")
      ;; opening door
      (warn ";; print check frig~%")
      (dump-structure *standard-output*
                      (list (send *pr2* :copy-worldcoords)
                            (send *obj* :copy-worldcoords)))
      (when (setq ret (open-fridge :rarm *obj* (/ pi 2)))
        (return)))
    (when (not ret)
      (return-from demo nil))
    )

  ;; free door handle
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq rend (send *pr2* :rarm :end-coords :copy-worldcoords))
  (send rend :translate #f(-100 -50 0) *pr2*)
  (send *pr2* :rarm :inverse-kinematics rend)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :stop-grasp :rarm)

  ;; ?? open door ()
  (send *ri* :angle-vector-sequence
	(list #f(50  5 74 100 -90 -70 -6 20  -70 50 -59 -43 118 0 0 -2 31)
	      #f(50  5 74 100 -90 -70 -6 20  -0 60 -20 -90 118 -90 0 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -45 26 -31 -62 22 -76 121 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31))
	(list 1500 1000 1000 3000 3000))
  (send *ri* :wait-interpolation)

  ;; move-to spot for grasping
  (ros::spin-once)
  (send *ri* :go-pos-unsafe 0 0 (rad2deg -1.3))
  (send *ri* :go-pos-unsafe 0.40 -0.15 0)

  (send *pr2* :head :angle-vector #f(90 10))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  ;; detect cans which was indicated by ( type )
  (check :type type :tf-force t :timeout 30)
  (warn ";; print check cans~%")
  (dump-structure *standard-output*
                  (list (send *pr2* :copy-worldcoords)
                        (send *obj* :copy-worldcoords)))

  ;;(objects *obj*) ;; *obj* -> position
  (send *ri* :stop-grasp :larm)

  ;; *obj* -> can position
  (let (pre1 pre via via-av tar)
    (setq pre #f(250 87 65 14 -122 61 -115 170 -5 74 -104 -90 70 -6 20 90 20)
	  pre1 #f(50 87 65 14 -122 61 -115 170 -5 74 -104 -90 70 -6 20 90 20))
    ;; traject: pre1 -> pre -> via -> tar -> ( grasp ) -> via -> pre
    (send *pr2* :angle-vector pre)
    (send *ri* :angle-vector pre1)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector pre)
    (send *ri* :wait-interpolation)
    ;;
    (setq via (send *obj* :copy-worldcoords))
    (send via :translate #f(0 -200 0) *pr2*)
    (send via :orient pi/2 :z *pr2*)
    (or (send *pr2* :larm :inverse-kinematics via :rotation-axis t)
	(send *pr2* :larm :inverse-kinematics via :rotation-axis nil))
    (setq via-av (send *pr2* :angle-vector))
    (send *ri* :angle-vector via-av)
    (send *ri* :wait-interpolation)
    ;;
    (setq tar (send *obj* :copy-worldcoords))
    (send tar :translate #f(0 30 0) *pr2*) ;; offset for grasping
    (send tar :orient pi/2 :z *pr2*)
    (send *pr2* :larm :inverse-kinematics tar :rotation-axis t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; grasp can
    (send *ri* :start-grasp :larm)
    ;;
    (send *ri* :angle-vector via-av)
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :angle-vector pre)
    (send *ri* :wait-interpolation)
    )
  (pr2-tuckarm-pose :rarm)

  ;; close the door of fridge
  (send *ri* :go-pos-unsafe 0 0 90)
  (send *ri* :angle-vector-sequence
	(list #f(250 5 74 100 -90 -70 -6 20 -13 50 -59 -43 118 -90 62 -2 31)
	      #f(200 5 74 100 -90 -70 -6 20 -50 36 -29 -43 118 -90 62 -2 31)
	      #f(150 5 74 100 -90 -70 -6 20 -63 10 -39 -33 -49 -32 89 -2 32)
	      #f(100 5 74 100 -90 -70 -6 20 -61 6 -74 -64 -35 -6 102 -2 33)
	      #f(50 5 74 100 -90 -70 -6 20 3 -2 -64 -80 -24 -55 10 -2 33))
	(list 2000 1000 3000 2000 2000))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :larm)

  ;; move-to user and put
  (send *ri* :move-to *opose*)
  (hand-over :larm)
  )

(defun wait-query ()
  (setq *type* nil)
  (ros::subscribe "/query" std_msgs::String
		  #'(lambda(m)(setq *type* (send m :data))))
  (do-until-key
   (ros::spin-once)
   (when *type*
     (demo *type*)
     (ros::unsubscribe "/query")
     (return-from wait-query t))
   (ros::sleep))
  )

(warn ";; (init)~%")
(warn ";; (demo)~%")
(warn ";; (demo \"georgia\")~%")
(warn ";; (demo \"mets\")~%")
(warn ";; (demo \"mountain_dew\")~%")
