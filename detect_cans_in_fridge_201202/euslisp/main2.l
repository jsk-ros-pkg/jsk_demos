;; open fridge and pick a can (and close the door)

(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")
(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  (unless (boundp '*tfb*)
    (setq *tfb* (instance ros::transform-broadcaster :init)))
  (setq *obj* (make-sphere 100)) ;; dummy object
  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  ;;(objects (list *pr2*))
  )

(defun grasp-can (obj)
  (let (via1 via2 tar orig-av)
    (send *ri* :stop-grasp :rarm)
    (send *tfb* :send-transform
	  obj "/base_footprint" "/objecta_pose_frame")
    (setq orig-av (send *ri* :state :potentio-vector))
    (setq tar (send obj :copy-worldcoords))
    (send tar :orient 0 :z :world)
    (setq via1 (send tar :copy-worldcoords))
    (send via1 :translate #f(-100 0 0) :world)
    (setq via2 (send tar :copy-worldcoords))
    (send via2 :translate #f(-50 0 0) :world)
    (setq via3 (send tar :copy-worldcoords))
    (send via3 :translate #f(30 0 0) :world)
    (send *tfb* :send-transform
	  via1 "/base_footprint" "/objecta_pose_frame_via1")
    (send *tfb* :send-transform
	  via2 "/base_footprint" "/objecta_pose_frame_via2")
    (send *tfb* :send-transform
	  via3 "/base_footprint" "/objecta_pose_frame_via3")
    ;;
    (send *pr2* :rarm :inverse-kinematics via1 :rotation-axis t :use-torso t)
    (send *tfb* :send-transform
	  (send *pr2* :rarm :end-coords)  "/base_footprint" "/objecta_pose_end_coords")
    (setq daa (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;
    ;;(send *ri* :go-velocity 0.1 0 0 1000)
    ;;(send *pr2* :rarm :inverse-kinematics tar :rotation-axis t :use-torso t)
    (send *pr2* :rarm :inverse-kinematics via3 :rotation-axis t :use-torso t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (setq isgrasp (< 10 (send *ri* :start-grasp :rarm)))
    ;;
    (send *pr2* :rarm :inverse-kinematics via1 :rotation-axis t :use-torso t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :go-velocity -0.1 0 0 1000)
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :angle-vector orig-av)
    (send *ri* :wait-interpolation)
    )
  )

(defun demo (&key (type "georgia"))
  ;;(init)
  (pr2-tuckarm-pose :rarm)
  (setq *opose* (send *ri* :state :worldcoords))
  ;; move-to front of fridge
  (setq co (send *scene* :spot "/eng2/7f/room73B2-fridge-front"))
  (send co :translate #f(0 0 0) :world) ;; for opening door
  (send *ri* :move-to co)
  ;;
  (send *pr2* :head :angle-vector #f(0 0))
  (send *pr2* :torso :angle-vector #f(50))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)

  ;; open door
  (let (ret)
    (dotimes (i 10 nil)
      ;; finding handle position
      (check :type "fridge")
      ;; opening door
      (warn ";; print check frig~%")
      (dump-structure *standard-output*
                      (list (send *pr2* :copy-worldcoords)
                            (send *obj* :copy-worldcoords)))
      ;;(setq diff (/ (- 650 (elt (send *obj* :worldpos) 0)) -1000))
      ;;(when (< diff 0.1)
      ;;  (send *ri* :go-velocity diff 0 0 1000))
      (ros::spin-once)
      (when (setq ret (open-fridge :rarm *obj* (/ pi 2)))
        (return))
      (speak-jp "こまった") (speak-jp ".") (unix::sleep 3)
      )
    (when (not ret)
      (format t "(demo2 :type \"~a\")" type)
      (return-from demo nil))
    )
  (demo2 :type type))

(defun demo2 (&key (type "georgia"))
  ;; free door handle
  (ros::spin-once)
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq rend (send *pr2* :rarm :end-coords :copy-worldcoords))
  (send rend :translate #f(-100 -50 0) *pr2*)
  (send *pr2* :rarm :inverse-kinematics rend)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  ;; ?? open door ()
  (send *ri* :angle-vector-sequence
	(list #f(50 5 74 100 -90 -70 -6 20 -70 50 -59 -43 118 0 0 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -0 60 -20 -90 118 -90 0 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31)
	      #f(50 5 74 100 -90 -70 -6 20 -45 26 -31 -62 22 -76 121 -2 31)
	      #f(150 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31))
	(list 1500 1000 1000 3000 3000))
  (send *ri* :wait-interpolation)

  ;; (send *pr2* :angle-vector #f(200 5.00529 74.2038 105.001 -90.0035 -70.0071 -5.73378 19.9988 9.96029 39.7801 -45.0972 -121.542 94.4262 -111.922 63.0666 -0.0316 21.024))
  (send *pr2* :angle-vector #f(199.971 5.30455 72.1244 105.231 -88.5188 -69.9972 -5.72958 19.9717 24.6812 36.1934 10.6903 -121.497 159.113 -79.5111 -2.92658 -1.35562 26.862))
  (send *ri* :angle-vector (send *pr2* :angle-vector))

  ;; move-to spot for grasping
  (ros::spin-once)
  (send *ri* :go-pos-unsafe 0 0 (rad2deg 0.40))
  (send *ri* :go-pos-unsafe 0.15 -0.30 0)

  (send *ri* :wait-interpolation)

  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (dotimes (trial 100)
    ;; detect cans which was indicated by ( type )
    (when (not (check :type type))
      (check :type type :tf-force t :timeout 30))
    (warn ";; print check cans~%")
    (dump-structure *standard-output*
		    (list (send *pr2* :copy-worldcoords)
			  (send *obj* :copy-worldcoords)))
    (unless (boundp '*tfb*)
      (setq *tfb* (instance ros::transform-broadcaster :init)))
    (send *tfb* :send-transform
	  *obj* "/base_footprint" "/object_pose_frame")

    (grasp-can *obj*)
    (setq isgrasp (< 10 (send *ri* :start-grasp :rarm)))
    (when isgrasp
      (return)))

  ;; close the door of fridge
  (send *ri* :go-pos-unsafe 0 0 -20)
  (send *ri* :go-velocity -0.1 0.10 0 1500)
  (send *ri* :angle-vector-sequence
	(list #f(250 5 74 100 -90 -70 -6 20 -13 50 -59 -43 118 -90 62 -2 31)
	      #f(200 5 74 100 -90 -70 -6 20 -50 36 -29 -43 118 -90 62 -2 31)
	      #f(150 5 74 100 -90 -70 -6 20 -63 10 -39 -33 -49 -32 89 -2 32)
	      #f(100 5 74 100 -90 -70 -6 20 -61 6 -74 -64 -35 -6 102 -2 33)
	      #f(50 5 74 100 -90 -70 -6 20 3 -2 -64 -80 -24 -55 10 -2 33))
	(list 2000 1000 3000 2000 2000))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)

  ;; move-to user and put
  (send *ri* :go-pos-unsafe 0 0 -90)
  ;;(send *ri* :move-to *opose*)
  (hand-over :rarm :wait-shock t)
  )

(defun wait-query ()
  (setq *type* nil)
  (ros::subscribe "/query" std_msgs::String
		  #'(lambda(m)(setq *type* (send m :data))))
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
(warn ";; (demo :type \"georgia\")~%")
(warn ";; (demo :type \"iemon\")~%")
(warn ";; (demo :type \"wonda\")~%")

;; (progn (check :type "wonda")(grasp-can *obj*))