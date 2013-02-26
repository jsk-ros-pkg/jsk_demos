(warn ";; define grasp-can-single")
(defun grasp-can-single (obj &key (rotation 0))
  (let (via1 via2 tar orig-av)
    (send *ri* :stop-grasp :rarm)
    (send *tfb* :send-transform
          obj "/base_footprint" "/objecta_pose_frame")
    (setq orig-av (send *ri* :state :potentio-vector))

    (when *use-arm-navigation*
      (dotimes (i 5)
        (send *plan-env* :get-planning-scene)
        ;; (unix::usleep (* 400 1000))
        ))

    (setq tar (send obj :copy-worldcoords))
    (send tar :orient 0 :z :world)

    (setq via1 (send tar :copy-worldcoords))
    (send via1 :translate (float-vector (* (cos (deg2rad rotation)) -100)
                                        (* (sin (deg2rad rotation))  100) 0) :world)

    ;; via2
    (setq via2 (send tar :copy-worldcoords))
    (send via2 :translate (float-vector 30 0 0) :world)

    ;; publish tf
    (send *tfb* :send-transform
          via1 "/base_footprint" "/objecta_pose_frame_via1")
    ;;(send *tfb* :send-transform
    ;;via2 "/base_footprint" "/objecta_pose_frame_via2")
    (send *tfb* :send-transform
          via2 "/base_footprint" "/objecta_pose_frame_via2")

    ;; move to via1
    (send *pr2* :rarm :inverse-kinematics via1 :rotation-axis t :use-torso t)
    (send *tfb* :send-transform
          (send *pr2* :rarm :end-coords)  "/base_footprint" "/objecta_pose_end_coords")

    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation))
    ;; check IK

    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
      (ros::ros-warn ";; can not solve collision free IK")
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation))
      ))

    ;; move to via2 (grasping pose)
    (send *pr2* :rarm :inverse-kinematics via2 :rotation-axis t :use-torso t)
    ;; check IK
    #|(cond
    ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
    (ros::ros-warn ";; can not solve collision free IK")
    ;; move to via1
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    (t
    (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
    (ros::ros-warn ";; can not solve collision free Plan")
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    ))|#
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; grasp
    (send *ri* :start-grasp :rarm)

    ;; move to via1
    (send *pr2* :rarm :inverse-kinematics via1 :rotation-axis t :use-torso t)
    ;; check IK
    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
      (ros::ros-warn ";; can not solve collision free IK")
      ;; move to via1
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation))
      ))
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation)

    ;; move to original pose
    (send *ri* :angle-vector orig-av)
    (send *ri* :wait-interpolation)
    )
  )

(warn ";; define open-fridge-door")
(defvar *fridge-distance-threshold* 25.0)
(defvar *fridge-rotation-threshold* 0.09)
(defun open-fridge-door ()
  (let (ret
	;;(idealcds (make-coords :pos (float-vector 733.757 135.013 1092.946)
	;;:rpy (float-vector 0.002 0.059 -0.019)))
	(idealcds (make-coords :pos (float-vector 760 100 0)
			       :rpy (float-vector 0.05 0 0)))
	(diffcds (make-coords :pos (float-vector 10000 10000 10000)))
	cds)

    (dotimes (i 10 nil)

      ;; finding handle position
      (setq cds (check-detection :type "fridge" :speak-name "れいぞうこ"))
      ;; need detection fail check

      (when cds
	(send *obj* :reset-coords)
	(send *obj* :transform cds)
	(dump-structure *standard-output*
			(list (send *pr2* :copy-worldcoords)
			      (send cds :copy-worldcoords)))
	(setq cds
	      (make-coords :pos
			   (float-vector (elt (send cds :worldpos) 0)
					 (elt (send cds :worldpos) 1) 0)
			   :rpy (list (elt (car (rpy-angle (send cds :worldrot))) 0)
				      0 0)))

	(setq diffcds (send (send idealcds :inverse-transformation)
                            :transform (send cds :worldcoords) :world))

	(warn "~%~A -> ~A / ~A~%" diffcds cds idealcds)
	(ros::ros-info "DIFF: ~A" diffcds)
        (cond
         ((and (< (norm (float-vector (elt (send diffcds :worldpos) 0)
                                      (elt (send diffcds :worldpos) 1)))
                  *fridge-distance-threshold*)
               (< (abs (elt (car (rpy-angle (send diffcds :worldrot))) 0))
                  *fridge-rotation-threshold*))
          (setq cds (check-detection :type "fridge" :speak-name "れいぞうこ"))
          (ros::spin-once)
          (send *ri* :spin-once)
          (when cds
            (setq cds
                  (make-coords :pos (send cds :worldpos)
                               :rpy (list (caar (send cds :rpy-angle)) 0 0)))
            (when (setq ret (open-fridge :rarm cds 1.7 ;;(/ pi 2)
                                         :rotation-axis t :radius 490
                                         :wait-time 5.5))
              (return)))
          (speak-jp "れいぞうこを あけられません でした")
          (send *pr2* :head :angle-vector (float-vector 0 0))
          (send *pr2* :torso :angle-vector (float-vector 50))
          (send *ri* :angle-vector (send *pr2* :angle-vector))
          (send *ri* :go-pos-unsafe -0.1 0 0)
          (send *ri* :wait-interpolation)
          )
         (t ;;  there is much difference
          (send *ri* :go-pos-unsafe
                (/ (elt (send diffcds :worldpos) 0) 1100.0)
                (/ (elt (send diffcds :worldpos) 1) 1100.0)
                (* 0.9 (rad2deg (elt (car (rpy-angle (send diffcds :worldrot))) 0))))
          ;; wait ???
          (unix::usleep (* 500 1000))
          ;; (send *ri* :wait-interpolation)
          ))
        ))

    (when (not ret)
      (ros::ros-info "TIMEOUT: open-fridge-door");;
      (speak-jp "れいぞうこ を みつけられません でした")
      (unix::sleep 2);;
      (return-from open-fridge-door nil))

    ;; free door handle
    (ros::spin-once)
    (send *ri* :spin-once)
    (send *ri* :stop-grasp :rarm)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq rend (send *pr2* :rarm :end-coords :copy-worldcoords))
    (send rend :translate (float-vector -100 -50 0) *pr2*)
    (send *pr2* :rarm :inverse-kinematics rend)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

    ;; ?? open door ()
    (send *ri* :angle-vector-sequence
          (list (float-vector 50 5 74 100 -90 -70 -6 20 -70 50 -59 -43 118 0 0 -2 31) ;; 1500
                (float-vector 50 5 74 100 -90 -70 -6 20 -0 60 -20 -45 118 -90 0 -2 31) ;; 1000
                (float-vector 50 5 74 100 -90 -70 -6 20 12 45 -34 -110 12 -38 74 -2 31) ;; 1000
                (float-vector 50 5 74 100 -90 -70 -6 20 -54.745 34.2625 -34.6956 -83.6538 24.9161 -66.3615 118.904 -2.0 31.0) ;; 4500
                (float-vector 50 5 74 100 -90 -70 -6 20 -54.745 34.2625 -34.6956 -83.6538 24.9161 -66.3615 118.904 -2.0 31.0) ;; 1500 ;; just wait
                (float-vector 150 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31)) ;; 2000
          (list 1500 1000 1000 4500 1500 2000))
    (send *ri* :wait-interpolation)

    ;; (send *pr2* :angle-vector (float-vector 200 5.00529 74.2038 105.001 -90.0035 -70.0071 -5.73378 19.9988 9.96029 39.7801 -45.0972 -121.542 94.4262 -111.922 63.0666 -0.0316 21.024))
    (send *pr2* :angle-vector (float-vector 199.971 5.30455 72.1244 105.231 -88.5188 -69.9972 -5.72958 19.9717 24.6812 36.1934 10.6903 -121.497 159.113 -79.5111 -2.92658 -1.35562 26.862))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    t
    ))

(warn ";; define grasp-can")
(defun grasp-can (&key (move t) (rotation))
  ;; move-to spot for grasping
  (ros::spin-once)
  (send *ri* :spin-once)
  (when move
    (ros::ros-info "GO POS UNSAFE for grasping can")
    (send *ri* :go-pos-unsafe 0 0 (rad2deg 0.38))
    (send *ri* :go-pos-unsafe 0.185 -0.25 0)
    (send *ri* :wait-interpolation))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (let (isgrasp)
    (dotimes (trial 10)
      ;; detect cans which was indicated by ( type )
      ;;(when (not (setq cds (check-detection :type *type* :single t)))
      ;;(setq cds (check-detection :type *type* :tf-force t :timeout 30 :single t)))
      (let (cds)
        (unless (setq cds (check-detection
                           :type *type* :tf-force t :timeout 35 :single t))
          (setq cds (check-detection
                     :type *type* :tf-force t :timeout 70 :single t)))
        (when cds
          ;; (speak-jp (format nil "~A を とりだします" *type*))
          (send *obj* :reset-coords)
          (send *obj* :transform cds)
          (warn ";; print check cans~%")
          (dump-structure *standard-output*
                          (list (send *pr2* :copy-worldcoords)
                                (send cds :copy-worldcoords)))
          (unless (boundp '*tfb*)
            (setq *tfb* (instance ros::transform-broadcaster :init)))
          (send *tfb* :send-transform
                cds "/base_footprint" "/object_pose_frame")
          (ros::spin-once)
          (send *ri* :spin-once)
          (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
          (unless rotation
            ;; FIXME: calc rotation from /map and /base_footprint
            (setq rotation 0))
          (grasp-can-single cds :rotation rotation)
          (setq isgrasp (< 10 (send *ri* :start-grasp :rarm)))
          (when isgrasp
            (return-from grasp-can t))
          ;; (unix::sleep 2)
          (speak-jp "しっぱいしました  もういちど やって みます");;
          )
        )))
  (speak-jp (format nil "~A を とりだし ません でした" *type*))
  nil)

(warn ";; define close-fridge")
(defun close-fridge (&key (move t))
  ;; close the door of fridge
  (when move (send *ri* :go-pos-unsafe -0.3 0.05 -20))
  (unix::sleep 1)
  (send *ri* :angle-vector-sequence
        (list (float-vector 250 5 74 100 -90 -70 -6 20 -13 50 -30 -25 118 -90 62 -2 31);; 2000
              (float-vector 200 5 74 100 -90 -70 -6 20 -50 54 -15 -43 118 -90 62 -2 31);; 1000
              (float-vector 150 5 74 100 -90 -70 -6 20 -80 10 -39 -33 -49 -32 89 -2 32);; 1500
              (float-vector 100 5 74 100 -90 -70 -6 20 -61 6 -74 -64 55 -75 102 -2 33);; 2500
              ;;(float-vector 50 5 74 100 -90 -70 -6 20 9.05631 26.4211 -42.0907 -111.215 97.6198 -19.5577 -24.0422 -2.0 33.0)
              (float-vector 50 5 74 100 -90 -70 -6 20 -1.74817 12.0099 -66.7541 -77.0519 67.3822 -49.1028 30.3261 -2.0 33.0))  ;; 3500
        (list 2000 1000 1500 2500 3500))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)
  t
  )
