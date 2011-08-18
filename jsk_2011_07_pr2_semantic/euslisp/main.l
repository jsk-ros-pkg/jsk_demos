#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "knowrob-interface.l") ;; -> json_prolog/src
(load "actions.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  ;;  (setq *scene* (make-eng2-scene))
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init :move-base-action-name "elevator_move_base")))
  ;;(unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (unless (boundp '*tfl*)(setq *tfl* (instance ros::transform-listener :init)))
  (setq *prolog* (instance json_prolog :init "json_prolog"))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'identity)
  )

(defun insert-robot-pose ()
  (let (pose po ret
             (type "'http://ias.cs.tum.edu/kb/knowrob.owl#Robot'")
             (robot "'http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#pr2'"))
    (ros::spin-once)
    (setq pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
    (send pose :locate (scale 0.001 (send pose :worldpos)) :world)
    (setq po (coerce (array-entity (send pose :4x4)) cons))
    (setq po (mapcan #'(lambda(x)(coerce (format nil ",~a" x) cons)) po))
    (setq po (coerce (cdr po) string))
    (send *prolog* :query
	  (list->prolog `((create_object_instance ,type ,robot)))) ;; always success
    (setq ret (send *prolog* :query
		    (list->prolog `((update_pose ,robot ,(format nil "[~a]" po))))))
    ret ))


;; ask to knowrob
(defun ask-query-and-move (&optional (type "Cup") (name nil))
  (let* (solutions pose opose flag)
    (ros::spin-once)
    (setq opose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
    (setq *opose* opose)
    ;;(setq opose *opose*)
    (insert-robot-pose)
    (setq solutions (similar-objects-with-info-by-distance type))
    (speak-jp "ごようけんはなんですか") (unix::sleep 4)
    (dolist (solution solutions)
      (setq obj (cadr (assoc "OBJ" solution :test #'equal)))
      (setq robot-pose (cadr (assoc "POSE_SPOT" solution :test #'equal)))
      (ros::spin-once)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :angle-vector #f(0 0))
      (send *pr2* :torso :angle-vector #f(0))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (clear-costmap)
      (send *ri* :wait-interpolation)
      (send *ri* :move-to robot-pose) ;; robot moves
      (ros::spin-once)
      (send *pr2* :move-to (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)) :world)
      (setq obj-pose (cadr (assoc "POSE_OBJ" solution :test #'equal)))
      (send *pr2* :head :look-at (send obj-pose :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; grasp cup here
      (speak-jp "こっぷをさがしています")
      (let ((neck-av (send *pr2* :head :angle-vector)))
	(setq flag
	      (or (check :timeout 6 :type name)
		  (progn (send *pr2* :head :angle-vector (v+ #f(20 0) neck-av))
			 (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
			 (send *ri* :wait-interpolation)
			 (check :timeout 6 :type name))
		  (progn (send *pr2* :head :angle-vector (v+ #f(-20 0) neck-av))
			 (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
			 (send *ri* :wait-interpolation)
			 (check :timeout 6 :type name)))
	      ))
      (when flag
	(speak-jp "こっぷがみつかりました")
	(send *pr2* :head :look-at (send *obj* :worldpos))
	(send *ri* :head-angle-vector (send *pr2* :angle-vector) 500)
	(or (pick :larm)
	    (progn
	      (check :timeout 4 :type name)
	      (pick :larm)))
	;; go to somewhere
;	(pr2-tuckarm-pose :larm)
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send *pr2* :head :angle-vector #f(0 0))
	(send *pr2* :torso :angle-vector #f(0))
	(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	(send *ri* :wait-interpolation)
	(clear-costmap)
	(send *ri* :move-to opose)
	(send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
	(send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
	(speak-jp "こっぷをもってきました")
	(hand-over)
	(return-from ask-query-and-move t))
      (speak-jp "こっぷはみつかりませんでした")
      (pr2-tuckarm-pose :larm)
      )
    (send *ri* :move-to opose)
    nil))

(defun wait-for-hand-impact (arm &key (timeout 30))
  (let* ((action-name (format nil "/~c_gripper_sensor_controller/event_detector" (if (eq arm :larm) #\l #\r)))
         (client (instance ros::simple-action-client :init action-name pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
         (goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init)))
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
    (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_ACC*)
    (send goal :goal :command :slip_trigger_magnitude 0.02)
    (send goal :goal :command :acceleration_trigger_magnitude 3.0) ;; m/s^2
    (unix::sleep 1)
    (send client :send-goal goal)
    (ros::ros-info "wait for touching robot hand")
    (send client :wait-for-result :timeout timeout))
  )

(load "package://jsk_maps/src/eng2-scene.l")
(defun pickup-sandwitch (av0)
  (let (koshi spose gpose avs avs2 tms tms2)
    (setq koshi 300 koshi/2 150)
    (setq avs (list (float-vector koshi 14.9142 67.1806 101.326 -90.5095 -71.4186 -5.72958 20.0777 -31.6467 46.2314 -67.9652 -107.952 -10.8162 -79.0911 170.481 0.000381 6.4)
		     (float-vector koshi -19.957 61.0395 56.1412 -120.628 -125.993 -100.145 24.7469 -62.8793 57.1855 -67.9469 -117.043 33.1559 -79.0911 170.481 0.006381 6.406)
		     (float-vector koshi -24.9115 21.5613 6.97807 -118.338 -153.238 -91.6696 175.646 -62.884 57.1758 -67.9469 -117.059 33.1559 -79.0911 170.481 0.000381 6.436)
		     (float-vector koshi -19.4962 16.1424 11.085 -64.9617 -156.874 -65.0731 70.751 -62.8793 57.1758 -67.9469 -117.059 33.1559 -79.0911 170.481 -0.005619 6.436)))
    (setq avs2 (list (float-vector 0 2.31667 31.3327 68.2505 -121.542 33.0023 -114.592 53.2743 -62.9885 57.1903 -67.9561 -117.234 33.1559 -79.0936 170.483 -0.005619 27.06)
		     (float-vector 0 43.5769 27.7993 129.082 -121.54 -3.76423 -41.4207 -69.8589 -62.9933 57.1903 -67.9561 -117.234 33.1559 -79.0936 170.483 0.018381 27.06)
		     (float-vector 0 1.87966 72.7594 92.5151 -92.2514 9.2315 -5.72958 -34.318 -63.0028 57.1903 -67.9561 -117.242 33.1559 -79.0936 170.483 0.012381 27.06)))
    (setq tms (list 1000 1000 1000 1000) tms2 (list 3000 2000 2000))

    ;;
    (print av0)
    (send *ri* :move-gripper :larm 0.08 :effort 20 :wait t)
    (send *pr2* :head :look-at (v+ (send *pr2* :worldpos) #f(800 0 1000)))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :angle-vector (car avs) 4000)
    (send *ri* :wait-interpolation)
    (speak-jp "さんどいっちをください") (unix::sleep 4)
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)

    (wait-for-hand-impact :larm :timeout 30)

    (send *ri* :move-gripper :larm 0.005 :effort 15 :wait t)
    (unix:sleep 5)
    (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list (car avs2))) tms)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector-sequence (append (cdr avs2) (list av0)) tms2)
    (send *ri* :wait-interpolation)
;   (send *ri* :angle-vector av0 3000)
    (speak-jp "どうもありがとう") (unix::sleep 5)
    ))

(defun tewatashi (arm)
  (let ((av0 (send *ri* :state :potentio-vector))
	(tuckarm (or (check-tuckarm-pose) arm)))
    (send *pr2* :reset-pose)
    (send *pr2* :head :look-at (v+ #f(1000 0 900) (send *pr2* :worldpos)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;
    (send *pr2* arm :inverse-kinematics (make-coords :pos #f(600 0 700)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :stop-grasp arm)
    (send *ri* :wait-interpolation)
    (wait-for-hand-impact arm :timeout 10)
    (unix::sleep 5)
    (send *ri* :start-grasp arm)
    ;;
    (pr2-tuckarm-pose tuckarm)
    ))


(defun ask-query-and-pick (&optional (type "Sandwich"))
  (let (spose gpose av0)
    (setq av0 (copy-object (send *ri* :state :potentio-vector)))
    (setq spose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
    (setq gpose (cadr (assoc "POSE_SPOT" (car (typical-place-to-buy-omics type)) :test #'equal)))
    ;;(setq gpose (send *scene* :spot "/eng2/2f/subway-lunchset"))
    ;;
    (send *ri* :move-to gpose)
    ;;
    (pickup-sandwitch av0)
    ;;
    (send *ri* :move-to spose)
    ;;
    (tewatashi :larm)
  ))

(defun demo (&rest args)
  (when (eq args nil)
    (warn "(:home) (:pick name) (:pick-kitchen name)~%")
    (return-from demo t))
  ;;
  (when (member :home args)
    (pr2-tuckarm-pose :larm)
    (send *ri* :move-to
	  (cadr (assoc "POSE_SPOT" (car (find-knowrob-objects-with-info "OfficeRoom")) :test #'equal))))
  ;;
  (when (member :pick args)
    (pr2-tuckarm-pose :larm)
    (change-inflation-range 0.35) (initialize-costmap)
    (ask-query-and-move "Cup" (cadr (member :pick args))))
  ;;
  (when (member :sandwich args)
    (setq *scene* (make-eng2-scene))
    (pr2-tuckarm-pose :rarm)
    (speak-jp "めいあいへるぷゆう") (unix::sleep 8)
    (ask-query-and-pick "Sandwich"))
  )

;;
(progn
  (init)  ;; for pr2 setup
  (init2) ;; for cup detection
)
(warn "
;;(demo :pick \"cmu\") ;; to start demo
;;(demo :sandwich) ;; to start demo
;;
;; to debug ...
;;(check :type \"cmu\")  ;; to start recognize
;;(pick :larm) ;; to pickup using openrave
")

