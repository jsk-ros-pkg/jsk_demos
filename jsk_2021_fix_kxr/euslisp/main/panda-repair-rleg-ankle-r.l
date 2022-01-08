#!/usr/bin/env roseus

(defun panda-repair-rleg-ankle-r ()
  (ros::ros-info "Start repairing rleg-ankle-r-servo")

  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "rarm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "larm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "show-rleg-ankle-r-1")
  (setq res (ros::service-call "repair_reaction" req t))

  ;;move-rarm

  (setq stamp (ros::time-now))
  (when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-rleg-ankle-r-servo-b" stamp 5)
    (setq *base_to_rleg_ankle_r_servo_b* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rleg-ankle-r-servo-b" stamp)))

  ;;grasp ankle-r from back
  (rarm-grasp-rleg-ankle-r-from-back)

  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)

  (send *robot* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :larm :move-end-pos #f(-70 100 -50))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "show-rleg-ankle-r-2")
  (setq res (ros::service-call "repair_reaction" req t))

  ;;convert back to front
  (setq *base_to_rleg_ankle_r_servo* (send (send *base_to_rleg_ankle_r_servo_b* :copy-worldcoords) :transform *servo-back-to-front*))

  (larm-search-servo-tag "lpov-rleg_ankle_r_servo")

  (setq stamp (ros::time-now))
  (when (send *tfl* :wait-for-transform "dual_arm_base" "lpov-rleg_ankle_r_servo" stamp 5)
    (setq *base_to_rarm_elbow_p_servo* (send *tfl* :lookup-transform "dual_arm_base" "lpov-rleg_ankle_r_servo" stamp)))

  ;;<TODO recognize which cable is missing>
  (while t
    (larm-grasp-cable "lpov-rleg_ankle_r_servo" "left")

    (larm-insert-cable "lpov-rleg_ankle_r_servo" "left")

    (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
    (send req :action "send-servo-info")
    (setq res (ros::service-call "repair_reaction" req t))
    (setq joint_states (send res :joint_state))
    (setq timeout-servo-list (make-timeout-servo-list joint_states))
    (if (not timeout-servo-list)
	(progn
	  (ros::ros-info "Success to insert cable")
	  (return-from nil))
      (progn
	(setq target (decide-target-servo timeout-servo-list))
	(if (string-equal target "rleg-ankle-r")
	    (progn
	      (ros::ros-info "Failed to insert cable")
	      (ros::ros-info "Retry to insert cable"))
	  (progn
	      (ros::ros-info "Success to insert cable")
	      (return-from nil))
	  ))))

  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1)

  (send *robot* :rarm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation))
