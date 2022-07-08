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
  (send *robot* :rarm_joint1 :joint-angle 30)
  (send *robot* :rarm_joint3 :joint-angle -20)
  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(390.444 -87.992 1363.0) :rpy (float-vector 1.57 1.526 0))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm_joint7 :joint-angle 0)
  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(456.263 -105.465 1325.954) :rpy (float-vector 1.641 0.512 -3.029))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq stamp (ros::time-now))
  (when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-rleg_ankle_r_servo_b" stamp 5)
    (setq *base_to_rleg_ankle_r_servo_b* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rleg_ankle_r_servo_b" stamp)))

  ;;grasp ankle-r from back
  (rarm-grasp-rleg-ankle-r-from-back)

  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)

  (send *robot* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :larm :move-end-pos #f(-70 100 -70))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "show-rleg-ankle-r-2")
  (setq res (ros::service-call "repair_reaction" req t))

  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(412.83 235.711 1381.438) :rpy (float-vector 0 1.511 -1.665))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(391.546 202.961 1263.654) :rpy (float-vector -1.606 -0.012 1.544))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (exit)
  ;;convert back to front
  ;; (setq *base_to_rleg_ankle_r_servo* (send (send *base_to_rleg_ankle_r_servo_b* :copy-worldcoords) :transform *servo-back-to-front*))

  ;; (larm-search-servo-tag "lpov-rleg_ankle_r_servo")

  ;; (setq stamp (ros::time-now))
  ;; (when (send *tfl* :wait-for-transform "dual_arm_base" "lpov-rleg_ankle_r_servo" stamp 5)
  ;;   (setq *base_to_rarm_elbow_p_servo* (send *tfl* :lookup-transform "dual_arm_base" "lpov-rleg_ankle_r_servo" stamp)))

  ;; ;;<TODO recognize which cable is missing>
  ;; (while t
  ;;   (larm-grasp-cable "lpov-rleg_ankle_r_servo" "left")

  ;;   (larm-insert-cable "lpov-rleg_ankle_r_servo" "left")

  ;;   (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  ;;   (send req :action "send-servo-info")
  ;;   (setq res (ros::service-call "repair_reaction" req t))
  ;;   (setq joint_states (send res :joint_state))
  ;;   (setq timeout-servo-list (make-timeout-servo-list joint_states))
  ;;   (if (not timeout-servo-list)
  ;; 	(progn
  ;; 	  (ros::ros-info "Success to insert cable")
  ;; 	  (return-from nil))
  ;;     (progn
  ;; 	(setq target (decide-target-servo timeout-servo-list))
  ;; 	(if (string-equal target "rleg-ankle-r")
  ;; 	    (progn
  ;; 	      (ros::ros-info "Failed to insert cable")
  ;; 	      (ros::ros-info "Retry to insert cable"))
  ;; 	  (progn
  ;; 	      (ros::ros-info "Success to insert cable")
  ;; 	      (return-from nil))
  ;; 	  ))))

  ;; (send *ri* :stop-grasp :rarm)
  ;; (unix:sleep 1)

  ;; (send *robot* :rarm :move-end-pos #f(-100 0 0))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  ;; (send *ri* :wait-interpolation)
  )
