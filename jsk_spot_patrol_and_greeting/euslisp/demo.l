#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(ros::load-ros-manifest "std_srvs")

(defun greeting ()
  (let (ret res)
    ;;
    (setq ret (ros::wait-for-service "~greeting" 1))
    (if (not ret)
      (progn
        (ros::ros-error "No server found.")
        (return-from greeting nil)))
    ;;
    (setq res (ros::service-call "~greeting" (instance std_srvs::TriggerRequest :init) t))
    (send res :success)
    ))


(defun main ()
  (ros::roseus "spot-patrol-and-greeting")
  (spot-init)

  (setq patrol-list (ros::get-param "~patrol_list" nil))
  (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
  (setq dock-id (ros::get-param "~dock_id" 520))
  (setq duration-spot-stay (ros::get-param "~duration_spot_stay" 30))

  ;; undock
  (send *ri* :undock)

  (dolist (target-id patrol-list)
    (ros::ros-info "Going to ~A" target-id)
    (setq ret (send *ri* :go-to-spot target-id))
    (if ret
      (let* ((deadline (ros::time+ (ros::time-now) (ros::time duration-spot-stay))))
        (ros::ros-info "Success to reach ~A" target-id)
        (ros::ros-info "(ros::time-now) : ~A" (ros::time-now))
        (ros::ros-info "deadline : ~A" deadline)
        (ros::ros-info "(ros::time< (ros::time-now) deadline) : ~A" (ros::time< (ros::time-now) deadline))
        (while (ros::time< (ros::time-now) deadline)
            (greeting)
            (send *ri* :go-to-spot target-id)
            (unix::sleep 5)
            )
        )
      (ros::ros-warn "Failed to reach ~A" target-id))
    )

  ;; go back to home and dock
  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id)
  )

(main)
(exit)
