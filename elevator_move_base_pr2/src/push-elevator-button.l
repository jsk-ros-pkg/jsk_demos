(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun push-button (target-coords)
  (let (rayvec via-coords (arm :rarm)
        via-angle-vector org-arm-anglev org-head-anglev
        (rarm-reset #f(-60 70 -70 -120 160 -30 180))  ;; 2nd element 80 -> 70
        (larm-reset #f(60 70 70 -120 -160 -30 180)))  ;; 2nd element 80 -> 70

    (unless (and (boundp '*pr2*) (boundp '*ri*))
      (ros::ros-error "*pr2* or *ri* is not bound.")
      (return-from push-button nil))

    ;; check push target simply from current tuckarm-pose
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;; if the free-arm is grasping something // TODO: smart arm change motion
    (ros::ros-info (format nil "grasp -> ~A" (send *ri* :start-grasp :arms)))
    (when (< 5 (send *ri* :start-grasp :larm))
      (pr2-tuckarm-pose :rarm))
    (when (< 5 (send *ri* :start-grasp :rarm))
      (pr2-tuckarm-pose :larm))

    (setq arm (check-tuckarm-pose))
    (setq rayvec
          (normalize-vector
           (v- (float-vector
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 0)
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 1)
                (elt (send target-coords :worldpos) 2))
               (send target-coords :worldpos))))

    ;; via-coords
    (setq via-coords (send (send target-coords :copy-worldcoords)
                           :translate (scale 70 rayvec) :world))
    ;; push 20mm
    (setq target-coords (send (send target-coords :copy-worldcoords)
                              :translate (scale -20 rayvec) :world))

    (when arm
      (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
                                              (:larm larm-reset)))
      (ros::ros-info (format nil "check if IK for free-arm ~A is solvable" arm))
      (unless (and arm
                   (send *pr2* arm :inverse-kinematics target-coords
                         :rotation-axis :x))
        (ros::ros-warn (format nil "could not solve IK: ~A" arm))
        (setq arm nil)))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq org-head-anglev (send *pr2* :head :angle-vector))

    ;; reset-pose (arms only) and select arm
    (unless arm
      (setq arm
            (if (plusp (elt (send (send *pr2* :transformation target-coords)
                                  :worldpos) 1))
                :larm
              :rarm))
      (send *pr2* :reset-pose)
      (send *pr2* :head :angle-vector org-head-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (send *ri* :wait-interpolation))

    (ros::ros-info (format nil "select ~A arm to push button ~A"
                           arm (send target-coords :worldpos)))

    (setq org-arm-anglev (send *pr2* arm :angle-vector))

    ;; start push
    (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
                                            (:larm larm-reset)))
    (unless
        (send *pr2* arm :inverse-kinematics via-coords :rotation-axis :x)
      (ros::ros-warn
       (format nil "could not solve ik to via-coords: ~A" via-coords))
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
      (return-from push-button nil))
    (setq via-angle-vector (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (unless
        (send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
      (ros::ros-warn
       (format nil "could not solve ik to target-coords: ~A" target-coords))
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
      (return-from push-button nil))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
    (send *ri* :wait-interpolation)

    (send *ri* :angle-vector via-angle-vector 400)
    (send *ri* :wait-interpolation)

    ;; return original pose
    (send *pr2* arm :angle-vector org-arm-anglev)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
    (send *ri* :wait-interpolation)
    t))
