#!/usr/bin/env roseus

;;
;; This script is for demonstration of GraphNav interface with euslisp.
;; By default, it is assumed that Spot is at the entrance of 73B2 and headed to the AR marker on the door.
;;

(load "package://spotkinovaeus/spotkinova-interface.l")
(spotkinova-init) ;; do not create-viewer

(defun look-at-desk ()
  (setq coord (make-cascoords :pos #f(800 0 500) :rpy (float-vector 0 (/ pi 4) 0)))
  (send *spotkinova* :head :inverse-kinematics coord :rotation-axis t)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000)
  (send *ri* :wait-interpolation nil)
  (send *spotkinova* :kinova_joint_1 :joint-angle 30)
  ;; (send *spotkinova* :head :move-end-rot 30 :z)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 3000)
  (send *ri* :wait-interpolation nil)
  (send *spotkinova* :kinova_joint_1 :joint-angle -30)
  ;; (send *spotkinova* :head :move-end-rot -30 :z)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 3000)
  (send *ri* :wait-interpolation nil)
  )

(defun put-on-desk (coord)
  (send *spotkinova* :head :inverse-kinematics coord :rotation-axis nil)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000)
  (send *ri* :wait-interpolation nil)
  )

(setq *path* (ros::get-param "~path" (format nil "~A/autowalk/73b2_inside.walk" (ros::rospack-find "spot_autowalk_data"))))
(setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
(setq *upload* (ros::get-param "~upload" t))

;; Upload graphnav files to the robot.
(if *upload*  (send *ri* :upload-path *path*))

;; Localize the robot in the map
(ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
;;(send *ri* :initial-localization-waypoint *init-waypoint*)
;; you can also use following command to initialize localization if you start from 73B2
(send *ri* :initial-localization-fiducial)
;; the difference is,  :initial-localization-waypoint can initialize with any waypoint, meaning you can start from arbitary wapoint,  e.g., -1 -> 0,  1 -> -1

;;(ros::ros-info "ready go to 81C1?")
;;(if (y-or-n-p) t (exit))

;; go to tsukamoto's desk
(send *ri* :navigate-to 9)
(send *ri* :go-pos 0 0 180)

;; (ros::ros-info "ready go back to 73B2?")
;; (if (y-or-n-p) t (exit))

;; go back to 73B2
;;(send *ri* :navigate-to 0)

(send *ri* :sit)
