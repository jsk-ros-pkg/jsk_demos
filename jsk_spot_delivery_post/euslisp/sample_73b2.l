#!/usr/bin/env roseus

;;
;; This script is for demonstration of GraphNav interface with euslisp.
;; By default, it is assumed that Spot is at the entrance of 73B2 and headed to the AR marker on the door.
;;
(ros::roseus-add-msgs "speech_recognition_msgs")
(load "package://spotkinovaeus/spotkinova-interface.l")
(unless (boundp '*ri*)
  (spotkinova-init)) ;; do not create-viewer

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

(setq *path* (ros::get-param "~path" (format nil "/home/lshinjo/ros/catkin_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data/autowalk/73b2_inside.walk" (ros::rospack-find "spot_autowalk_data")))) ;; (setq *path* (ros::get-param "~path" (format nil "~A/autowalk/73b2_inside.walk" (ros::rospack-find "spot_autowalk_data))))
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



(defun go-to-desk ()
  (let (name)
  (while (not (setq name (set-name)))
    (send *ri* :speak-jp "聞き取れませんでした" :wait t))
  (if (string= name "hiraoka")
      (progn
      (send *ri* :navigate-to 4)
    (send *ri* :go-pos 0 -0.2 0)))
  (if (string= name "obinata")
      (send *ri* :navigate-to 4))
  (if (string= name "ishida")
      (progn
    (send *ri* :navigate-to 5)
    (send  *ri* :go-pos 0 0 90)))
  ;; go to tsukamoto's desk
  (if (string= name "tsukamoto")
      (progn
      (send *ri* :speak-jp "塚本のところに向かいます" :wait t)
      (send *ri* :navigate-to 6))
   )
  (if (string= name "PR2")
      (send *ri* :navigate-to 10)
    )
  (if (string= name "ichikura")
      (send *ri* :navigate-to 11))
  (if (string= name "3dprinter")
      (progn
    (send *ri* :navigate-to 12)
    (send *ri* :go-pos 0 0.2 0)))
  (if (string= name "kitagawa")
      (progn
      (send *ri* :speak-jp "北川さんのところに向かいます" :wait t)
      (send *ri* :navigate-to 13)))
  (if (string= name "shinjo")
      (send *ri* :navigate-to 16))
  (if (string= name "kanazawa")
      (send *ri* :navigate-to 16)
    (send *ri* :go-pos 0.3 0 0))
  (if (string= name "wakabayashi")
      (send *ri* :navigate-to 17)
    (send *ri* :go-pos 0 -0.2 0))
  (if (string= name "tanemoto")
      (send *ri* :navigate-to 18))
  (if (string= name "guiga")
      (progn
      (send *ri* :navigate-to 18)
    (send *ri* :go-pos 0 0 -30)
    (send *ri* :go-pos 0.3 0 0)))

  ;;(send *ri* :speak-jp "ゴミはありますか")
  ;;(send *ri* :speak-jp "あったら入れてください")
  ))
;;ros::ros-info "ready go back to 73B2?")
;; (if (y-or-n-p) t (exit))

(defun set-name ()
  (let ((message (elt (send (one-shot-subscribe "/speech_to_text_jp" speech_recognition_msgs::SpeechRecognitionCandidates) :transcript) 0)))
  (print message)
  (cond ((substringp "新城" message) "shinjo")
        ((substringp "北川" message) "kitagawa")
        ((substringp "石田" message) "ishida")
        ((substringp "塚本" message) "tsukamoto")
        (t nil)
        )))
;;(send *ri* navigate-to -1) ;; go back to dock
;; go back to 73B2
;;(send *ri* :navigate-to 0)

;;(send *ri* :sit)

(defun main ()
  ;;(send *ri* :go-pos 1.0 1.2 0)
  (send *ri* :speak-jp "宅配便ですか" :wait t)
  (send *ri* :speak-jp "どこに届けますか" :wait t)
  (unix::sleep 1)
  (go-to-desk)
  
  (send *ri* :speak-jp "お届け物です")
  )
