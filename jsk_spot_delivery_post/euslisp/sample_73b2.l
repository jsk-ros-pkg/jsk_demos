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

(defun autowalk-data-upload ()
  (setq *path*
        (ros::get-param "~path" (format nil "~A/autowalk/73b2_inside.walk"
                                        (ros::rospack-find "spot_autowalk_data"))))
  (setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
  (setq *upload* (ros::get-param "~upload" t))
  ;; Upload graphnav files to the robot.
  (if *upload*  (send *ri* :upload-path *path*))
  ;; Localize the robot in the map
  (ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
  ;; (send *ri* :initial-localization-waypoint *init-waypoint*)
  (send *ri* :initial-localization-fiducial)
  )

(defun go-to-desk ()
  (let (name position)
    (while (not (setq name (set-name)))
      (send *ri* :speak-jp "聞き取れませんでした" :wait t))
    (setq position (look-up-desk-position name))
    (send *ri* :navigate-to (car position))
    (unless (and (= (second position) 0) (= (third position) 0) (= (fourth position) 0))
      (send *ri* :go-pos #f((second position) (third position) (fourth position))))
    ))

(defun set-name ()
  (let ((message (elt (send (one-shot-subscribe "/speech_to_text_jp" speech_recognition_msgs::SpeechRecognitionCandidates) :transcript) 0)))
  (ros::ros-info "~A" message)
  (cond
   ((substringp "北川" message) "kitagawa")
   ((substringp "新城" message) "shinjo")
   ((substringp "金沢" message) "kanazawa")
   ((substringp "若林" message) "wakabayashi")
   ((substringp "種本" message) "tanemoto")
   ((substringp "ギガ" message) "guiga")
   ((substringp "石田" message) "ishida")
   ((substringp "平岡" message) "hiraoka")
   ((substringp "おびなた" message) "obinata")
   ((substringp "市倉" message) "ichikura")
   ((substringp "塚本" message) "tsukamoto")
   ((substringp "PR" message) "pr2")
   ((substringp "3Dプリンター" message) "3dprinter")
   (t nil)
   )))


(defun main ()
  ;;(send *ri* :go-pos 1.0 1.2 0)
  (send *ri* :speak-jp "宅配便ですか" :wait t)
  (send *ri* :speak-jp "どこに届けますか" :wait t)
  (unix::sleep 1)
  (go-to-desk)
  
  (send *ri* :speak-jp "お届け物です")
  )
