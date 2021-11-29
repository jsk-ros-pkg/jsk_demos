#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(ros::roseus "patrol")
(setq *list-upload* nil)

(defun taking-picture (place)
  (setq img (one-shot-subscribe "/dual_fisheye_to_panorama/output"
                                  sensor_msgs::Image :timeout 1000))
  (if img
    (progn
      (ros::ros-info "taking a picture of ~A" place)
      (send *ri* :speak-jp "写真を撮ります" :wait t)
      (play-sound (pathname (ros::resolve-ros-path
                             "package://jsk_pr2_startup/jsk_pr2_lifelog/camera.wav")))
      (write-image-file (format nil "/tmp/~A.jpg" place) (ros::sensor_msgs/Image->image img))
      )
    (ros::ros-error "fail saving image and tweeting"))))

(defun eng2-7FEvFront-behavior ()
  (send *ri* :speak-jp "巡回しています" :wait t)
  (send *ri* :speak-en "I'm patroling" :wait t)
  (send *ri* :go-to-spot "eng2_7FElevator")
  (send *ri* :speak-jp "7階エレベーターの巡回が終わりました" :wait t)
  (send *ri* :speak-en "I finished patroling at the front of elevator on the 7th floor" :wait t)
  ;; TODO
  ;; look at trashcan
  )

(defun eng2-3FEvFront-behavior ()
  (send *ri* :speak-jp "巡回しています" :wait t)
  (send *ri* :speak-en "I'm patroling now" :wait t)
  (send *ri* :go-to-spot "eng2_3FElevator")
  ;; TODO
  ;; look at trashcan?
  )
(defun eng2-2FEvFront-behavior ()
  (send *ri* :speak-jp "巡回しています" :wait t)
  (send *ri* :speak-en "I'm patroling now" :wait t)
  (send *ri* :go-to-spot "eng2_2FElevator")
  ;; TODO
  ;; look at trashcan
  )

(defun eng2-MechOffice-behavior ()
  (send *ri* :go-to-spot "eng2_Mech_Office")
  ;; take a photo of bulletin boards
  ;; look at post
  )

(defun eng2-73A4-behavior ()
  (send *ri* :go-to-spot "eng2_73A4")
  )

(defun eng2-81C1-behavior ()
  (send *ri* :speak-jp "ゴミ捨て場を見に行きます" :wait t)
  (send *ri* :go-to-spot "eng2_8Ftrashcan_front")
  (send *ri* :speak-jp "8 1 C1の巡回をします" :wait t)
  (send *ri* :go-to-spot "eng2_81C1")
  (send *ri* :speak-jp "巡回しています" :wait t)
  (send *ri* :speak-en "I'm patroling now" :wait t)
  (send *ri* :speak-jp "81C1の巡回を終わります" :wait t)
  (send *ri* :speak-en "I finished patroling 8 1 c 1" :wait t)
  )

(defun mechoffice-behavior ()
  (send *ri* :go-to-spot "eng2_Mech_Office")
  (send *ri* :go-to-spot "eng2_Mailbox"))

(defun mailbox-behavior ()
  (send *ri* :go-to-spot "eng2_Mailbox"))

(defun hongo-maingate-behavior ()
  (send *ri* :go-to-spot "HongoMainGate"))

(defun eng2-patrol ()
  (eng2-7FEvFront-behavior)
  ;; (eng2_3FEvFront_Behavior)
  ;; (eng2_MechOffice_Behavior)
  ;; (eng2_2FEvFront_Behavior)
  ;; (eng2-81C1-behavior)
  (send *ri* :go-to-spot "eng2_73B2")
  (unix::sleep 1)
  (send *ri* :speak-jp "2号館の巡回が終わりました" :wait t)
  (send *ri* :speak-en "I finished patroling the building number 2" :wait t)
  ;;(eng2_81C1_Behavior)
  )

(defun hongo-patrol ()
  (send *ri* :go-to-spot "eng2_MainEntrance_upper")
  (hongo-maingate-behavior)
  )
