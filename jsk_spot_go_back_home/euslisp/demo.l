#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")


(defun send-resque-mail (mail-address media-directory)
  (let (file-name
        file-path
        current-time
        success
        (mail-title (format nil "Resque request from Spot"))
        (mail-body (format nil "Resque request mail ~%")))
    ;; capture a image
    (setq current-time (unix:localtime))
    (setq file-name
        (format nil "resque-request-image-~A-~A-~A-~A-~A-~A.jpg"
                (+ 1900 (elt current-time 5))
                (+ 4 (elt current-time 4))
                (elt current-time 3)
                (elt current-time 2)
                (elt current-time 1)
                (elt current-time 0)
                ))
    (setq file-path (format nil "~A/~A" media-directory file-name))
    (setq success (capture-image "/dual_fisheye_to_panorama/output" file-path))

    ;; upload a image
    (setq res (upload-file file-path file-name :parents-path "/go_back_home/images"))
    (setq mail-body (concatenate string mail-body (format nil "~A: ~A ~%" file-name (elt res 2))))

    ;; send a mail
    (send-mail mail-title mail-address mail-body)
    ))


(defun main ()
  (let (success)
    (ros::roseus "spot-go-back-home")
    (spot-init)
    (init-mail)

    (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
    (setq dock-id (floor (ros::get-param "~dock_id" 520)))
    (setq receiver-address (ros::get-param "~receiver_address"))
    (setq media-directory (ros::get-param "~media_directory" "/tmp"))

    ;;; First go back to home target id
    (setq success (send *ri* :go-to-spot home-id))

    ;; Abort if failed to read home
    (if (not success)
      (progn
        (ros::ros-error "Failed to reach home position.")
        (send-resque-mail receiver-address media-directory)
        (return-from main nil)))

    ;; Then try to dock
    (setq success (send *ri* :dock dock-id))

    ;; Abort if failed to dock
    (if (not success)
      (progn
        (ros::ros-error "Failed to dock.")
        (send-resque-mail receiver-address media-directory)
        (return-from main nil)))
    ))

(main)
(exit 0)
