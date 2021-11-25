#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(setq *list-upload* nil)

(defun capture-snapshot (image-file-name text
                       &key
                       (image-topic "/spot_recognition/object_detection_image")
                       (video-directory "/tmp"))
  (let ((file-name (format nil "~A/~A" video-directory image-file-name)))
    (ros::ros-info "Capture a image to ~A" file-name)
    (capture-image image-topic file-name)
    (setq *list-upload* (append *list-upload* (list (list file-name image-file-name text))))
    ))

(defun upload-images (report-mail-address
                      &key
                       (gdrive-parents-path "/patrol_and_greeting"))
  (let (res (mail-body (format nil "Patrol results ~%")))
    ;; upload image and prepare mail body
    (dolist (upload-item *list-upload*)
      (ros::ros-info "Upload file ~A" (elt upload-item 0))
      (setq res (upload-file (elt upload-item 0) (elt upload-item 1) :parents-path gdrive-parents-path))
      (if (elt res 0)
          (setq mail-body (concatenate string mail-body (format nil "~A : ~A ~%" (elt upload-item 2) (elt res 2))))
        nil)
      )
    ;;
    (if *list-upload*
        (send-mail (format nil "Patrol And Greeting Demo Report")
                   report-mail-address
                   mail-body)
        nil)
    (ros::ros-info "Sent mail to ~A" report-mail-address)
  ))

(defun main ()
  (ros::roseus "spot-patrol-and-greeting")
  (spot-init)
  (send *ri* :undock)

  (setq patrol-list (ros::get-param "~patrol_list" nil))
  (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
  (setq dock-id (ros::get-param "~dock_id" 520))

  (dolist (target-id patrol-list)
    (setq ret (send *ri* :go-to-spot target-id))
    (if ret
      (progn
        (setq current-time (unix:localtime))
        (setq image-file-name
              (format nil "patrol-and-greeting-demo-~A-~A-~A-~A-~A-~A-at-~A.jpg"
                      (+ 1900 (elt current-time 5))
                      (+ 4 (elt current-time 4))
                      (elt current-time 3)
                      (elt current-time 2)
                      (elt current-time 1)
                      (elt current-time 0)
                      target-id
                      ))
        (capture-snapshot image-file-name target-id)
        )
      nil)
    )
  (upload-images "sktometometo@gmail.com")

  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id)
  )

(main)
