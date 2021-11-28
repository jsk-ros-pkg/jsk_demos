#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://audio_video_recorder/euslisp/audio-video-recorder-client.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(setq *list-upload* nil)

(defun capture-snapshot (image-file-name text
                       &key
                       (image-topic "/spot_recognition/object_detection_image")
                       (image-directory "/tmp"))
  (let (success (file-name (format nil "~A/~A" image-directory image-file-name)))
    (ros::ros-info "Capture a image to ~A" file-name)
    (setq success (capture-image image-topic file-name))
    (if success
        (setq *list-upload* (append *list-upload* (list (list file-name image-file-name text))))
      nil)
    ))

(defun upload-images (&key
                       (gdrive-parents-path "/patrol_and_greeting/image"))
  (let (res (mail-body ""))
    ;; upload image and prepare mail body
    (dolist (upload-item *list-upload*)
      (ros::ros-info "Upload file ~A" (elt upload-item 0))
      (setq res (upload-file (elt upload-item 0) (elt upload-item 1) :parents-path gdrive-parents-path))
      (if (elt res 0)
          (setq mail-body (concatenate string mail-body (format nil "~A : ~A ~%" (elt upload-item 2) (elt res 2))))
        nil)
      )
    mail-body
  ))

(defun report-mail (report-mail-address mail-body)
  (send-mail (format nil "Patrol And Greeting Demo Report")
             report-mail-address
             mail-body)
  )

(defun upload-video (video-path
                     video-name
                     &key
                      (gdrive-parents-path "/patrol_and_greeting/video"))
  (let (res)
    (ros::ros-info "Upload file ~A" video-path)
    (setq res (upload-file video-path video-name :parents-path gdrive-parents-path))
    (if (elt res 0)
        (format nil "~A : ~A ~%" video-name video-path)
      "")
  ))

(defun main ()
  (ros::roseus "spot-patrol-and-greeting")
  (spot-init)
  (init-mail)

  (setq patrol-list (ros::get-param "~patrol_list" nil))
  (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
  (setq receiver-address (ros::get-param "~receiver_address"))
  (setq dock-id (ros::get-param "~dock_id" 520))
  (setq video-directory (ros::get-param "~video_directory" "/tmp/"))

  (setq video-audio-topic-name (ros::get-param "~video_audio_topic_name"))
  (setq video-image-topic-name (ros::get-param "~video_image_topic_name"))
  (setq video-frame-rate (ros::get-param "~video_frame_rate"))

  ;; start a recording task
  (setq current-time (unix:localtime))
  (setq video-file-name
        (format nil "patrol-and-greeting-demo-~A-~A-~A-~A-~A-~A.avi"
                (+ 1900 (elt current-time 5))
                (+ 4 (elt current-time 4))
                (elt current-time 3)
                (elt current-time 2)
                (elt current-time 1)
                (elt current-time 0)
                ))
  (setq video-file-path (concatenate string video-directory video-file-name))
  (call-start-record-service video-audio-topic-name video-image-topic-name video-file-path video-frame-rate)

  ;; undock
  (send *ri* :undock)

  (dolist (target-id patrol-list)
    (setq ret (send *ri* :go-to-spot target-id))
    (if ret
      (progn
        (ros::ros-info "Success to reach ~A" target-id)
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
      (ros::ros-warn "Failed to reach ~A" target-id))
    )
  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id)

  ;; stop a recording task
  (call-stop-recording-service video-file-path)

  (setq whole-mail-body (format nil "Patrol results ~%"))
  (setq mail-body-images (upload-images))
  (setq mail-body-video (upload-video video-file-path video-file-name))
  (setq whole-mail-body (concatenate string
                                     whole-mail-body
                                     mail-body-images
                                     mail-body-video))

  (ros::ros-info "sending a mail")
  (report-mail receiver-address whole-mail-body)
  )

(main)
(exit)
