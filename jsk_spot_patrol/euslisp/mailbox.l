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
  (let (success (file-name (format nil "~A/~A" video-directory image-file-name)))
    (ros::ros-info "Capture an image to ~A" file-name)
    (setq success (capture-image image-topic file-name))
    (if success
        (setq *list-upload* (append *list-upload*
                                    (list (list file-name image-file-name text))))
        nil)
    ))

(defun image-uploading (report-mail-address
                        &key
                          (gdrive-parents-path "/checking_mailbox"))
  (let (res (mail-body (format nil "Mailbox checking test ~%")))
    (init-mail)
    (dolist (upload-item *list-upload*)
      (ros::ros-info "Upload file ~A" (elt upload-item 0))
      (setq res (upload-file (elt upload-item 0) (elt upload-item 1)
                             :parents-path gdrive-parents-path))
      (if (elt res 0)
          (setq mail-body (concatenate string mail-body
                                       (format nil "~A: ~A ~%"
                                               (elt upload-item 2) (elt res 2))))
          nil))
    (if *list-upload*
        (send-mail (format nil "Mailbox Checking Report")
                   report-mail-address
                   mail-body)
        nil)
    (ros::ros-info "Sent mail to ~A" report-mail-address)
    ))

(defun speech-recognition (text &optional (duration 5))
  "args: text (string)"
  (let ((start nil) (tm nil))
    (setq start (send (ros::time-now) :sec))
    (while (or (< duration (- tm start))
               (not (string= text
                             (elt
                              (send
                               (one-shot-subscribe
                                "/speech_to_text_jp"
                                speech_recognition_msgs::SpeechRecognitionCandidates)
                               :transcript) 0))))
      (setq tm (send (ros::time-now) :now)))))

(defun main ()
  (spot-init)
  ;;(send *ri* :undock)

  (setq target-id "eng2_Mailbox")
  (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
  (setq dock-id (ros::get-param "~dock_id" 520))

  (setq ret (send *ri* :go-to-spot target-id))
  (send *ri* :go-pos 0 0 -90)

  ;; to watch the JSK Mailbox
  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 150)
                     :rpy (float-vector 0 0 0)))
  (send *ri* :body-pose (send *spot* :copy-worldcoords))
  (if ret
      (progn
        (setq current-time (unix::localtime))
        (setq image-file-name
              (format nil "mailbox-checking-~A-~A-~A-~A-~A-~A-at-~A.jpg"
                      (+ 1900 (elt current-time 5))
                      (+ 1 (elt current-time 4))
                      (elt current-time 3)
                      (elt current-time 2)
                      (elt current-time 1)
                      (elt current-time 0)
                      target-id))
        (capture-snapshot image-file-name target-id))
      nil)
  (image-uploading "tsukamoto@jsk.imi.i.u-tokyo.ac.jp")

  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 0)
                     :rpy (float-vector 0 0 0)))
  (send *ri* :body-pose (send *spot* :copy-worldcoords))

  (send *ri* :speak-jp "JSK宛の荷物を取ってください" :wait t)
  (speech-recognition "はい")
  (send *ri* :speak-jp "載せ終わったら載せましたと言ってください" :wait t)
  (speech-recogtnition "乗せました" 20)

  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id))
