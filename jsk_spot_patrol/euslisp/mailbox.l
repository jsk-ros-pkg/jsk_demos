#!/usr/bin/env roseus
(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")
(load "package://jsk_spot_patrol/euslisp/speech_recognition.l")

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

(defun upload-video (video-path
                     video-name
                     &key
                       (gdrive-parents-path "/checking_mailbox/video"))
  (let (res)
    (ros::ros-info "Upload file ~A" video-path)
    (setq res (upload-file video-path video-name :parents-path gdrive-parents-path))
    (if (elt res 0)
        (format nil "~A : ~A ~%" video-name (elt res 2))
        "")
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
  (setq speech-recog-result (instance speech-recognition-client :init))

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
        (capture-snapshot image-file-name target-id)
        (capture-snapshot (format nil "kinova-~A" image-file-name) target-id
                          :image-topic "/kinova_wrist_camera/color/image_raw"))
      nil)
  (image-uploading "tsukamoto@jsk.imi.i.u-tokyo.ac.jp")

  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 0)
                     :rpy (float-vector 0 0 0)))
  (send *ri* :body-pose (send *spot* :copy-worldcoords))

  (send *ri* :speak-jp "助けてくれる人を探しています" :wait t)
  (setq hoge nil)
  (setq duration (send (instance ros::time :now) :sec))
  (while (> 5 (- (send (instance ros::time :now) :sec) duration))
    (send *ri* :speak-jp "助けてくれる人を探しています" :wait t)
    (ros::ros-info "1111")
    ;;(send *ri* :speak-jp "助けてくれる人は僕に声をかけてください" :wait t)
    ;;(send *ri* :speak-jp "僕を助けてください" :wait t)
    (setq hoge (send speech-recog-result :recognize)))
  (send *ri* :speak-jp "助けてくれる人が見つかりました" :wait t)

  (send *ri* :speak-jp "岡田先生宛の郵便物を取ってください" :wait t)

  (string= "はい" (elt (send speech-recog-result :recognize) 0))
  (send *ri* :speak-jp "載せ終わったら返事をしてください" :wait t)
  (string= "はい" (elt (send speech-recog-result :recognize) 0))
  ;; のせました
  ;; 受け取りました
  (send *ri* :speak-jp (format nil "~A 郵便物を届けに来ました" "岡田先生") :wait t)
  (send *ri* :speak-jp "受け取ってください" :wait t)
  (send *ri* :speak-jp "受け取ったら受け取りましたと言ってください" :wait t)
  (string= "受け取りました" (send speech-recog-result :recognize))
  (send *ri* :speak-jp "ありがとうございます" :wait t)
  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id))


;; memo of kinova-face-camera-front-pose
;; #f(28.3015 81.8334 -160.021 -27.723 82.1618 -160.021 32.7776 80.807 -160.021 -32.405 80.9873 -160.021 0.011719 -16.0895 75.154 90.698 6.25143 89.9037)
;;;; #f(28.3015 81.8334 -160.021 -27.723 82.1618 -160.021 32.7776 80.807 -160.021 -32.405 80.9873 -160.021 -15.9637 -17.8319 72.6996 -0.670532 -71.8119 0.715897)
;; #f(28.3015 81.8334 -160.021 -27.723 82.1618 -160.021 32.7776 80.807 -160.021 -32.405 80.9873 -160.021 4.311 -9.98627 11.0518 -90.6682 -90.2765 -90.1864)
;; #f(28.3015 81.8334 -160.021 -27.723 82.1618 -160.021 32.7776 80.807 -160.021 -32.405 80.9873 -160.021 5.44795 -60.5937 126.823 -85.8483 98.7568 -87.5314)
;; #f(6.98482 45.9484 -85.6121 -7.04426 46.2331 -85.471 7.87209 45.291 -85.8227 -8.16397 45.4545 -85.7934 -0.016876 -148.045 -147.935 -89.7255 -88.7563 -89.7032)

;; bag-pose-candidate
;; #f(7.04187 43.2021 -85.7339 -7.28943 43.9643 -85.8408 7.74104 44.4562 -84.8577 -8.3431 44.0024 -84.9219 25.48 -13.7442 -71.5442 42.6138 -41.4793 27.0543)
;; bag-release-prepare
;; #f(7.30028 43.4228 -85.4931 -7.00945 44.1781 -85.6065 8.04634 45.1718 -85.5132 -8.12638 44.705 -85.5874 20.5747 10.7034 -14.4456 76.7093 -108.093 -2.16058)
;; bag-release-done
;; #f(7.88775 43.4308 -85.7202 -6.45973 44.0962 -85.7429 8.5504 44.9248 -85.2715 -7.57978 44.332 -85.1828 20.5904 10.6967 -14.4566 76.6674 -108.116 -2.22333)
;; via
;; #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 25.4763 -13.4232 -29.9682 47.1164 -17.4094 27.032)
