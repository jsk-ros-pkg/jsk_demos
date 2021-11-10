#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "nav_msgs")

(load "package://roseus_smach/src/state-machine-ros.l")


(defclass watchdog-interface
  :super spot-interface
  :slots (luminance
          ypr-euler
          luminance-threshold
          stand-start-time
          yaw-offset
          battery-temp
          temp-threshold-sit
          temp-threshold-stand
          human-rects
          rest-time-per-hour
          human-width-threshold
          human-height-threshold
          max-tracking-yaw
          paranoma-width
          paranoma-height
          lookup-angle
          change-body-duration
          bark-sound
          video-directory
          gdrive-parents-path
          list-upload-queue))
(defmethod watchdog-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init args)
     (setq luminance-threshold (ros::get-param "~luminance_threshold" 0))
     (setq human-height-threshold (ros::get-param "~human_height_threshold" 0.0))
     (setq rest-time-per-hour (ros::get-param "~rest_time_per_hour" 0))
     (setq human-width-threshold (ros::get-param "~human_width_threshold" 0.0))
     (setq max-tracking-yaw (ros::get-param "~max_tracking_yaw" 0.0))
     (setq lookup-angle  (ros::get-param "~lookup_angle" -0.4)) ;; radian
     (setq change-body-duration (ros::get-param "~change_body_duration" 1.0))
     (setq temp-threshold-sit (ros::get-param "~temp_threshold_sit" 50))
     (setq temp-threshold-stand (ros::get-param "~temp_threshold_stand" 40))
     (setq bark-sound (pathname (ros::get-param "~bark_sound" "bark.wav")))
     (let ((ros-img (one-shot-subscribe "~panorama_image" sensor_msgs::Image)))
       (setq paranoma-width (send ros-img :width))
       (setq paranoma-height (send ros-img :height)))
     (ros::subscribe "~input" sensor_msgs::Image #'send self :image-cb)
     (ros::subscribe "~human" jsk_recognition_msgs::RectArray #'send self :detect-cb)
     (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'send self :odom-cb)
     (setq stand-start-time (ros::time-now))
     (setq yaw-offset 0)
     (setq luminance 0)
     (setq battery-temp 0)
     (setq ypr-euler '(0 0 0))
     (setq video-directory (ros::get-param "~video_directory" "/tmp"))
     (setq gdrive-parents-path (ros::get-param "~gdrive_parents_path"))
     (setq list-upload-queue (list))
     ))
  (:upload-images ()
                  (let (res)
                    (dolist (upload-item list-upload-queue)
                      (ros::ros-info "Upload file ~A" (elt upload-item 0))
                      (setq res (upload-file (elt upload-item 0) (elt upload-item 1) gdrive-parents-path))
                      )
                    (setq list-upload-queue nil)
                    )
                  )
  (:capture-image (image-file-name)
                  (let ((image-topic "/spot_recognition/object_detection_image")
                        (file-name (format nil "~A/~A" video-directory image-file-name)))
                    (ros::ros-info "Capture a image to ~A" file-name)
                    (capture-image image-topic file-name)
                    (setq list-upload-queue (append list-upload-queue (list (list file-name image-file-name))))
                    ))
  (:spot-status-battery-states-callback  (msg)   ;; overwrite
                                         (send-super :spot-status-battery-states-callback msg)
                                         (setq battery-temp (apply #'max (coerce (send (elt (send msg :battery_states) 0) :temperatures) cons)))
                                         (ros::ros-debug "battery temp: ~A" battery-temp)
                                         )
  (:image-cb (msg)
             (let ((img (instance grayscale-image :init
                                  (send msg :width)
                                  (send msg :height)
                                  (copy-object (send msg :data)))))
               (dotimes (w (send img :width))
                 (dotimes (h (send img :height))
                   (setq luminance (+ luminance (send img :pixel w h)))))
               (setq luminance (/ luminance (* (send img :width) (send img :height))))
               (ros::ros-debug "receive ~A, luminance is ~A" (send (send msg :header) :frame_id) luminance)
               t)
             )
  (:detect-cb (msg)
        (setq human-rects (send msg :rects)))
  (:odom-cb (msg)
            (let ()
              (setq q (float-vector
                      (send msg :pose :pose :orientation :w)
                      (send msg :pose :pose :orientation :x)
                      (send msg :pose :pose :orientation :y)
                      (send msg :pose :pose :orientation :z)))
              (setq ypr-euler (elt (rpy-angle (quaternion2matrix q)) 0))
              (if (and
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 2)) 0.05)
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 1)) 0.05))
                  (send *ri* :set-val 'yaw-offset (elt (send *ri* :get-val 'ypr-euler) 0)))
              (ros::ros-debug  "body quaternion is ~A, YPR euler is ~A ~%" q ypr-euler)
              ))
  (:bright-surrounding () (if (> luminance luminance-threshold) t nil))
  ;;(:high-battery-temp () (if (> battery-temp temp-threshold) t nil))
  (:ready-stand-temp () (if (< battery-temp temp-threshold-stand)
                             (progn
                               t)
                             (progn
                               (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp temp-threshold-stand)
                               nil)))
  (:high-battery-temp () (if (> battery-temp temp-threshold-sit)
                             (progn
                               (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp temp-threshold-sit)
                             t)
                           nil))
  )


(defun watchdog-init (&optional (create-viewer))
  (unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
  (unless (ros::ok) (ros::roseus "spot_watchdog"))
  (unless (boundp '*ri*) (setq *ri* (instance watchdog-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *ri* :claim)
  (when create-viewer (objects (list *spot*)))
  )


(defun func-day-basic (userdata-alist)
  (let ((ret nil))
    (ros::ros-debug "day motion, luminance: ~A, stand duration: ~A min" (send *ri* :get-val 'luminance) (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60))
    (when (null (send *ri* :bright-surrounding))
      (send *ri* :sit)
      (send *ri* :power-off)
      (ros::ros-info "sit down because surrounding is dark")
      (setq ret :light-off))
    (unless (ros::ok)
      (setq ret :preempted))
    ret))

(defun func-rest (userdata-alist)
  (let ((ret nil))
    (send *ri* :upload-images)
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
      (if ret (return-from func-rest ret))
      (when (and (send *ri* :ready-stand-temp) 
                 (> (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60)
                    (send *ri* :get-val 'rest-time-per-hour)))
        (ros::ros-info  "finish rest")
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :stand))
      (unix::sleep 1) ;; (ros::sleep)
      )
    ret))

(defun func-watch (userdata-alist)
  (let ((ret nil))
    ;;(ros::rate (/ 1 (float (send *ri* :get-val 'change-body-duration))))
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
      (if ret (return-from func-watch ret))
      ;; rest
      (when (> (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60)
             (- 60  (send *ri* :get-val 'rest-time-per-hour)))
        (ros::ros-info "have a rest")
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
      ;; check battery temperature
      (when (send *ri* :high-battery-temp)
        (ros::ros-warn "the battery temperature is higher than the threshold: ~A vs ~A, sit down and turn off servo" (send *ri* :get-val 'battery-temp) (send *ri* :get-val 'temp-threshold-sit))
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
      ;; watch person
      (let
          ((max-person-height 0)
           (target-person nil)
           (human-height-threshold (send *ri* :get-val 'human-height-threshold))
           (image-file-name nil)
           (current-time (unix:localtime))
           target-rpy-euler)
        ;; workaround to handle the smaller bbox height when robot look up (head up).
        (if (< (elt (send *ri* :get-val 'ypr-euler) 1) (* (send *ri* :get-val 'lookup-angle) 0.5))
            (setq human_height_threshold (* (send *ri* :get-val 'human-height-threshold)  0.8)))
        (dolist (rect (send *ri* :get-val 'human-rects))
          (when (and (< (send rect :width) (* (send *ri* :get-val 'human-width-threshold) (send rect :height)))
                   (> (send rect :height) (* (send *ri* :get-val 'human-height-threshold) (send *ri* :get-val 'paranoma-height)))
                   (> (send rect :height) max-person-height))
            (setq max-person-height (send rect :height))
            (setq target-person rect)))
        (if (null target-person) (setq target-rpy-euler '(0 0 0))
          (progn
            (setq rel-yaw-angle (/ (* (* (- (/ (send *ri* :get-val 'paranoma-width) 2)  (+ (send target-person :x) (/ (send target-person :width) 2)))   2.0) pi) (send *ri* :get-val 'paranoma-width)))
            (if (< (abs rel-yaw-angle) (send *ri* :get-val 'max-tracking-yaw))
                (progn
                  (setq target-rpy-euler  (list 0  (send *ri* :get-val 'lookup-angle)  (- (+ rel-yaw-angle  (elt (send *ri* :get-val 'ypr-euler) 0)) (send *ri* :get-val 'yaw-offset))))
                  (ros::ros-debug "find good person to look at: target-rpy-euler: ~A, rel-yaw: ~A body yaw: ~A, yaw offset: ~A" target-rpy-euler rel-yaw-angle (elt (send *ri* :get-val 'ypr-euler) 0) (send *ri* :get-val 'yaw-offset))
                  (setq image-file-name
                        (format nil "~A-~A-~A-~A-~A-~A.jpg"
                                (+ 1900 (elt current-time 5))
                                (+ 4 (elt current-time 4))
                                (elt current-time 3)
                                (elt current-time 2)
                                (elt current-time 1)
                                (elt current-time 0)
                                ))
                  (send *ri* :capture-image image-file-name)
                  (send *ri* :play-sound (send *ri* :get-val 'bark-sound)))
              (setq target-rpy-euler '(0 0 0)))))
        (send* *ri* :body-pose target-rpy-euler))
      (unix::usleep (floor (* (send *ri* :get-val 'change-body-duration) 1000000))) ;; single thred, so consider the image luminance calculation (~0.2s)
      )
    ret))

(defun func-dark (userdata-alist)
  (let ((ret nil))
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (ros::ros-debug "dark motion, luminance: ~A, stand duration: ~A min" (send *ri* :get-val 'luminance) (/ (send (ros::time- (ros::time-now) (send *ri* :get-val 'stand-start-time)) :to-sec) 60))
      (when (send *ri* :bright-surrounding)
        (ros::ros-info "stand up because surrounding is bright")
        (send *ri* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :light-on))
      (unless (ros::ok)
        (setq ret :preempted))
      (unix::sleep 1))
    ret))


(defun watchdog-sm ()
  (let (sm sm-day)
    (setq sm-day
          (make-state-machine
           '((:watch :preempted :preempted) ;; transitions
             (:watch :sit :rest)
             (:watch :light-off :light-off)
             (:rest :preempted :preempted)
             (:rest :stand :watch)
             (:rest :light-off :light-off))
           '((:watch 'func-watch) ;; function maps
             (:rest 'func-rest))
           '(:watch)      ;; initial
           '(:light-off :preempted) ;; goal
           ))
    (setq sm
          (make-state-machine
           '((:dark-motion :light-on :day-motion) ;; transitions
             (:dark-motion :preempted :preempted)
             (:day-motion :light-off :dark-motion)
             (:day-motion :preempted :preempted))
           `((:dark-motion 'func-dark)  ;; functon maps
             (:day-motion ,sm-day))   ;; set "nestaed state machine"
           '(:dark-motion)      ;; initial
           '(:preempted) ;; goal
           ))
    sm))

;; create robot interface
(watchdog-init nil)

;; state machine
(exec-state-machine (watchdog-sm))

