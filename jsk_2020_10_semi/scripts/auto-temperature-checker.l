#!/usr/bin/env roseus

(require "package://roseus_smach/src/state-machine-ros.l")
(require "package://roseus_smach/src/state-machine-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus "auto_temperature_checker")
(ros::roseus-add-msgs "opencv_apps")
(fetch-init)

(setq *roi-msg* (instance opencv_apps::FaceArrayStamped :init :header (instance std_msgs::Header :init :stamp (ros::time-now))))
(defun face-roi-cb (msg)
  (setq *roi-msg* msg))

(setq *name-msg* (instance opencv_apps::FaceArrayStamped :init :header (instance std_msgs::Header :init :stamp (ros::time-now))))
(defun face-name-cb (msg)
  (setq *name-msg* msg))

(setq *battery-msg* (instance power_msgs::BatteryState :init :is_charging t))
(setq *battery-msg-arrived* (ros::time-now))
(defun battery-state-cb (msg)
  (setq *battery-msg-arrived* (ros::time-now))
  (setq *battery-msg* msg))

(ros::subscribe "head_camera/rgb/face_roi" opencv_apps::FaceArrayStamped #'face-roi-cb)
(ros::subscribe "head_camera/rgb/face_name" opencv_apps::FaceArrayStamped #'face-name-cb)
(ros::subscribe "battery_state" power_msgs::BatteryState #'battery-state-cb)

(setq *count* 0)
(defun go-start (userdata-alist)
  (print "go-start")
  (auto-undock)
  (send *ri* :go-pos-unsafe 0 0 180)
  (let ((pos (car (get-spot-coords "/eng2/7f/room73B2-center"))))
    (go-to-spot "/eng2/7f/room73B2-center"
                (send (send (send pos :copy-worldcoords) :transform (send (make-coords) :rotate pi/2 :z)) :transform (send pos :inverse-transformation))))
  (send *ri* :angle-vector (send *fetch* :reset-pose) 2000)
  (send *ri* :start-grasp)
  (send *ri* :wait-interpolation)
  t)


(setq *not-found-count* 0)
(setq *face-found-count* 0)
(defun search-people (userdata-alist)
  ;; track people
  ;; return t if it founds face for a few seconds
  ;; if it could not find people for 60 sec, look around.
  (ros::ros-info "search-people roi received ~A sec ago, *not-found*:~A, *face-found*:~A"
                 (send (ros::time- (ros::time-now) (send *roi-msg* :header :stamp)) :to-sec)
                 *not-found-count* *face-found-count*)
  (when (and
         (< (send (ros::time- (ros::time-now) (send *roi-msg* :header :stamp)) :to-sec) 3)
         (send *roi-msg* :faces))
    (send *ri* :state)
    (let ((x (send (elt (send *roi-msg* :faces) 0) :face :x))
          (y (send (elt (send *roi-msg* :faces) 0) :face :y))
          (size (* (send (elt (send *roi-msg* :faces) 0) :face :width)
                   (send (elt (send *roi-msg* :faces) 0) :face :height)))
          (neck-y-angle (send *ri* :robot :head :neck-y :joint-angle)))
      (ros::ros-info "found face at ~A ~A with size ~A, neck-angle is ~A" x y size neck-y-angle)
      (when (and (> size 500) ;; if we found large face
                 (< (- 320 20) x)
                 (< x (+ 320 20))
                 (< (abs neck-y-angle) 10))
        (incf *face-found-count*)
        (if (> *face-found-count* 5) ;; if we found face for 5 times, transit to next situation
            (return-from search-people t)))
      ;; rotate toward human face
      (when (> (abs neck-y-angle) 10)
        (let ((last-msg-sec (send (ros::time- (ros::time-now) *battery-msg-arrived*) :to-sec)))
          (ros::ros-info "last battery msg ~A ago and is charning ~A" last-msg-sec (send *battery-msg* :is_charging))
          (if (and (< last-msg-sec 3) (null (send *battery-msg* :is_charging)))
              (send *ri* :go-velocity 0 0 (if (> neck-y-angle 0) 0.2 -0.2) 1000 :stop t))))
      (return-from search-people nil)
      ))
  (setq *face-found-count* 0) ;; could not found face, so reset to 0
  (when (> (send (ros::time- (ros::time-now) (send *roi-msg* :header :stamp)) :to-sec) 10)
    (let (req res)
      (case (mod *not-found-count* 3)
            (0 (send *fetch* :head :angle-vector #f( 60 0)))
            (1 (send *fetch* :head :angle-vector #f(-60 0)))
            (2 (send *fetch* :head :angle-vector #f(  0 0))))
      (ros::ros-warn "send head :angle-vector ~A" (send *fetch* :head :angle-vector))
      ;; let search-people.l to stop tracking
      (setq req (instance topic_tools::MuxSelectRequest :init))
      (send req :topic "__none")
      (setq res (ros::service-call "/edgetpu_human_pose_estimator/output/mux/select" req))
      (ros::ros-info "change ~A to ~A" (send res :prev_topic) (send req :topic))
      (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 2000 :head-controller)
      ;; resume tracking
      (send req :topic "/edgetpu_human_pose_estimator/output/poses")
      (setq res (ros::service-call "/edgetpu_human_pose_estimator/output/mux/select" req))
      (ros::ros-info "change ~A to ~A" (send res :prev_topic) (send req :topic))
      ;; inc not-found-count
      (incf *not-found-count*)
    ;; renew *roi-msg* with current time
      (setq *roi-msg* (instance opencv_apps::FaceArrayStamped :init :header (instance std_msgs::Header :init :stamp (ros::time-now))))))
  ;; if we do not find a face for a while, timedout
  (if (> *not-found-count* 10)
      (return-from search-people :timeout))
  nil)


(setq *name* nil)
(defun facial-recognition (userdata-alist)
  (print "facial-recognition")
  (setq *name* "")
  (when (null (send *name-msg* :faces))
    (send *ri* :speak (format nil "Who are you?. Please register your face"))
    (return-from facial-recognition nil))
  (setq *name* (send (car (send *name-msg* :faces)) :label))
  (send *ri* :speak (format nil "You look like ~A, is that right?" *name*))
  t)

(defun temperature-recognition (userdata-alist)
  (print "facial-recognition")
  (send *fetch* :reset-pose)
  (send *fetch* :rarm :inverse-kinematics (make-cascoords :pos #f(500 200 1300) :rpy (float-vector 0 (deg2rad -45) (deg2rad 90))) :look-at-target t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak (format nil "~, I will measure your temperature" *name* :wait t))
  ;;
  (send *fetch* :rarm :inverse-kinematics (make-cascoords :pos #f(350 0 1000) :rpy (float-vector 0 (deg2rad -110) (deg2rad 90))) :look-at-target t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak (format nil "~, You look very healthy" *name* :wait t))
  (send *ri* :angle-vector (send *fetch* :reset-pose) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun go-back (userdata-alist)
  (print "go-back")
  (auto-dock) ;; auto-dock returns t/nil
  )

(defun done (userdata-alist)
  (print "done")
  t)

(setq *sm*
      (make-state-machine
       '((:go-start -> :search-people)
         (:search-people -> :facial-recognition)
         (:search-people !-> :search-people)
         (:search-people :timeout :go-back)
         (:facial-recognition -> :temperature-recognition)
         (:facial-recognition !-> :search-people)
         (:temperature-recognition -> :go-back)
         (:go-back -> :done))
       '((:go-start 'go-start)
         (:search-people 'search-people)
         (:facial-recognition 'facial-recognition)
         (:temperature-recognition 'temperature-recognition)
         (:go-back 'go-back)
         (:done 'done))
       '(:go-start)
       '(:done))
      )

(warn "
;; (ros::set-logger-level \"ros.roseus\" ros::*roserror*)
;; (exec-state-machine *sm*)
")


