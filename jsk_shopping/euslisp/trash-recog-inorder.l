#!/usr/bin/env roseus

(require "move-trashcan-to-spot.l")

(ros::load-ros-manifest "posedetection_msgs")

(setq collection-list nil)

(defun hrp2-trashcan ()
  (move-to-hrp2-trashcan-front)
  (let* ((object (one-shot-subscribe "/head_camera/rgb/ObjectDetection"
                                     posedetection_msgs::ObjectDetection
                                     :timeout 5000)))
    (unless object
      (return-from hrp2-trashcan))
    (unless (boundp '*tfl*) (setq *tfl* transform-listener :init))
    (setq *mark* (send *tfl* :lookup-transform "/base_link" "/trash_mark" (ros::time 0)))
    (ros::ros-info "msg [~A]" *mark*)
    (setq distance (- (/ (elt (send *mark* :pos) 0) 1000) 0.2))
    (send *ri* :speak-jp "~Aメートル動きます．注意してください" :wait t)
    (send *ri* :go-pos-unsafe distance 0 0)
    ;; TODO use *tfl* to reach trashcan
    ;; (send *ri* :go-pos-unsafe 0.5 0 0)
    (if (and (trashcan-occupancy-recognition) (< 90 (trashcan-occupancy-recognition))
      ;; TODO enhance grasp code
      (push "hrp2" collection-list)
      (return-from hrp2-trashcan))
    t))

(defun kitchen-trashcan ()
  (move-to-trashcan-front)
  (let* ((object (one-shot-subscribe "/head_camera/rgb/ObjectDetection"
                                     posedetection_msgs::ObjectDetection
                                     :timeout 5000)))
    (unless object
      (return-from kitchen-trashcan))
    ;; TODO use *tfl* to reach trashcan
    (if (< 90 (trashcan-occupancy-recognition))
      ;; TODO enhance grasp code
      (push "kitchen" collection-list)
      (return-from kitchen-traashcan))
    t))

(defun trashcan-occupancy-recognition ()
  (let* ((msg (one-shot-subscribe "/trashbin_occupancy_detector/trashbin/occupancy"
                                  jsk_recognition_msgs::BoundingBoxArray
                                  :timeout 5000))
         (occupancy (if msg (send (elt (send msg :boxes) 0) :value)))
         (notify-text (if occupancy (if (> occupancy 1.0)
                                      "Trashcan is full."
                                      "Trashcan is not full."))))
    (when occupancy
      (* occupancy 100))))

(defun belka-support ()
  (move-to-belka-front)
  (send *ri* :speak-jp "一緒にゴミを捨てに行きましょう" :wait t)
  )

(defun fetch-only (place-list)
  (if (stringp "kitchen" (car place-list))
      (kitchen-trash-collection)
      (hrp2-trash-collection))
  (move-to-temp-area))

(defun lift-trashbag-fetch ()
  ;; assume calling after trashbag grasp
  (send *fetch* :rarm :shoulder-y :joint-angle -91.0)
  (send *ri* :angle-vector (send *fetch* :angle-vector) :fast)
  (send *ri* :wait-interpolation)
  (send *fetch* :angle-vector #f(99.6253 -91.9143 -63.6014 -148.503 46.6524 -2.28871 25.3505 0.454471 0.492839 19.0466))
  (send *ri* :angle-vector (send *fetch* :angle-vector) :fast)
  (send *ri* :wait-interpolation))
 
(defun main ()
  (hrp2-trashcan)
  (kitchen-trashcan)
  (if (< 1 (length collection-list))
      (belka-support)
      (fetch-only)))


;; memo
;; trashbag back hold
;; #f(99.6253 -91.9143 -63.6014 -148.503 46.6524 -2.28871 25.3505 0.454471 0.492839 19.0466)
