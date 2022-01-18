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
    ;; TODO use *tfl* to reach trashcan
    (send *ri* :go-pos-unsafe 0.5 0 0)
    (if (< 90 (trashcan-occupancy-recognition))
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
    (* occupancy 100)))

(defun belka-support ()
  (move-to-belka-front)
  (send *ri* :speak-jp "一緒にゴミを捨てに行きましょう" :wait t)
  )

(defun fetch-only (place-list)
  (if (stringp "kitchen" (car place-list))
      (kitchen-trash-collection)
      (hrp2-trash-collection))
  (move-to-temp-area))

(defun main ()
  (hrp2-trashcan)
  (kitchen-trashcan)
  (if (< 1 (length collection-list))
      (belka-support)
      (fetch-only)))
