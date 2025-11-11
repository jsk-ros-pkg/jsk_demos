#!/usr/bin/env roseus

(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://semi2024/euslisp/semi-kxr.l")
(load "models/arrow-object.l")

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "apriltag_ros")
(ros::roseus "visualize_hand_pose")

(kxr-init)

(setq *camera-color-optical-frame* (arrow))
(send *camera-color-optical-frame* :newcoords
      (send (send (send (send (send *robot* :HEAD_LINK1_lk) :copy-worldcoords) :translate #f(0 10 50)) :rotate pi/2 :y :local) :rotate -pi/2 :z :local))
(send (send *robot* :HEAD_LINK1_lk) :assoc *camera-color-optical-frame*)
(objects (list *robot* *camera-color-optical-frame*))


(setq *tag-detection* nil)

(defun tag-detection-callback (msg)
  (setq *tag-detection* msg)
  )


(ros::subscribe "/camera/color/tag_detections"
                apriltag_ros::AprilTagDetectionArray
                #'tag-detection-callback)
(ros::rate 1)


(send *ri* :servo-on)
(send *robot* :init-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(do-until-key
    (ros::spin-once)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (dotimes (i (length (send *tag-detection* :detections)))
      (when (send *tag-detection* :detections)
        (let ((detection (car (send *tag-detection* :detections))))


          ;; (when (send detection :id)
          ;;   (setq tag-id (elt (send detection :id) 0))
          ;;   (print tag-id)
          ;;   (when (= tag-id 49) ;; ここのtag-idは自分のマーカーに合わせて変える
          ;;     (send *robot* :init-pose)
          ;;     (send *robot* :RARM_JOINT0 :joint-angle 90)
          ;;     (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          ;;     (send *ri* :wait-interpolation)
          ;;     )
          ;;   (when (= tag-id 43) ;; ここのtag-idは自分のマーカーに合わせて変える
          ;;     (send *robot* :init-pose)
          ;;     (send *robot* :LARM_JOINT0 :joint-angle -90)
          ;;     (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          ;;     (send *ri* :wait-interpolation)
          ;;     )
          ;;   )
          ))
    )
  ))
