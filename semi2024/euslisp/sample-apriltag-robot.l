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
(send *ri* :send-stretch :value 30)
(send *robot* :init-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(do-until-key
    (ros::spin-once)
  (send *irtviewer* :draw-objects :flush nil)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (dotimes (i (length (send *tag-detection* :detections)))
      (when (send *tag-detection* :detections)
        (let ((detection (car (send *tag-detection* :detections)))
              target-coords
              (bx (make-cube 40 40 40)))
          (when (send detection :id)
            (send *robot* :angle-vector (send *ri* :state :potentio-vector))
            (setq target-coords
                  (ros::tf-pose->coords (send (send (send (car (send *tag-detection* :detections)) :pose) :pose) :pose)))
            (setq target-coords (send (send *camera-color-optical-frame* :copy-worldcoords)
                                      :transform target-coords))

            (send bx :newcoords target-coords)
            (send bx :worldcoords)
            (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
            (send *robot* :inverse-kinematics-loop-for-look-at :head
                  (send target-coords :worldpos) :move-target *camera-color-optical-frame*)
            (send *ri* :angle-vector (send *robot* :angle-vector) 1000)

            ;; (send *ri* :wait-interpolation)
            )))
    )
  )
  (send *irtviewer* :viewer :viewsurface :flush)
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  )

(send *ri* :servo-off)
