;;;
;;;	roslaunch realsense2_camera rs_rgbd.launch gui:=false
;;;	roslaunch stereo_image_sandbox hand_pose_estimation_d405.launch gui:=false
;;;

(require :rcb4robots "rcb4robots.l")
(load "models/arrow-object.l")

(make-kxr-robot)
(send *ri* :com-init)
(send *ri* :hold)
(send *ri* :neutral)
(send *ri* :send-stretch 40)

(setq *camera-color-optical-frame* (arrow))
(send *camera-color-optical-frame* :newcoords
      (send (send (send (send (send *robot* :HEAD_LINK1_lk) :copy-worldcoords) :translate #f(0 10 50)) :rotate pi/2 :y :local) :rotate -pi/2 :z :local))
(send (send *robot* :HEAD_LINK1_lk) :assoc *camera-color-optical-frame*)
(objects (list *robot* *camera-color-optical-frame*))


(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "visualize_hand_pose")

(defvar *topic-name* "/camera/hand_skeleton_with_depth/output/pose")
(defvar *skeleton-array* nil)

(defun skeleton->cube (skeleton)
  (let* (cube-a
         cube-b
         bone
         cube-list)
    (dotimes (i (length (send skeleton :bones)))
      (setq bone (elt (send skeleton :bones) i))
      (setq cube-a (make-cube 10 10 10))
      (setq cube-b (make-cube 10 10 10))
      (send cube-a :newcoords (make-coords :pos (ros::tf-point->pos (send bone :start_point))))
      (send cube-b :newcoords (make-coords :pos (ros::tf-point->pos (send bone :end_point))))
      (setq cube-list (append cube-list (list cube-a cube-b))))
    cube-list))


(defun human-skeleton-array-cb (msg)
  (setq *skeleton-array* (send msg :skeletons))
  (when *skeleton-array*
    (send *irtviewer* :draw-objects :flush nil)
    (send *robot* :angle-vector (send *ri* :read-angle-vector))
    (send *irtviewer* :draw-objects)
    (dolist (cube-list (mapcar #'skeleton->cube *skeleton-array*))
      (dolist (cube cube-list)
        (print (send cube :worldpos))
        (send cube :newcoords
              (send (send *camera-color-optical-frame* :copy-worldcoords) :transform (send cube :copy-worldcoords)))
        (print (send cube :worldpos))
        (send cube :draw-on :flush nil :color #f(1 0 0)))))
  (send *irtviewer* :viewer :viewsurface :flush))

(ros::subscribe *topic-name* jsk_recognition_msgs::HumanSkeletonArray #'human-skeleton-array-cb 1)

(unless (boundp '*irtviewer*) (make-irtviewer))

(do-until-key
    (x::window-main-one)
  (ros::spin-once)
  (ros::sleep)
  )
