#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(objects (list *robot*))

(send *ri* :stop-grasp :rarm :width 0.006)

(setq ee-pos #f(547 -18 825))
(setq ee-rpy #f(-3.14 1.57 1.57))

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos ee-pos :rpy ee-rpy)) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(setq i 0)
(setq stroke #f(0 0 -1))

(while (< i 10)
  (setq ee-pos (v+ ee-pos stroke))
  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos ee-pos :rpy ee-rpy)) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (setq i (+ i 1)))

  
  


