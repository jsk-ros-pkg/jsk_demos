#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")
(ros::roseus-add-msgs "std_msgs")                           
(ros::roseus-add-msgs "sensor_msgs")                        
(ros::roseus-add-msgs "geometry_msgs") 
(ros::load-ros-manifest "franka_msgs")
 
(dual_panda-init)

(objects (list *robot*))

(ros::roseus "cable-insertion_heuristic_insert")

(send *ri* :set-joint-pd-gain "rarm_joint1" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint2" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint3" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint4" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint5" 500 15)
(send *ri* :set-joint-pd-gain "rarm_joint6" 300 15)
(send *ri* :set-joint-pd-gain "rarm_joint7" 100 10)

(setq ee-pos #f(547 -18 825))
(setq ee-rpy #f(-3.14 1.57 1.57))
(setq wrench_init (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos ee-pos :rpy ee-rpy)) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(setq i 0)
(while (< i 20)
  (setq base_stroke #f(0.6 0 0))
  
  (setq wrench (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
  (setq wrench_diff (v- wrench wrench_init))
  (setq d_x (elt wrench_diff 0))
  (setq d_y (elt wrench_diff 1))

  (if (> d_x 1.2)
      (setq base_stroke #f(0.2 0.0 0.3)))
  (if (< d_x -0.7)
      (setq base_stroke #f(0.2 0.0 -0.3)))

  (if (> d_y 1.2)
      (setq base_stroke #f(0.2 -0.3 0)))
  (if (< d_y -1.2)
      (setq base_stroke #f(-0.05 0.5 0)))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :rarm :move-end-pos base_stroke)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (setq i (+ i 1))
  (ros::ros-info (format nil "i = ~A" i))
  (ros::ros-info (format nil "stroke is ~A" base_stroke)))

(send *ri* :stop-grasp :rarm :width 0.003)

(send *robot* :rarm :move-end-pos #f(10 0 -3))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(-15 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(exit)
