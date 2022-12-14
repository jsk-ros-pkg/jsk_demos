#! usr/bin/env roseus

(ros::ros-error "start set_umbrella.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "set_umbrella")

(setq *threshold* 0.5)  ;; 近くに人間がいるかどうかの距離の基準
(setq *counter* 0)
(setq *pre_position_number* 100)
;; (setq *umbrella_replace_done* nil)  ;; 傘を差し出し済みかどうか

(defun control_gripper ()
  (send *ri* :gripper-open)

  ;; (send *ri* :speak-jp "傘をセットしてください")
  (send *ri* :wait-interpolation)
  (unix::sleep 2)

  (send *ri* :gripper-close)
  (send *ri* :wait-interpolation)
  )
  
(defun sub_to_set (msg)
  (when *umbrella_replace_done*
    (return-from sub_to_set))
    
  (setq data (send msg :data))

  (setq position_number (elt data 0))
  (setq x (elt data 1))
  (setq y (elt data 2))
  (setq z (elt data 3))
  (setq dis (elt data 4))

  (ros::ros-info (format nil "position_number: ~A" position_number))

  (when (= position_number 100)
    (progn
      (setq *counter* 0)
      (setq *pre_position_number* position_number)
      (ros::ros-info (format nil "counter: ~A" *counter*))
      (return-from sub_to_set)))

  (cond
   ((= *counter* 0)
    (progn
      (ros::ros-info (format nil "(= *counter* 0)"))
      (ros::ros-info (format nil "counter: ~A" *counter*))
      (setq *counter* 1)
      (setq *pre_position_number* position_number)
      (ros::ros-info (format nil "counter: ~A" *counter*))
      (return-from sub_to_set)))
   ((and (> *counter* 0) (< *counter* 5))
    (progn
      (ros::ros-info (format nil "(and (> *counter* 0) (< *counter* 3))"))
      (ros::ros-info (format nil "counter: ~A" *counter*))
      (if (= *pre_position_number* position_number)
	(progn
	  (setq *counter* (+ *counter* 1)))
	(progn
          (setq *counter* 1)))
      (setq *pre_position_number* position_number)
      ;; (print *counter*)
      (ros::ros-info (format nil "counter: ~A" *counter*))
      (return-from sub_to_set)))
   ((= *counter* 5)
    (progn
      (ros::ros-info (format nil "(= *counter* 3)"))
      (ros::ros-info (format nil "counter: ~A" *counter*))
      ;; (print position_number)
      ;; (print "position")
      (when (= position_number 1)
	(progn
	  (send *ri* :go-pos (* 1.2 (- x 0.37)) -0.2 0)
	  (send *ri* :wait-interpolation)
	  (send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 90 -100 70 0 30 -90 -80))
	  (send *ri* :angle-vector (send *spot* :angle-vector) 3000)
	  (send *ri* :wait-interpolation)))

      (when (= position_number 2)
	(progn
	  (send *ri* :go-pos (* 1.2 (- x 0.37)) 0.2 0)
	  (send *ri* :wait-interpolation)
	  (send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 -90 -100 70 0 30 -90 -80))
	  (send *ri* :angle-vector (send *spot* :angle-vector) 3000)
	  (send *ri* :wait-interpolation)))

      (control_gripper)

      (setq *pre_x* x)
      (setq *pre_y* y)
      (setq *umbrella_replace_done* t)))))

(ros::subscribe "/nearest_pos" std_msgs::float32multiarray #'sub_to_set)
