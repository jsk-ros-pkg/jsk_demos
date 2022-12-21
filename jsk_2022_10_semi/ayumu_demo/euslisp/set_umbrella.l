(defun control_gripper ()
  (send *ri* :gripper-open)

  ;; (send *ri* :speak-jp "傘をセットしてください")
  (send *ri* :wait-interpolation)
  (unix::sleep 2)

  (send *ri* :gripper-close)
  (send *ri* :wait-interpolation))





(ros::ros-info (format nil "position_number: ~A" position_number))

;; 近くに人がいないとき
(if (= position_number 100)
    (progn
      (setq *counter* 0)
      (setq *pre_position_number* position_number)
      (ros::ros-info (format nil "counter: ~A" *counter*)))

  (cond
   ((= *counter* 0)
    (progn
      (ros::ros-info (format nil "(= *counter* 0)"))
      ;; (ros::ros-info (format nil "counter: ~A" *counter*))
      (setq *counter* 1)
      (setq *pre_position_number* position_number)
      (ros::ros-info (format nil "counter: ~A" *counter*))
      ;;(return-from sub)
      ))
   ((and (> *counter* 0) (< *counter* 5))
    (progn
      (ros::ros-info (format nil "(and (> *counter* 0) (< *counter* 5))"))
      ;; (ros::ros-info (format nil "counter: ~A" *counter*))
      (if (= *pre_position_number* position_number)
	  (progn
	    (setq *counter* (+ *counter* 1)))
	(progn
	  (setq *counter* 1)))
      (setq *pre_position_number* position_number)
      (ros::ros-info (format nil "counter: ~A" *counter*))
      ;;(return-from sub)
      ))
   ((= *counter* 5)
    (progn
      (ros::ros-info (format nil "(= *counter* 3)"))
      ;; (ros::ros-info (format nil "counter: ~A" *counter*))
      
      ;; spotの左側に人がいるとき
      (when (= position_number 1)
	(progn
	  (send *ri* :go-pos (* 1.3 (- x 0.37)) -0.3 0)
	  ;; (send *ri* :wait-interpolation)
	  (send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 90 -100 70 0 30 -90 -80))
	  (send *ri* :angle-vector (send *spot* :angle-vector) 3000)
	  (send *ri* :wait-interpolation)))
      
      ;; spotの右側に人がいるとき
      (when (= position_number 2)
	(progn
	  (send *ri* :go-pos (* 1.3 (- x 0.37)) 0.3 0)
	  ;; (send *ri* :wait-interpolation)
	  (send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 -90 -100 70 0 30 -90 -80))
	  (send *ri* :angle-vector (send *spot* :angle-vector) 3000)
	  (send *ri* :wait-interpolation)))
      
      (control_gripper)
      
      ;; 傘の差し出し完了
      (setq *umbrella_replace_done* t)))))
