(cond
 ((< *counter2* 3)
  (progn
    (ros::ros-info (format nil "counter2: ~A" *counter2*))
    (setq *counter2* (+ *counter2* 1))
    (setq *sum_x* (+ *sum_x* x))
    (setq *sum_y* (+ *sum_y* y))))

 ((= *counter2* 3)
  (progn
    (ros::ros-info (format nil "counter2: ~A" *counter2*))
    (setq *ref_x* (/ *sum_x* 3))
    (setq *ref_y* (/ *sum_y* 3))
    (ros::ros-info (format nil "ref_x: ~A, ref_y: ~A" *ref_x* *ref_y*))
    (send *ri* :speak "go walking")
    (send *ri* :wait-interpolation)
    (setq *counter2* 10)))

 ((> dis 3)
  (progn
    (send *ri* :go-velocity 0 0 0 500)
    (send *ri* :speak "lost a person")
    (send *ri* :wait-interpolation)
    (unix::sleep 3)))
 
 (t
  (when (> (+ (abs (- x *ref_x*)) (abs (- y *ref_y*))) 0.1)
    (progn
      (ros::ros-info "go-vel")

      ;; go-pos
      ;; (send *ri* :go-pos (* 1.3 (- x *ref_x*)) (* 1.3 (- y *ref_y*)) 0)

      ;; go-velocity
      (send *ri* :go-velocity 0 (* 0.7 (- y *ref_y*)) 0 500 :stop nil)
      (send *ri* :wait-interpolation)
      (send *ri* :go-velocity (* 1.4 (- x *ref_x*)) 0 0 500 :stop nil)
      (send *ri* :wait-interpolation)))))
