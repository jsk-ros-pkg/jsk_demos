(defun generate-robot-interface
  ()
  (setq *robot-name* (ros::get-param "~robot_name" "hrp2jsknts"))
  (cond ((string= *robot-name* "hrp2jsknts")
         (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
         (hrp2jsknts-init)
         (setq *robot* *hrp2jsknts*)
         )
        ((string= *robot-name* "hrp2jsknt")
         (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
         (hrp2jsknt-init)
         (setq *robot* *hrp2jsknt*)
         )
        ((string= *robot-name* "staro")
         (load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
         (staro-init)
         (setq *robot* *staro*)
         ))
  )

(when (boundp 'hrp2jsknts-robot)
  (defmethod hrp2jsknts-robot
    (:reach-drill-walk-pose
     ()
     (send self :angle-vector
           #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 10.7134 -42.5314 0.849741 -134.051 50.0635 22.293 25.1515 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
    (:reach-drill-pre-pose
     ()
     (send self :angle-vector
           #f(-8.144307e-06 -0.000153 -36.5591 70.8929 -34.3338 0.000178 0.0 8.144307e-06 0.000153 -36.5591 70.8929 -34.3338 -0.000178 0.0 40.0 0.0 0.0 40.0 -56.1706 -73.4951 92.0 -127.965 106.0 37.3389 20.1853 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
    (:pierce-drill-pre-pose
     ()
     (send self :angle-vector
           #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 1.19212 -20.9065 -5.36626 -98.3812 -17.2023 6.28459 -83.1406 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
    )
  (:overhand-grip-drill-pose
   ()
   (send self :angle-vector
         #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -130.0 -25.0 -5.0 80.0 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
         ))   

  )

(when (boundp 'staro-robot)
  (defmethod staro-robot
    (:reach-drill-walk-pose
     ()
     )
    (:reach-drill-pre-pose
     ()
     )
    (:pierce-drill-pre-pose
     ()
     )
    (:overhand-grip-drill-pose
    )
  )

;; (send *ri* :reach-until-touch :rarm #f(0 0 1) :coords-system :local :thre 10)
