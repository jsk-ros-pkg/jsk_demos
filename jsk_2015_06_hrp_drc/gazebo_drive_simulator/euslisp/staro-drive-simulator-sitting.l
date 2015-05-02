(load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")

(defun init ()
  (staro-init)
  )

(defun staro-normal-sitting-pose (&key (real nil))
  (send *staro* :reset-manip-pose)
  (send *staro* :lleg :angle-vector #f(0.0 0.0 -70.0 64.0 6.5 0.0))
  (send *staro* :rleg :angle-vector #f(0.0 0.0 -70.0 64.0 6.5 0.0))
  (when real
    (send *ri* :angle-vector (send *staro* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    )
  )

(defun staro-lift-leg-pose (&key (real nil))
  (staro-normal-sitting-pose :real real)
  (send *staro* :legs :move-end-pos #f(0 0 250))
  (when real
    (send *ri* :angle-vector (send *staro* :angle-vector))
    )
  )
