(defun c* (a b)
  (send a :transform b)
  )

(defun z0 (pos)
  (float-vector (elt pos 0) (elt pos 1) 0))
(defun zset (pos z)
  (float-vector (elt pos 0) (elt pos 1) z))

(defun load-robot-model (robot-type)
  (if (eq robot-type :hrp2)
      (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
    (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
    ))
