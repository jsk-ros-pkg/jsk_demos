#!/usr/bin/env roseus

;; utility
(defun r2p ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
(defun p2r (&optional (time 3000))
  (send *ri* :angle-vector (send *pr2* :angle-vector) time))

(defun grasp-with-angle (&optional (arm :arms) angle &key (wait t))
  (send *ri* :move-gripper arm (* angle 0.001) :wait wait)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun pr2-solve-ik (&optional (arm :arms) coords &key (rotation-axis nil) (debug-view nil) (time 3000) (wait t))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* arm :inverse-kinematics
		coords
		:rotation-axis rotation-axis
		:look-at-target t
		:debug-view debug-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) time)
  (if wait (send *ri* :wait-interpolation)))

(defmethod pr2-robot
  (:start-grasp (&key (arm :arms) (time 3000))
				(let ((current-joint-angle (send self arm :gripper :joint-angle))
					  (max-count (floor (/ time 30))))
				  (dotimes (i max-count)
					(send self arm :gripper :joint-angle (/ current-joint-angle (- max-count i))))))
  (:stop-grasp (&key (arm :arms) (time 3000))
			   (let ((current-joint-angle (send self arm :gripper :joint-angle))
					 (d-angle (- 90 current-joint-angle)
							  (max-count (floor (/ time 30))))
					 (dotimes (i max-count)
					   (send self arm :gripper :joint-angle (* (/ d-angle max-count) i))))))
  )

