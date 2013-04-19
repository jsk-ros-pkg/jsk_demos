#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")

(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))

(objects (list *room610* *pr2*))

;; utility
(defun r2p ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
(defun p2r (&optional (time 3000))
  (send *ri* :angle-vector (send *pr2* :angle-vector) time))

(defun grasp-with-angle (&optional (arm :arms) angle &key (wait t))
  (send *ri* :move-gripper arm (* angle 0.001) :wait wait)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun pr2-solve-ik (&optional (arm :arms) coords &key (rotation-axis nil) (debug-view nil) (head t) (time 3000) (wait t))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* arm :inverse-kinematics
		coords
		:rotation-axis rotation-axis
		:debug-view debug-view)
  (if head (send *pr2* :head :look-at (send *pr2* arm :end-coords :worldpos)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) time)
  (if wait (send *ri* :wait-interpolation)))
