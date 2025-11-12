(load "package://jsk_2025_10_semi/xlerobot.l")

;; bug avoid 0.0 to max-joint-velocity and max-joint-torque
(unless (assoc :init-org (send rotational-joint :methods))
  (rplaca (assoc :init (send rotational-joint :methods)) :init-org))
(defmethod rotational-joint
    (:init
     (&rest args)
	(let (mjv mjt)
	  (when (memq :max-joint-velocity args)
	    (setq mjv (cadr (memq :max-joint-velocity args)))
	    (if (<= mjv 0)
		(rplaca (cdr (memq :max-joint-velocity args)) 5)))
	  (when (memq :max-joint-torque args)
	    (setq mjt (cadr (memq :max-joint-torque args)))
	    (if (<= mjt 0)
		(rplaca (cdr (memq :max-joint-torque args)) 100)))
	  (send* self :init-org args)
	  )
	))

(defmethod xlerobot-robot
    (:gripper
     (limb &rest args)
	     (case limb
	       (:larm (send* self :larm-gripper args))
	       (:rarm (send* self :rarm-gripper args))
	       (:arms (send-all (send* self :larm args) (send* self :rarm args))))
	     )
  )
