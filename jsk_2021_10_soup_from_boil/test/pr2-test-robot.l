;; If you want to (assert nil) when IK fails,
;; use *pr2* of pr2-test-robot by loading this file.
;; pr2-test-robot class overrides :inverse-kinematics function
;; and assert error when IK fails

(load "package://pr2eus/pr2.l")

(defclass pr2-test-robot
  :super pr2-sensor-robot
  :slots ()
  )
(defmethod pr2-test-robot
  (:inverse-kinematics (&rest args)
    (let ((ik-result (send-super* :inverse-kinematics args)))
      (assert ik-result)
      ik-result)))

(setq *pr2* (instance pr2-test-robot :init))
