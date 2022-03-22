;; If you want to speed up *ri* demo, use *ri* of pr2-test-interface by loading this file.
;; pr2-test-interface class overrides :angle-vector function and send fastest motion

(load "package://pr2eus/pr2-interface.l")

(defclass pr2-test-interface
  :super pr2-interface
  :slots ()
  )
(defmethod pr2-test-interface
  (:angle-vector (av &optional (tm 3000) &rest args)
   (send-super* :angle-vector av :fast args))
  (:angle-vector-sequence (avs &optional (tms (list 3000)) &rest args)
   (send-super* :angle-vector-sequence avs :fast args))
  (:wait-interpolation (&rest args)
    nil)
  (:move-gripper
   (arm pos &key (effort 25) (wait t) (ignore-stall))
   (send-super :move-gripper arm pos :wait nil))
  (:move-to (coords &rest args &key (no-wait nil) &allow-other-keys)
    (send-super* :move-to coords args :no-wait t))
  (:go-pos-unsafe (&rest args)
    (send* self :go-pos-unsafe-no-wait args))
  (:speak (text &key (lang "") (topic-name "robotsound") wait)
    nil)
  )

(setq *ri* (instance pr2-test-interface :init :controller-timeout 0))
