#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(objects (list *pr2*))

(defun replay-angle-vector-on-model-safe (filename &optional (dt 0.02) (draw-p nil))
  (load (concatenate string "angle_vector/" filename ".l"))
  (setq av-list (symbol-value (intern (string-upcase filename))))
  (format t "Loaded ~A samples~%" (length av-list))
  (dolist (av av-list)
    (let ((safe-av (copy-object av)))
      (send *pr2* :angle-vector safe-av)
      (unix::usleep (truncate (* dt 1000000)))))
  (format t "Replay finished (~A samples)~%" (length av-list)))

replay-angle-vector-on-model-safe "test-motion3"