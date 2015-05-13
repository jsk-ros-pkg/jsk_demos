(defun make-msg-from-tunnel
  (tunneled-msg target output-topic-type &optional (slot :data))
  (let ((msg (instance output-topic-type :init)))
    (send msg slot (send tunneled-msg target))
    msg
    )
  )
