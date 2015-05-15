(defun make-msg-from-tunnel
  (tunneled-msg target output-topic-type &key (slot :data) (convert nil))
  (let ((msg (instance output-topic-type :init)) data)
    (if convert
        (setq data (funcall convert (send tunneled-msg target)))
      (setq data (send tunneled-msg target))
      )
    (send msg slot data)
    msg
    )
  )

(defun convert-mode-to-uint8
  (string-mode)
  (let (uint8-mode)
    (cond
     ((equal (string-downcase string-mode) "stop") (setq uint8-mode 0))
     ((equal (string-downcase string-mode) "operation") (setq uint8-mode 1))
     ((equal (string-downcase string-mode) "recognition") (setq uint8-mode 2))
     (t (setq uint8-mode 0))
     )
    uint8-mode
    )
  )

(defun convert-uint8-to-mode
  (uint8-mode)
  (let (string-mode)
    (case uint8-mode
      (0 (setq string-mode "stop"))
      (1 (setq string-mode "operation"))
      (2 (setq string-mode "recognition"))
      (t (setq string-mode "stop"))
     )
    string-mode
    )
  )
