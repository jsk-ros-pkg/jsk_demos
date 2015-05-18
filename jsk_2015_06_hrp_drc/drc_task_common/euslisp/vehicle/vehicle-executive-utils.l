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

(defclass empty-service-client
  :slots (service-name waiting response)
  )
(defmethod empty-service-client
  (:init
   (name)
   (setq service-name name
         waiting nil
         reponse nil
         )
   )
  (:call
   (req)
   (if req
       (when (not waiting)
         (setq waiting t)
         (ros::service-call service-name (instance std_srvs::EmptyRequest :init))
         (setq response t)
         )
     (progn (when waiting (setq waiting nil))
            (setq response nil)
            )
     )
   )
  (:response () response)
  )

(defclass empty-service-server
  :slots (service-name request)
  )
(defmethod empty-service-server
  (:init
   (name)
   (setq service-name name
         request nil
         )
   )
  (:receive
   (response)
   (when (and request response)
     (setq request nil)
     )
   )
  (:callback
   (req)
   (let ((res (send req :response)))
     (setq request t)
     res
     )
   )
  (:request () request)
  )
