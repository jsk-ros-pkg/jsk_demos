(ros::roseus-add-srvs "drc_task_common")

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
     res ;; return response immediately
     )
   )
  (:request () request)
  )

(defclass set-value-service-client
  :slots (service-name waiting response response_value)
  )
(defmethod set-value-service-client
  (:init
   (name)
   (setq service-name name
         waiting nil
         reponse nil
         response_value 0.0
         )
   )
  (:call
   (req req_value)
   (if req
       (when (not waiting)
         (let ((set-value-req (instance drc_task_common::SetValueRequest :init))
               set-value-res)
           (setq waiting t)
           (send set-value-req :value req_value)
           (setq set-value-res (ros::service-call service-name set-value-req))
           (setq response_value (send set-value-res :set_value))
           (setq response t)
           )
         )
     (progn (when waiting (setq waiting nil))
            (setq response nil)
            )
     )
   )
  (:response () response)
  (:response_value () response_value) ;; currently not used (server response ideal value immediately)
  )
(defclass set-value-service-server
  :slots (service-name request request_value)
  )
(defmethod set-value-service-server
  (:init
   (name)
   (setq service-name name
         request nil
         request_value 0.0
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
     (setq request_value (send req :value))
     (send res :set_value (send req :value))
     res ;; return response immediately with ideal value
     )
   )
  (:request () request)
  (:request_value () request_value)
  )

(defclass set-controller-mode-service-client
  :slots (service-name waiting response response_value)
  )
(defmethod set-controller-mode-service-client
  (:init
   (name)
   (setq service-name name
         waiting nil
         reponse nil
         )
   )
  (:call
   (req req_value)
   (if req
       (when (not waiting)
         (let ((set-mode-req (instance drc_task_common::StringRequestRequest :init)))
           (setq waiting t)
           (send set-mode-req :data (convert-uint8-to-mode req_value))
           (ros::service-call service-name set-mode-req)
           (setq response t)
           )
         )
     (progn (when waiting (setq waiting nil))
            (setq response nil)
            )
     )
   )
  (:response () response)
  (:response_value () response_value) ;; currently not used (server response ideal value immediately)
  )
(defclass set-controller-mode-service-server
  :slots (service-name request request_value)
  )
(defmethod set-controller-mode-service-server
  (:init
   (name)
   (setq service-name name
         request nil
         request_value 0
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
     (setq request_value (convert-mode-to-uint8 (send req :data)))
     res ;; return response immediately with ideal value
     )
   )
  (:request () request)
  (:request_value () request_value)
  )
