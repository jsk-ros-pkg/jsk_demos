;;;
;;; Manager Class
;;;

(defclass ros::ros-callback-manager
  :super propertied-object
  :slots (subscribed-topics))
(defmethod ros::ros-callback-manager
  (:init () )
  (:add-subscribe
   (ros-object topic-name topic-type callback)
   (unless (member topic-name subscribed-topics :key #'car :test #'equal)
     (ros::ros-info "subscribe ~A ~A" topic-name topic-type)
     (ros::subscribe topic-name topic-type #'send self :callback topic-name))
   (push (list topic-name ros-object callback) subscribed-topics))
  (:delete-subscribe (obj topic-name)
   (setq subscribed-topics 
	 (delete-if #'(lambda(x)(and (eq (cadr x) obj) (string= (car x) topic-name)))
	      subscribed-topics))
   (unless (member topic-name subscribed-topics :key #'car :test #'equal)
     (ros::ros-info "unsubscribe ~A" topic-name)
     (ros::unsubscribe topic-name)))
  (:callback
   (topic-name msg)
   (dolist (subscribed-topic subscribed-topics)
     (when (string= topic-name (car subscribed-topic))
       (let ((rosobj (cadr subscribed-topic))
	     (callback (caddr subscribed-topic)))
	 (send rosobj callback msg)))))
  )

;; manager instance
(defvar ros::*ros-callback-manager*
  (instance ros::ros-callback-manager :init))

;;
;; callback super class
;;
(defclass ros::ros-callback-object
  :super propertied-object
  :slots ())
(defmethod ros::ros-callback-object
  (:init
   (type name &optional (cb :callback))
   (send ros::*ros-callback-manager* :add-subscribe self type name cb)
   self)
  (:delete (name)
   (send ros::*ros-callback-manager* :delete-subscribe self name))
  )

;; sample callback class
(defclass ros::ros-callback-message
  :super ros::ros-callback-object
  :slots (msg))
(defmethod ros::ros-callback-message
  (:init (type name)
	 (send-super :init name type :callback))
  (:callback (msg_) (setq msg msg_))
  (:msg (&rest args)
	(if (and msg (keywordp (car args))) (send* msg args) msg))
  )
