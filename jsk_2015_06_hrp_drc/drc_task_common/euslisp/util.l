(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))
