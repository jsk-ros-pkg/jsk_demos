(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))


(defun coords-to-float-vector (coords)
  (float-vector (elt (send coords :worldpos) 0)
                (elt (send coords :worldpos) 1)
                (elt (send coords :worldpos) 2)
                (elt (car (send coords :rpy-angle)) 0)
                (elt (car (send coords :rpy-angle)) 1)
                (elt (car (send coords :rpy-angle)) 2)
                0 0))

(defun float-vector-to-coords (fv)
  (make-coords :pos (subseq fv 0 3)
               :rpy (subseq fv 3 6)))

