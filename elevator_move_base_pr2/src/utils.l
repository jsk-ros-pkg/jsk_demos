(defun call-service (service req)
  (if (ros::wait-for-service service 0)
      (ros::service-call service req)
    nil))


(defun update-scene-by-tf (scene tfl &optional (tm (ros::time 0)) (timeout 0.1))
  (let* ((objects (send scene :objects))
         (fixed-frame (send scene :name)) ;; ok?
         (updated nil)
         trans)
    (dolist (obj objects)
      (when (and (stringp (send obj :name))
                 (send tfl :wait-for-transform fixed-frame (send obj :name)
                       tm timeout))
        (setq trans
               (send tfl :lookup-transform fixed-frame (send obj :name) tm))
        (send obj :move-to trans scene)
        (push obj updated)
        ))
    updated))