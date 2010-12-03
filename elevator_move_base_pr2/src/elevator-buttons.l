(load "package://jsk_maps/src/eng8-scene.l")

;;
(defun update-scene-by-tf (scene &optional (tm (ros::time 0)))
  (let* ((objects (flatten (send scene :inheritance)))
	 (fixed-frame (send scene :name)) ;; ok ?
	 (fixed-obj (car (send scene :find-object fixed-frame)))
	 (updated nil))
    (if (not fixed-obj) (return-from update-scene-by-tf nil))
    (dolist (obj objects)
      (when (and (stringp (send obj :name))
	     (send *tfl* :can-transform fixed-frame (send obj :name) tm))
	(let ((trans (send *tfl* :lookup-transform
			   fixed-frame (send obj :name) tm)))
	  (send obj :move-to trans fixed-obj)
	  (push obj updated)
	  ;;(ros::ros-info "trans ~A is ~A" obj trans)
      )))
    updated
    ))

