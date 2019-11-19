(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")


(defun update-scene-by-tf (scene &optional (tm (ros::time 0)) (timeout 0.1))
  (let* ((objects (send scene :objects))
         (fixed-frame (send scene :name)) ;; ok?
         (updated nil))
    (dolist (obj objects)
      (when (and (stringp (send obj :name))
                 (send *tfl* :wait-for-transform fixed-frame (send obj :name)
                       tm timeout))
        (let ((trans
               (send *tfl* :lookup-transform fixed-frame (send obj :name) tm)))
          (send obj :move-to trans scene)
          (push obj updated)
          ;;(ros::ros-info "trans ~A is ~A" obj trans)
          )))
    updated
    ))
