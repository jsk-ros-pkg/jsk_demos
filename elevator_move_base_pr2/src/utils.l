(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")


(defun call-service (service req)
  (if (ros::wait-for-service service 0)
      (ros::service-call service req)
    nil))

(defun make-scene-by-tf (tfl)
  "Load building scene by existing TF frame"
  (let (frames scene)
    (unix::sleep 1) ;; wait for TF messages

    (setq frames (send tfl :get-frame-strings))
    (cond ((find "eng2" frames :test #'string=)
           (setq scene (make-eng2-scene)))
          ((find "eng8" frames :test #'string=)
           (setq scene (make-eng8-scene)))
          (t
           (ros::ros-fatal
            "There is no building frame_id in TF tree, so exiting...")
           (exit))
          )
    scene))

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
