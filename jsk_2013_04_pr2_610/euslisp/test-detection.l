(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_perception/euslisp/detection_interface.l")

(defparameter *detection-topic* "/openni/rgb/ObjectDetection")

(defun init ()
  (pr2-init))

(setq obj (make-cube 50 50 50))

(setq idealcds (make-coords))

(ros::rate 1)
(while (ros::ok)
  (setq cds (check-detection :type "laundry" :speak-name "せんたくき"))
  (when cds
	(send obj :reset-coords)
	(send obj :transform cds)
	(setq cds
	      (make-coords :pos
			   (float-vector (elt (send cds :worldpos) 0)
					 (elt (send cds :worldpos) 1) 0)
			   :rpy (list (elt (car (rpy-angle (send cds :worldrot))) 0)
				      0 0)))

	(setq diffcds (send (send idealcds :inverse-transformation)
                            :transform (send cds :worldcoords) :world))
	(warn "~A -> ~A / ~A~%" diffcds cds idealcds)
	)
  (ros::spin-once)
)
