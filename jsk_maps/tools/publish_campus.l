#!/usr/bin/env roseus

(load "package://jsk_maps/tools/publish_spot_utils.l")
(load "package://jsk_maps/src/hongo-scene.l")

(defun publish-spot ()
  (let* ((stamp (ros::time-now))
         (header (instance std_msgs::header :init :stamp stamp :frame_id "/world"))
         spots)
    (dolist (spot (send *scene* :spots))
      (when (substringp "/hongo" (send spot :name))
        (push (cons (send spot :name)
                    (send spot :worldcoords)) spots)))

    ;; publish spots
    (if *use-pictogram-p*
        (publish-pictogram-array spots header :size 5)
      (publish-pin-marker-array spots header :height 30000 :radius 5000))
    ))


(defun main ()
  (ros::roseus "publish_campus")

  ;; load params
  (setq *use-pictogram-p* (ros::get-param "~use_pictogram" nil))

  (if (or (not (boundp '*scene*)) (null *scene*))
      (setq *scene* (make-hongo-campus)))

  (ros::rate 0.3)
  (while (ros::ok)
    (ros::spin-once)
    (publish-spot)
    (ros::sleep)))

(main)
