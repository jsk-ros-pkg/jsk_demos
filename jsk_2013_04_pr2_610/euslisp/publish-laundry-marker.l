#!/usr/bin/env roseus

(load "models/room610-laundry-machine-object.l")

(ros::roseus "publish_laundry_marker")

(defvar *pub-laundry-topic* "/laundry_marker")
(defvar *pub-all-topic* "/all_marker_array")

(if (not (boundp '*tfb*) (setq *tfb* (instance ros::transform-broadcaster :init))))

(if (not (boundp '*laundry*) (setq *laundry* (room610-laundry-machine))))

(defun laundry-pub ()
  (ros::rate 10)
  (do-until-key
	  (let* ((header (instance std_msgs::header :init
							   :stamp (ros::time-now)
							   :frame_id "/laundry_id"))
			 (laundry-msg (laundry->laundry-msg *laundry* header))
			 (all-msg (instance visualization_msgs::MarkerArray :init))))
	(send *tfb* :send-transform (make-coords) "/base_footprint" "/laundry_id")

	;; publish msg
	(ros::publish *pub-laundry-topic* laundry-msg)
	(ros::publish *pub-all-topic* all-msg)
	
	(send laundry-msg :ns "laundry")
	(send all-msg :markers (list laundry-msg))
	(ros::publish *pub-all-topic* all-msg)

	(ros::spin-once)
	(ros::sleep)
	(ros::sleep)
	(print header)))


;; main
(ros::advertise *pub-laundry-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-all-topic* visualization_msgs::Marker 5)
(laundry-pub)