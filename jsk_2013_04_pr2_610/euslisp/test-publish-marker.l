#!/usr/bin/env roseus

(ros::roseus "test_publish_marker")
(defvar *pub-all-input-topic* "/all_input_marker_array")
(defvar *base-laser-link* "/base_laser_link")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun test-pub (vector-list &key (size 100))
  (ros::rate 10)

  ;;create header
  (setq header (instance std_msgs::header :init
                              :stamp (ros::time-now)
                              :frame_id *base-laser-link*))

 ;;make input array msg
  (setq input-list-marker-array-msg nil)
  (setq index-input 0)
  (setq list-num (length vector-list))
  (dolist (flovec vector-list)
                (setq sp (make-sphere size :pos (scale 1000 flovec)))
                (setq sp-msg (sphere->marker-msg sp header :color (float-vector 0 0 (* (/ 1.0 list-num) index-input)) :alpha 0.5))
		(send sp-msg :ns (string (format nil "~dinput_sphere" index-input)))
                (push sp-msg input-list-marker-array-msg)
		(inc index-input)
        )


    (do-until-key
     (let* ((all-input-msg (instance visualization_msgs::MarkerArray :init))
	    )
	;;send all-input msg
       (send all-input-msg :markers input-list-marker-array-msg)
       (ros::publish *pub-all-input-topic* all-input-msg)
	
       (ros::spin-once)
       (ros::sleep)

       )))

(ros::advertise *pub-all-input-topic* visualization_msgs::MarkerArray 5)
