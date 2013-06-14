#!/usr/bin/env roseus

(defvar *pub-all-input-topic* "/detect_particle_marker")
(defvar *base-laser-link* "/base_laser_link")
(defvar *prticle-subscrive-topic* "/check_particle_topic")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_2013_04_pr2_610")


(defun check-pub-cb (geo-vec-list)

  ;;create header
  (setq header (instance std_msgs::header :init
                         :stamp (ros::time-now)
                         :frame_id *base-laser-link*))

  ;;make input array msg
  (setq input-list-marker-array-msg nil)
  (setq index-input 0)

  (setq geo-vec-vec (send geo-vec-list :vectors))
  (ros::ros-info "------------callback is called!!----------------" )
  (setq vector-list (coerce geo-vec-vec cons))
  (setq list-num (length vector-list))

  (dolist (geovec vector-list)
    (setq size (* (send geovec :w) 100))
    (setq flovec (float-vector (send geovec :x) (send geovec :y) (send geovec :z)))
    (ros::ros-info "flovec ~A" flovec)
    (setq sp (make-sphere size :pos (scale 1000 flovec)))
    (setq sp-msg (sphere->marker-msg sp header :color (float-vector 0 0 1) :alpha 0.5))
    (send sp-msg :ns (string (format nil "~dinput_sphere" index-input)))
    (push sp-msg input-list-marker-array-msg)
    (inc index-input)
    )


  (setq all-input-msg (instance visualization_msgs::MarkerArray :init))
  ;;send all-input msg
  (send all-input-msg :markers input-list-marker-array-msg)
  (ros::publish *pub-all-input-topic* all-input-msg)
  )

(defun check-particle-pub ()
  (ros::roseus "check_particle_marker")
  (ros::subscribe *prticle-subscrive-topic*
                  jsk_2013_04_pr2_610::geometry_quaternions
                  #'check-pub-cb)

  (ros::advertise *pub-all-input-topic* visualization_msgs::MarkerArray 5)

  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(check-particle-pub)
