#!/usr/bin/env roseus

(load "package://drc_valve_task/euslisp/variable-valve-model.l")

(defun publish-valve-marker-one
  (&key (thick 20) (radius 115))
  (let* ((obj (instance variable-valve :init :thick thick :radius radius :ring-color (float-vector 1 0 0)))
         (header-msg (instance std_msgs::header :init
                               :stamp (ros::time-now) :frame_id "/OBJ")))
    (ros::publish "valve_model_marker" (object->marker-msg obj header-msg))
    (ros::sleep)
    (ros::spin-once)
    ))


(defun main
  (&key (debug nil))

  (ros::roseus "variable_model_publisher")
  (ros::rate 100)

  (ros::advertise "valve_model_marker" visualization_msgs::Marker 1)
  (ros::subscribe "valve_model_thick" std_msgs::Float32 #'valve-model-thick-cb)
  (ros::subscribe "valve_model_radius" std_msgs::Float32 #'valve-model-radius-cb)

  (setq *valve-thick* 10)
  (setq *valve-radius* 100)

  (let* ((count 0))
    (do-until-key
     (incf count)
     (when debug
       (setq *valve-thick* (* 20 (abs (sin (/ count 100.0)))))
       (setq *valve-radius* (* 200 (abs (cos (/ count 100.0))))))
     (publish-valve-marker-one :thick *valve-thick* :radius *valve-radius*)
     ))
  )

(defun valve-model-thick-cb
  (msg)
  (setq *valve-thick* (send msg :data)))

(defun valve-model-radius-cb
  (msg)
  (setq *valve-radius* (send msg :data)))


(main :debug nil)
