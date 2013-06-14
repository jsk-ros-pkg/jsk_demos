#!/usr/bin/env roseus


(load "package://pr2eus/pr2-interface.l")
(load "models/room610-table-object.l")
(load "~/prog/euslib/jsk/pfilter.l")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_2013_04_pr2_610")
(defparameter *result-topic* "/narrow_stereo/left/ObjectDetection")
(defparameter *check-particles-topic* "/check_particles_topic")
(defvar *base-frame-id* "/base_footprint")

(ros::roseus "detect_object")

(setq *room610-origin* "/eng8/6f/610")
(setq *tobject* (instance room610-table-object :init))
(setq *ri* (instance pr2-interface :init))

;;3rd argument is not meaningful
(setq *object-foot1* (float-vector (/  1.09 2) (/  0.63 2) 0)) ;;1.09
(setq *object-foot2* (float-vector (/ -1.09 2) (/  0.63 2) 0)) ;;0.63
(setq *object-foot3* (float-vector (/  1.09 2) (/ -0.63 2) 0))
(setq *object-foot4* (float-vector (/ -1.09 2) (/ -0.63 2) 0))
(setq *error-thred* 0.05) ;;0.05
(setq *object-xy* nil)
(setq *inner-threshold* 0.2)


(setq *detect-pf* nil)
(setq *detect-system-noise* 0.05)
(setq *detect-measurement-noise* 0.1)
(setq *detect-threshold* 1)


(defclass detect-particle-filter
  :super particle-filter)
(defmethod detect-particle-filter
  (:likelihood
   (x y)
   (let ((foot1 (v+ x (rotate-vector *object-foot1* (elt x 2) :z)))
         (foot2 (v+ x (rotate-vector *object-foot2* (elt x 2) :z)))
         (foot3 (v+ x (rotate-vector *object-foot3* (elt x 2) :z)))
         (foot4 (v+ x (rotate-vector *object-foot4* (elt x 2) :z)))
         (foot-list nil)
         (error-sum 0) (error-list nil)
         )
     (setq foot-list (list foot1 foot2 foot3 foot4))
	 (dolist (foot foot-list)
       (if (> (length y) 0) (setq min-error (distance foot (elt y 0))))
       (dolist (measure-point y)
         (setq d (distance measure-point foot))
         (if (> min-error d) (setq min-error d))
         )
       (push min-error error-list)
       )
     (setq error-pass-num 0)

     (dolist (error-val error-list)
       ;;is error is over threshold?
       (if (< error-val *detect-threshold*)
           (progn
             (+ error-sum error-val)
             (incf error-pass-num)
             )
         )
       )
	 (print "passnum:")
	 (print  error-pass-num)
	 
     (if (> error-pass-num 2)
         (progn
		   (print "GOOD!::error-list")
		   (pprint error-list)
		   (print x)
           (exp (* measurement-noise error-sum))
           )
	   (progn
		 (print "--------BAD::error-list:--------------")
;;		 (pprint error-list)
		 (print "error-list -end")
		 0
		 )
	   )
	 )
   )
  )



(defun init-particles ()
  (setq *detect-pf* (instance detect-particle-filter :init :state-dim 3
                              :particle-num 200
                              :system-noise *detect-system-noise*
                              :measurement-noise *detect-measurement-noise*
                              :initial-noise 1.0
                              :use-adaptive-particle-num t))
  (send *detect-pf* :particle-num 20)
  (send *detect-pf* :initial-noise 0.1)
  (setq pr2->target (send (send *pr2* :base_laser_link) :transformation *target-object*))
  (steq pr2->target-pos (send pr2->target :worldpos))
  (setq pr2->target-rot (send pr2->target :rpy-angle))
  (send *detect-pf* :initial-state (float-vector (elt pr2->target-pos 0) (elt pr2->target-pos 1) (rad2deg (car (car pr2->target-rot)))))
  (send *detect-pf* :reset-particles)
  )

;;convert r theta to xy
(defun rtheta->xy (r theta &key (debug nil))
  (let (x y)
    (setq x (* r (cos theta)))
    (setq y (* r (sin theta)))
    (when debug
      (ros::ros-info "~d ~d x-y" x y)
      )
    (float-vector x y 0)
    )
  )

;;CALL-BACK this will be called when subscriver is activated
(defun table-detection-cb (laser-scan-msg)
  (let ((trans nil) (angle (send laser-scan-msg :angle_min)) (angle-step (send laser-scan-msg :angle_increment)) (ranges-vec (send laser-scan-msg :ranges)) (x 0) (y 0) (xy-list nil) (object-width-likely nil) (object-height-likely nil) (object-width2-likely nil) (object-height2-likely nil) ret)

    (setq ranges-vec (send laser-scan-msg :ranges))
    (ros::ros-info "------------callback is called!!----------------" )
    (setq ranges (coerce ranges-vec cons))

    ;; First Calc the x y and put those in xy-list
    ;; and add index too.
    (dolist (range ranges)
      ;;here if range > 0 it is meaningful
      (if (> range 0)
	  (push (rtheta->xy range angle) xy-list)
	)
      (setq angle (+ angle angle-step))
      )

    (setq ret (send *detect-pf* :update xy-list))
    (send *tobject* :reset-coords)
    (send *tobject* :locate (float-vector (elt ret 0) (elt ret 1) 0))
    (send *tobject* :rotate (elt ret 2) :z)

    ;;publish particles----------------------------------------------------------
    (setq publish-particles-list nil)
    (dotimes (j (send *detect-pf* :particle-num))
      (setq pf-weight (aref (send *detect-pf* :weight) j))
      (setq pf-predict (send *detect-pf* :predict))
      (setq gq (instance geometry_msgs::Quaternion :init :x (elt (matrix-row  pf-predict j) 0) :y (elt (matrix-row pf-predict j) 1) :z 0 :w pf-weight))
      (push gq publish-particles-list)
      )
    (stq all-input-msg (instance jsk_2013_04_pr2_610::geometry_quaternions :init))
    (send all-input-msg :vectors publish-particles-list)
    (when (> (length publish-particles-list) 0)
      (ros::publish *check-particle-topic* all-input-msg)
      )


    ;;publish ObjectDetection----------------------------------------------------
    (let ((res (instance posedetection_msgs::ObjectDetection :init))
          lst
          (header  (instance std_msgs::header :init
                             :stamp (send laser-scan-msg :header :stamp)
                             :frame_id *base-frame-id*)))
      (let* ((o6dp (instance posedetection_msgs::Object6DPose :init)))
        ;;*base-laser-to-laser-scan*
        (send target-coords :transform *base-laser-to-laser-scan* :world)
        (send o6dp :pose (ros::coords->tf-pose target-coords))
        (send o6dp :type "table-center")
        (push o6dp lst)

        ;; publish object_detection_marekr
        (setq marker-life 10)
        (send *tobject* :move-to (send target-coords :copy-worldcoords)
              (send *ri* :state :worldcoords *room610-origin*))
        (send *tobject* :update-descendants) ;; ???
        (send *tobject* :copy-worldcoords)   ;; ???
        (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
        (ros::publish "/object_detection_marker_array"
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list (text->marker-msg
                                       "table-center"
                                       (send (send *tobject* :copy-worldcoords)
                                             :translate #f(-100 0 100))
                                       (instance std_msgs::header :init
                                                 :stamp (send header :stamp)
                                                 :frame_id *room610-origin*)
                                       :ns (format nil "object_text_~A" "table-center") :color #f(1 0 0)
                                       :scale 100 :lifetime marker-life :id 0)
                                      (coords->marker-msg
                                       (send *tobject* :copy-worldcoords)
                                       (instance std_msgs::header :init
                                                 :stamp (send header :stamp)
                                                 :frame_id *room610-origin*)
                                       :ns (format nil "object_coords_~A" "table-center")
                                       ;; :size 200 :width 20
                                       :lifetime marker-life :id 1)
                                      (object->marker-msg
                                       *tobject*
                                       (instance std_msgs::header :init
                                                 :stamp (send header :stamp)
                                                 :frame_id *room610-origin*)
                                       :ns (format nil "object_body_~A" "table-center")
                                       :lifetime marker-life :id 2)
                                      )))
        )
      (send res :header header)
      (send res :objects lst)
      (ros::publish *result-topic* res)
      (ros::ros-info "PUBLISHED to ~A" *result-topic*)
      )

    )
  )


(defun detect-table ()
  (ros::subscribe "/scan_filtered_foot"
                  sensor_msgs::LaserScan
                  #'table-detection-cb)

  (ros::advertise *result-topic* posedetection_msgs::ObjectDetection)
  (ros::advertise *check-particles-topic* jsk_2013_04_pr2_610::geometry_quaternions)
  (init-particles)

  ;; get base_link to base_laser_link
  (setq *base-laser-to-laser-scan* (make-coords :pos #f(275 0 303)))

  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

(detect-table)