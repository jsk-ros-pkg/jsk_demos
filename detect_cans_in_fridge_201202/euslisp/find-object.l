(ros::load-ros-manifest "jsk_recognition_msgs")
(defvar *approach-target-bounding-box-topic* "/fcn/cluster_indices_decomposer_target/boxes")
(defvar *approach-target-bounding-box-param* "/fcn/target_names")
(defvar *base-frame-id* "/base_footprint")

(defparameter *bounding-box-msg* nil)
(defun bounding-box-topic-callback (msg)
  (setq *bounding-box-msg* msg))

(defun subscribe-bounding-box-topic ()
  (ros::ros-info "subscribe ~A" *approach-target-bounding-box-topic*)
  (ros::subscribe *approach-target-bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-topic-callback))

(defun unsubscribe-bounding-box-topic ()
  (ros::ros-info "unsubscribe ~A" *approach-target-bounding-box-topic*)
  (ros::unsubscribe *approach-target-bounding-box-topic*))

(defun bounding-box-msg->cube (msg)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
(labels ((ros-vector->list (vmsg)
          (list 
           (* 1000.0 (send vmsg :x))
           (* 1000.0 (send vmsg :y))
           (* 1000.0 (send vmsg :z)))))
  (let ((cube (apply #'make-cube (ros-vector->list (send msg :dimensions))))
        (robot->cube (send *tfl* :lookup-transform
                           *base-frame-id* (send msg :header :frame_id) (ros::time))))
    (send cube :transform robot->cube)
    (send cube :transform (ros::tf-pose->coords (send msg :pose)))
    cube)))

(defun find-fcn-object (name &key (max-count 30) (box-volume-min 1000000.0))
  (let ((target-names (ros::get-param *approach-target-bounding-box-param*))
        (cnt 0)
        label-id target-box target-angle target-dist ret)
    (dotimes (i (length target-names))
      (when (substringp name (elt target-names i))
        (setq label-id i)
        (ros::ros-info "target object: ~A (label: ~A)" (elt target-names i) i)
        (return)))
    (unless label-id
      (ros::ros-error "target name ~A is not registered (~A)" name target-names)
      (return-from find-fcn-object nil))
    (ros::set-dynamic-reconfigure-param "/fcn/label_to_mask" "label_value" :int label-id)
    (unwind-protect
        (progn
          (subscribe-bounding-box-topic)
          (ros::rate 3)
          (while (and (ros::ok) (null *bounding-box-msg*) (< (inc cnt) max-count))
            (ros::ros-info "waiting bounding box... ~A" cnt)
            (ros::spin-once)
            (ros::sleep))
          (unless *bounding-box-msg*
            (ros::ros-error "timed out")
            (return-from find-fcn-object nil))
          (when (> (length (send *bounding-box-msg* :boxes)) 0)
            (setq target-box (car (send *bounding-box-msg* :boxes))))
          (unless target-box
            (ros::ros-error "target is not found in bounding boxes (length of boxes: ~A)"
                            (length (send *bounding-box-msg* :boxes)))
            (return-from find-fcn-object nil))
          (setq ret (bounding-box-msg->cube target-box))
          (when (< (send ret :volume) box-volume-min)
            (ros::ros-error "target bounding box volume is too small: ~A < ~A"
                            (send ret :volume) box-volume-min)
            (return-from find-fcn-object nil))
          (ros::ros-info "found ~A (volume: ~A)"
                         (elt target-names label-id) (send ret :volume))
          ret)
      (progn
        (unsubscribe-bounding-box-topic)
        (setq *bounding-box-msg* nil)))))

(provide :find-object) ;; end of find-object.l
