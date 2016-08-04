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

(defun find-fcn-object (name &key (max-count 100) (box-area-min (* 600 200)))
  (let ((target-names (ros::get-param *approach-target-bounding-box-param*))
        (cnt 0)
        label-id target-box target-angle target-dist ret diagonal)
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
          (let ((now (ros::time-now)))
            (ros::rate 3)
            (inc cnt)
            (while (and (ros::ok) (< (inc cnt) max-count))
              (ros::spin-once)
              (ros::sleep)
              (cond
               ((null *bounding-box-msg*)
                (ros::ros-info "waiting bounding box... ~A" cnt))
               ((= (length (send *bounding-box-msg* :boxes)) 0)
                (ros::ros-info "no boxes")
                (setq *bounding-box-msg* nil)
                (return))
               (t
                (setq target-box (car (send *bounding-box-msg* :boxes)))
                (setq ret (bounding-box-msg->cube target-box))
                (setq diagonal (sort (send (send ret :box) :diagonal) #'>=))
                (cond
                 ((c-isnan (send (send ret :box) :volume))
                  (ros::ros-info "box too big")
                  (setq *bounding-box-msg* nil)
                  (return))
                 ((> (send now :to-sec) (send (send *bounding-box-msg* :header :stamp) :to-sec))
                  (ros::ros-error "target timestamp ~A is before now: ~A"
                                  (send (send *bounding-box-msg* :header :stamp) :to-sec)
                                  (send now :to-sec))
                  (setq *bounding-box-msg* nil))
                 ((< (* (aref diagonal 0) (aref diagonal 1)) box-area-min)
                  (ros::ros-error "target bounding box area is too small: ~A min: ~A"
                                  diagonal box-area-min)
                  (setq *bounding-box-msg* nil))
                 (t
                  (ros::ros-info "found ~A (volume: ~A, diagonal: ~A pose: ~A)"
                                 (elt target-names label-id) (send ret :volume)
                                 (send (send ret :box) :diagonal) (send ret :worldcoords))
                  (speak-jp (format nil "~A が見つかりました。" (string-downcase name)))
                  (return-from find-fcn-object ret))))))))
      (progn
        (unless *bounding-box-msg*
          (speak-jp (format nil "~A が見つかりませんでした。" (string-downcase name)))
          (ros::ros-error "timed out."))
        (unsubscribe-bounding-box-topic)
        (setq *bounding-box-msg* nil)))))

(provide :find-object) ;; end of find-object.l
