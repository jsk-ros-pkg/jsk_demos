(defvar *points-topic* "/kinect_head/depth_registered_half/points")
(defvar *fix-frame-id* "/base_footprint")
(defvar *map-frame-id* "/odom_combined")

(unless (boundp '*tfl*)
  (unless (ros::ok)
    (ros::roseus "lookaround_pr2"))
  (setq *tfl* (instance ros::transform-listener :init)))

(defun get-one-points (&key (timeout 1000) (stamp (ros::time-now)))
  (let ((msg
         (one-shot-subscribe *points-topic* sensor_msgs::PointCloud2
                             :timeout timeout :after-stamp stamp)))
    (when msg
      (let ((st (send msg :header :stamp))
            (fm (send msg :header :frame_id))
            cds)
        (setq cds (send *tfl* :lookup-transform *fix-frame-id* fm st))
        (unless cds
          (when (send *tfl* :wait-for-transform *fix-frame-id* fm st 0.5)
            (setq cds (send *tfl* :lookup-transform *fix-frame-id* fm st))))
        (when cds
          (setf (get msg :base-transform) cds))
        ))
    msg))

(defun look-around-pr2 (&key (relative-p 12) (relative-y 16) (tm 1000) (timeout 1000))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((org-av (send *pr2* :head :angle-vector))
        (p-ang (send *pr2* :head :neck-p :joint-angle))
        (y-ang (send *pr2* :head :neck-y :joint-angle))
        ylst ret)
    ;; check min-max angle
    (if (> (+ p-ang relative-p) (send *pr2* :head :neck-p :max-angle))
        (setq p-ang (- (send *pr2* :head :neck-p :max-angle) relative-p)))
    (if (< (- p-ang relative-p) (send *pr2* :head :neck-p :min-angle))
        (setq p-ang (+ (send *pr2* :head :neck-p :min-angle) relative-p)))
    (if (> (+ y-ang relative-y) (send *pr2* :head :neck-y :max-angle))
        (setq y-ang (- (send *pr2* :head :neck-y :max-angle) relative-y)))
    (if (< (- y-ang relative-y) (send *pr2* :head :neck-y :min-angle))
        (setq y-ang (+ (send *pr2* :head :neck-y :min-angle) relative-y)))

    (setq ylst (list (- y-ang relative-y) (+ y-ang relative-y)))
    (dolist (pp (list (- p-ang relative-p) (+ p-ang relative-p)))
      (send *pr2* :head :neck-p :joint-angle pp)
      (dolist (yy ylst)
        (send *pr2* :head :neck-y :joint-angle yy)
        (send *ri* :angle-vector (send *pr2* :angle-vector) tm)
        (send *ri* :wait-interpolation)
        (push (get-one-points :timeout timeout) ret))
      (setq ylst (nreverse ylst)))
    (send *pr2* :head :angle-vector org-av)
    (send *ri* :angle-vector (send *pr2* :angle-vector) tm)
    (send *ri* :wait-interpolation)

    ret))

#|
(let ((ret (look-around-pr2 :timeout 3000))
      lst)
  (dolist (r ret)
    (let ((pts
           (make-eus-pointcloud-from-ros-msg r :remove-nan t))
          (cds (get r :base-transform)))
      (when cds
        (send pts :transform cds))
      (setf (get pts :header) (send r :header))
      (push pts lst)
      ))
  lst)
|#
