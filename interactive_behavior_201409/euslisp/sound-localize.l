#!/usr/bin/env roseus

(ros::roseus "sound_localization")
(ros::roseus-add-msgs "jsk_hark_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-msgs "visualization_msgs")

;; for PCA calculation
(defun pca (x)
  (let* ((mat (apply #'matrix x))
         (len (length (car x)))
         (mea (vector-mean x))
         (cen (copy-object mat))
         vs cov ret)
    ;; cov = (covariance x)
    (dotimes (i len)
      (push (matrix-column mat (- len i 1)) vs))
    (setq cov (apply #'covariance-matrix vs))
    ;; cen = (center x)
    (dotimes (i (array-dimension cen 0))
      (setf (matrix-row cen i)
            (v- (matrix-row cen i) mea)))
    ;; pc : principal components
    ;; z  : transformed data
    ;; w  : eigenvalues of covariance matrix
    (setq ret (sv-decompose cov)) ;; U D pc
    ;; returns (pc (center(x)*pc) w v-mean)
    (list (elt ret 2) (m* cen (elt ret 2)) (elt ret 1) mea)
    ))

(defun covariance-matrix (&rest vs)
  "make covariance matrix of given input vector-list(vs)"
  (let (m n)
    (setq n (length vs))
    (setq m (make-matrix n n))
    (dotimes (i n m)
      (dotimes (j n)
	(setf (aref m i j) (covariance (elt vs i) (elt vs j)))))))

(defun covariance (v1 v2)
  "make co-variance of vector, covariance #f(1 2 3) #(0 2 4)->1.3"
  (let* ((a 0.0) (m1 (vmean v1)) (m2 (vmean v2)) e1 e2)
    (dotimes (i (length v1))
      (setq e1 (elt v1 i) e2 (elt v2 i))
      (setq a (+ a (* (- e1 m1) (- e2 m2)))))
    (/ a (- (length v1) 1))))

(defun vmean (v)
  "make mean of vector, vmean #f(1 2 3)->2.0"
  (let ((a 0.0))
    (dotimes (i (length v))
      (setq a (+ a (elt v i))))
    (/ a (length v))))

(defun elipse-estimation (vs)
  ;; vs is a list of 2-dimensional float-vector
  ;; return '(primaly axis, secondoly axis)
  (let ((pca-res (pca (mapcar #'(lambda (v)
                                  (float-vector (elt v 0) (elt v 1) 0))
                              vs))))
    (list (scale (elt (caddr pca-res) 0) (matrix-column (car pca-res) 0))
          (scale (elt (caddr pca-res) 1) (matrix-column (car pca-res) 1)))))

(defun elipse-ratio (vs)
  (multiple-value-bind
   (primary secondary) (elipse-estimation vs)
   (/ (norm primary) (norm secondary))))

(defun power-area (2d-power-points)
  (let ((area 0))
    (dotimes (i (1- (length 2d-power-points)))
      (let ((v0 (elt 2d-power-points i))
            (v1 (elt 2d-power-points (1+ i))))
        (if (and (> (norm v0) 0.01)
                 (> (norm v1) 0.01))
            (let ((f (instance face :init :vertices 
                               (list (float-vector 0 0 0)
                                     v0
                                     v1))))
              (setq area (+ (send f :area) area))))))
    area))
  
(defun mean-powers ()
  (let ((ret nil))
    (dotimes (i (length (car *prev-data*)))
      (let* ((this-channel-data (mapcar #'(lambda (samples)
                                           (elt samples i))
                                       *prev-data*))
             (sum (reduce #'+ this-channel-data)))
        (push (/ sum (length *prev-data*)) ret)
      ))
    (reverse ret)))

(defun sound-cb (msg)
  (let ((powers (coerce (send msg :powers) cons)))
    (when *prev-data*
      (let ((biased-powers (mapcar #'(lambda (v pv)
                                       (max 0 (- v (max *sound-bias* pv))))
                                   powers  (mean-powers))))
        ;;(print biased-powers)
        ;; map to 2d points
        (let ((2d-power-points nil))
          (dotimes (i (length biased-powers))
            (let ((theta (- (* (/ 2pi (length powers)) i) pi)))
              (let ((x (* (elt biased-powers i) (cos theta)))
                    (y (* (elt biased-powers i) (sin theta))))
                (push (float-vector x y 0) 2d-power-points))))
          (setq 2d-power-points (reverse 2d-power-points))
          ;; compute area
          (let ((area (power-area 2d-power-points)))
            (let* ((elipse-axis (elipse-estimation 2d-power-points))
                   (elipse-ratio (elipse-ratio 2d-power-points)))
              (ros::ros-info "area: ~A" area)
              (ros::ros-info "elipse-ratio: ~A" elipse-ratio)
              (if (and (> area *sound-area-threshold*)
                       (> elipse-ratio *elipse-ratio-threshold*))
                  (let ((primary-direction (normalize-vector (car elipse-axis))))
                  (let ((sound-direction
                         (check-direction biased-powers primary-direction)))
                    (ros::ros-info "direction: ~A" sound-direction)
                    (let ((vmsg (instance geometry_msgs::Vector3Stamped :init)))
                      (send vmsg :header (send msg :header))
                      (send vmsg :vector 
                            (ros::pos->tf-translation (scale 1000.0 sound-direction)))
                      (ros::publish "sound_direction" vmsg)
                      ;; publish marker
                      (let ((coords (make-coords)))
                        (orient-coords-to-axis coords sound-direction :x)
                        (ros::publish 
                         "sound_direction_marker"
                         (print (arrow->marker-msg 
                          coords (send msg :header)
                          :scale (float-vector 1000 100 100))))
                        (publish-sound-pictogram t)
                        ))))
                (let ((marker (instance visualization_msgs::Marker :init)))
                  (send marker :action visualization_msgs::Marker::*DELETE*)
                  (send marker :header (send msg :header))
                  (ros::publish "sound_direction_marker" marker)
                  (publish-sound-pictogram nil)
                  )))))))
    (push powers *prev-data*)
    (if (> (length *prev-data*) *max-sampling-data*)
        (setq *prev-data* (subseq *prev-data* 0 *max-sampling-data*))
      )))

(defun check-direction (biased-powers primary-direction)
  ;; direction might be flipped, so we need to check the suitable 
  (let ((direction-theta (acos (v. (float-vector -1 0 0) 
                                   (normalize-vector primary-direction))))
        (direction-theta* nil))
    (if (> (v. (float-vector 0 0 1) (v* (float-vector -1 0 0) primary-direction)) 0)
        (setq direction-theta* direction-theta)
      (setq direction-theta* (- 2pi direction-theta)))
  (let* ((N (length biased-powers))
         (N/2 (/ N 2))
         (direction-index (floor (* N (/ direction-theta* 2pi))))
         (direction-index-reversed
          (if (>= direction-index N/2)
              (- direction-index N/2)
            (+ direction-index N/2))))
    (let ((p (elt biased-powers direction-index))
          (p* (elt biased-powers direction-index-reversed)))
      (if (> p p*) primary-direction (scale -1 primary-direction))))))

(defun publish-sound-pictogram (addp)
  (let ((msg (instance jsk_rviz_plugins::Pictogram :init)))
    (send msg :header :frame_id "head_pan_link")
    (send msg :header :stamp (ros::time-now))
    (send msg :size 0.3)
    (send msg :character "sound")
    (send msg :color :r (/ 25 255.0))
    (send msg :color :g 1.0)
    (send msg :color :b (/ 240 255.0))
    (send msg :color :a 1.0)
    (send msg :pose :position :z 0.5)
    (send msg :pose :orientation :w 0.7)
    (send msg :pose :orientation :y -0.7)
    (if addp
        (send msg :action jsk_rviz_plugins::Pictogram::*ADD*)
      (send msg :action jsk_rviz_plugins::Pictogram::*DELETE*))
    (ros::publish "sound_pictogram" msg)
    ))

(ros::advertise "sound_direction" geometry_msgs::Vector3Stamped)
(ros::advertise "sound_direction_marker" visualization_msgs::Marker)
(ros::advertise "sound_pictogram" jsk_rviz_plugins::Pictogram)

(ros::subscribe
 "/HarkPower"
 jsk_hark_msgs::HarkPower
 #'sound-cb)

(setq *prev-data* nil)                                ;dynamic bias
(setq *max-sampling-data* (ros::get-param "~max_sampling_for_bias" 10))
(setq *sound-bias* (ros::get-param "~sound_bias" 28.0)) ;minimum bias
(setq *elipse-ratio-threshold* (ros::get-param "~elipse_ratio_threshold" 2.0)) ;minimum bias
(setq *sound-area-threshold* (ros::get-param "~sound_area_thr" 1.3))
(ros::spin)

