#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::roseus-add-msgs "people_msgs")
(ros::roseus-add-msgs "jsk_hark_msgs")
(ros::roseus "interactive_behavior_server")

;; initialization
(ros::ros-info "initializing")
(pr2-init)
(setq *tfl* (instance ros::transform-listener :init))
(setq *behavior-server-enabled* nil)
(setq *current-behavior* :idle)
(setq *behavior-timeout* 0)
(setq *behavior-start-at* (ros::time-now))
;; behaviors
;;   :idle
;;   :look-at-face
;;   :look-at-sound

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
p	(setf (aref m i j) (covariance (elt vs i) (elt vs j)))))))

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

(defun check-behavior-timeout ()
  (let ((now (ros::time-now)))
    (if (not (eq *current-behavior* :idle))
      (when (> (send (ros::time- now *behavior-start-at*) :to-sec) *behavior-timeout*)
        (ros::ros-warn "~A is timeout (behavior timeout was ~A)" *current-behavior* *behavior-timeout*)
        (setq *current-behavior* :idle))
      (progn                            ;idle -> idle
        (if (= (random 50) 0)          ;2%
            (progn
              (ros::ros-info "random-lookat")
              (run-behavior :idle-random-look)))))))

(defun run-behavior (behavior &rest args)
  (when *behavior-server-enabled*
    (setq *current-behavior* behavior)
    (setq *behavior-start-at* (ros::time-now))
    (case behavior
      (:look-at-face
       (apply #'look-at-pos args)
       )
      (:look-at-sound
       (apply #'look-at-pos args)
       )
      (:idle-random-look
       (look-at-pos-random)))))
  
(defun look-at-pos-random ()
  (let ((theta (deg2rad (- (random 30.0) 15))))
    (ros::ros-info "theta is ~A" theta)
    (let ((local-pos (float-vector (* 3000.0 (cos theta))
                                   (* 3000.0 (sin theta))
                                   0)))
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       "/head_pan_link"
                       (ros::time 0))))
        (if cds (look-at-pos (send cds :transform-vector local-pos)))))))

(defun look-at-pos (world-pos)
  (ros::ros-info "looking at ~A" world-pos)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((current-pitch (send *pr2* :head :neck-p :joint-angle))
        (current-yaw (send *pr2* :head :neck-y :joint-angle)))
    (send *pr2* :head :look-at world-pos) ;LOOK AT!!
    (let ((next-pitch (send *pr2* :head :neck-p :joint-angle))
          (next-yaw (send *pr2* :head :neck-y :joint-angle)))
      (let ((diff (max (abs (- next-pitch current-pitch))
                       (abs (- next-yaw current-yaw)))))
        (let ((tm (/ diff 40.0)))
          (send *ri* :angle-vector (send *pr2* :angle-vector) (* 1000.0 tm) :head-controller)
          (setq *behavior-timeout* (* 2 tm))
          )))))

(defun position-measurement-cb (msg)
  (when (or (eq *current-behavior* :idle)
          (eq *current-behavior* :idle-random-look)
          (eq *current-behavior* :look-at-sound)
          (eq *current-behavior* :look-at-face))
      (setq msg (elt (send msg :people) 0))
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       (send msg :header :frame_id)
                       (send msg :header :stamp))))
        (if cds
            (progn
              (let ((world-pos (send cds :transform-vector (ros::tf-point->pos (send msg :pos)))))
                (let ((distance-from-robot (distance
                                            (float-vector (elt world-pos 0)
                                                          (elt world-pos 1)
                                                          0)
                                            (float-vector 0 0 0))))
                  (if (< distance-from-robot 2000.0)
                      (run-behavior :look-at-face world-pos)))))
          (progn
            (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id)))))))

(defun find-max-powers-index (powers)
  (let ((max-value (elt powers 0))
        (max-index 0))
    (dotimes (i (length powers))
      (when (< max-value (elt powers i))
        (setq max-value (elt powers i))
        (setq max-index i))
      )
    max-index))
  

(defun sound-cb (msg)
  ;; offset ... - pi
  (if (or (eq *current-behavior* :idle)
          (eq *current-behavior* :idle-random-look)
          (eq *current-behavior* :look-at-sound))
      ;; (send *tfl* :wait-for-transform 
      ;;       "/base_footprint"
      ;;       (send msg :header :frame_id)
      ;;       (send msg :header :stamp) 1)
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       ;;(send msg :header :frame_id)
                       "/head_pan_link"
                       (send msg :header :stamp))))
                       ;;(ros::time- (send msg :header :stamp) (ros::time 0.5)))))
                       ;;(ros::time 0))))
        (if cds
            (let ((powers (send msg :powers)))
              ;; look up maximum position
              (let* ((max-index (find-max-powers-index powers))
                     (max-power (elt powers max-index)))
                (when (and (> max-index 4) (> max-power 32))
                    (ros::ros-info "max-power: ~A" max-power)
                  (ros::ros-info "max-index: ~A/~A" max-index (length powers))
                    ;; look at direction!!
                    (let ((theta (- (* (/ 2pi (length powers)) max-index) pi)))
                      ;; compute the posistion to look at
                      (let ((local-pos (float-vector (* 3000 (cos theta)) (* 3000 (sin theta)) 0)))
                        (let ((world-pos (send cds :transform-vector local-pos)))
                          (run-behavior :look-at-sound world-pos)
                          ))
                      )
                  ))))))
  )

(defun enable-cb (req)
  (setq *behavior-server-enabled* t)
  (send req :response))

(defun disable-cb (req)
  (setq *behavior-server-enabled* nil)
  (send req :response))

;; subscribe topics
(ros::subscribe
 "/face_detector/people_tracker_measurements_array"
 people_msgs::PositionMeasurementArray
 #'position-measurement-cb)
(ros::subscribe
 "/HarkPower"
 jsk_hark_msgs::HarkPower
 #'sound-cb)
(ros::advertise-service
 "/interactive_behavior_enable"
 std_srvs::Empty
 #'enable-cb)
(ros::advertise-service
 "/interactive_behavior_disable"
 std_srvs::Empty
 #'disable-cb)

(ros::ros-info "running main loop")

;;(ros::spin)

(ros::rate 5)
(while (ros::ok)
  (check-behavior-timeout)
  (ros::spin-once)
  (ros::sleep))

