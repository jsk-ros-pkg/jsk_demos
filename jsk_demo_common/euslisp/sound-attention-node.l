(ros::load-ros-manifest "jsk_hark_msgs")

(setq *initial-baias* 28.0)
(setq *average-count* 20)
(setq *average-power* 0.0)

(setq *av-counter* 0)
(setq *average-vector-list* nil)

(setq *store-counter* 0)
(setq *pub-msg* nil)

(defun vmean (v)
  (let ((ret 0.0))
    (dotimes (i (length v))
      (incf ret (elt v i)))
    (/ ret (length v))))

(defun mean-vector-list (vlst)
  (let ((ret (instantiate float-vector (length (car vlst)))))
    (dolist (v vlst)
      (v+ ret v ret))
    (scale (/ 1.0 (length vlst)) ret ret)
    ret
    ))

(defun callback (msg)
  (let ((ret 0.0)
        (pw (send msg :powers))
        ave)
    (push pw *average-vector-list*)
    (when (> (length *average-vector-list*) *average-count*)
      (setq *average-vector-list* (butlast *average-vector-list*))
      )
    (setq *average-power*
          (mean-vector-list *average-vector-list*))

    #| debug print
    (pprint (list *average-power*
                  (vmean *average-power*)
                  (v- pw *average-power*)
                  (apply #'max (coerce pw cons))))
    |#
    ;; average message
    (setq *pub-msg* (instance jsk_hark_msgs::HarkPower :init))
    (send *pub-msg* :header (send msg :header))
    (send *pub-msg* :count 0)
    (send *pub-msg* :directions (send msg :directions))
    (send *pub-msg* :data_bytes (send msg :data_bytes))
    (send *pub-msg* :powers (v- pw *average-power*))
    ))
(unless (ros::ok)
  (ros::roseus "hark_power_average"))

(ros::subscribe "/HarkPower" jsk_hark_msgs::HarkPower #'callback)
(ros::advertise "/HarkPower_ave" jsk_hark_msgs::HarkPower)

(setq *pmax-countr* 0)
(setq *pmax-threshold* 2.0)
(defun sound-attention ()
  (when *pub-msg*
    (ros::publish "/HarkPower_ave" *pub-msg*)
    (let ((powers (send *pub-msg* :powers))
          pmax)
      (setq *pub-msg* nil)
      (setq pmax (apply #'max (coerce powers cons)))
      (if (> pmax *pmax-threshold*) (incf *pmax-counter*))
      (when (> *pmax-counter* 1)
        (return sound-attention t))
      ;;
      ))
  nil)

#|
(do-until-key
 (ros::spin-once)
 (send *ri* :spin-once)
 (sound-attention)
 )
|#
