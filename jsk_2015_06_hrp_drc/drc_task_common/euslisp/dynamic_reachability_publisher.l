#!/usr/bin/env roseus

(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "visualization_msgs")

(ros::roseus "dynamic_reachability_publisher")

(defun dynamic-reachability-publisher-init
  ()
  (setq *request-renew-reachability* nil)
  (ros::advertise "/dynamic_reachable_visible_map" visualization_msgs::marker 1)
  (ros::advertise-service "/calc_dynamic_reachability" moveit_msgs::GetPositionIK #'dynamic-reachability-call)
  (setq *reachability-coords-list* nil)
  )

(defun dynamic-reachability-call
  (req)
  (let* ((res (send req :response)))
    (setq *request-renew-reachability* req)
    res
    )
  )

(defun show-reachability-map 
  ()
  (let* ((msg (instance visualization_msgs::marker :init))
         (pose-msg (instance geometry_msgs::pose :init))
         points-list-msg colors-list-msg
         )
    (send msg :header (instance std_msgs::header :init
                                :stamp (ros::time-now) :frame_id "/jsk_model_marker_interface/robot/BODY"))
    (send msg :lifetime (ros::time 0.0))
    (send msg :frame_locked t)
    (send msg :type 6) ;; cube_list
    (send msg :action 0)
    (send pose-msg :orientation :w 1.0)
    (send msg :pose pose-msg)
    (send msg :scale (instance geometry_msgs::vector3 :init
                               :x 0.05
                               :y 0.05
                               :z 0.05))
    (dotimes (i (length *reachability-coords-list*))
      (push (instance geometry_msgs::point :init
                      :x (* 0.001 (elt (send (elt *reachability-coords-list* i) :pos) 0))
                      :y (* 0.001 (elt (send (elt *reachability-coords-list* i) :pos) 1))
                      :z (* 0.001 (elt (send (elt *reachability-coords-list* i) :pos) 2)))
            points-list-msg)
      (push (vector->rgba (float-vector 1 0 1)) colors-list-msg)
      )
    (send msg :points points-list-msg)
    (send msg :colors colors-list-msg)
    (ros::publish "/dynamic_reachable_visible_map" msg)
   )
  )

(defun exec-reachability
  ()
  (ros::ros-info "exec-reachability-start")
  (setq *reachability-coords-list* nil)
  (show-reachability-map)
  (let (tcl-for-arm tcl-tmp res map)
    (setq tcl-for-arm (reverse (cddr (reverse (mapcar #'ros::tf-pose-stamped->coords (cons (send *request-renew-reachability* :ik_request :pose_stamped) (send *request-renew-reachability* :ik_request :pose_stamped_vector)))))))
    
    (dotimes (index (length tcl-for-arm)) (push (make-coords) tcl-tmp))

    (dotimes (ix 4)
      (dotimes (iy 4)
        (dotimes (iz 4)
          (dotimes (index (length tcl-for-arm))
            (setf (elt tcl-tmp index) (send (send (elt tcl-for-arm index) :copy-worldcoords) :transform (make-coords :pos (float-vector (- (* ix 170) 255) (- (* iy 170) 255)  (- (* iz 170) 255))) :world))
            )
          (send *request-renew-reachability* :ik_request :pose_stamped :pose (ros::coords->tf-pose (car tcl-tmp)))
          (dotimes (index (- (length tcl-tmp) 1)) (send (elt (send *request-renew-reachability* :ik_request :pose_stamped_vector) index) :pose (ros::coords->tf-pose (elt tcl-tmp (+ 1 index)))))
          (setq res (ros::service-call "solve_ik" *request-renew-reachability* nil))
          (if (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*) 
              (push (elt tcl-tmp 0) *reachability-coords-list*)
            )
          )
        )
      )
    )
  (show-reachability-map)
  (setq *request-renew-reachability* nil)
  )


(dynamic-reachability-publisher-init)

(while t
  (ros::spin-once)
  (when *request-renew-reachability*
    (exec-reachability)
    )
  )
