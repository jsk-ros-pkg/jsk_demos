#!/usr/bin/env roseus

#|
*origin-frame* -> global fixed frame
*robot-frame*  -> robot root frame
click_target_0 ~ click_target_N
|#
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defun set-dynamic-tf (in-coords parent-frame child-frame &optional (freq 50))
  (let* ((cur-tf (instance geometry_msgs::TransformStamped
                           :init
                           :header
                           (instance std_msgs::header :init :frame_id parent-frame
                                     :stamp (ros::time 0))
                           :child_frame_id child-frame
                           :transform (ros::coords->tf-transform in-coords)))
         (req (instance dynamic_tf_publisher::SetDynamicTFRequest
                        :init :freq freq :cur_tf cur-tf))
         )
    (ros::service-call "set_dynamic_tf" req)
    ))

(setq *cam-model* nil)
(setq *polygon* nil)

(defun callback-camerainfo (msg)
  (unless *cam-model*
    (setq *cam-model*
          (make-camera-from-ros-camera-info-aux
           (send msg :width)
           (send msg :height)
           (send msg :P)
           nil)
          )
    (send *cam-model* :put :frame_id (send msg :header :frame_id))
    (send *cam-model* :put :stamp    (send msg :header :stamp))
    )
  )
(defun callback-polygon (msg)
  (setq *polygon* msg)
  )

(defun get-click-targets-tf ()
  ;; get list of coordinates // *robot-frame* -> click_target_0, ... , click_target_N
  (when *target-list*
    (let (target-list)
      (dolist (tg *target-list*)
        (setq ret (send *tfl* :lookup-transform
                        *robot-frame* tg (ros::time 0)))
        (if ret
            (push ret target-list)
          (progn (ros::ros-warn "did not find TF (~A -> ~A)" *robot-frame* tg)))
        )
      (return-from get-click-targets-tf
        (mapcar #'(lambda (x) (send x :worldpos)) (reverse target-list)))
      ))
  (let (target-list (cntr 0) (ret t))
    (while ret
      (let ((target-name (format nil "click_target_~D" cntr)))
        (setq ret (send *tfl* :lookup-transform
                        *robot-frame* target-name (ros::time 0)))
        (when ret
          (push ret target-list)))
      (incf cntr))
    (mapcar #'(lambda (x) (send x :worldpos)) (reverse target-list))
    ))

(defun calc-polygon-to-coords (polygon cam-model)
  ;; get polygon
  (let* (
         (local-pos-lst (get-click-targets-tf))
         (world-pos-lst local-pos-lst)
         )
    (unless (string= *origin-frame* *robot-frame*)
      (let ((ori->robo (send *tfl* :lookup-transform *origin-frame* *robot-frame* (ros::time 0))))
        (unless ori->robo
          (ros::ros-warn "Can not find TF (~A -> ~A)" *origin-frame* *robot-frame*)
          (return nil))
        (setq world-pos-lst (mapcar #'(lambda (cc)
                                        (send ori->robo :transform-vector cc))
                                    local-pos-lst))
        ))

    (let ((pts (send (send polygon :polygon) :points))
          line-lst pix-lst)
      (when (< (length pts) (length local-pos-lst))
        (ros::ros-warn "This program require more then ~D points. Polygon has ~D points."
                       (length local-pos-lst) (length pts))
        (return-from calc-polygon-to-coords)
        )
      (setq pix-lst
            (mapcar #'(lambda (pt)
                        (float-vector (send pt :x) (send pt :y))) pts))
      ;;;
      ;;; publish result
      ;;;
      (let ((cds (optimization-projected-points-nlopt world-pos-lst pix-lst))
            pt-lst)
        ;;
        (ros::ros-info "set tf");
        (set-dynamic-tf cds *origin-frame* (send cam-model :get :frame_id))

        ;; for DEBUG
        (when *publish-debug*
          (let ((msg (instance geometry_msgs::PolygonStamped :init
                               :header (instance std_msgs::header :init
                                                 :frame_id *origin-frame*
                                                 :stamp (ros::time-now))
                               )))
            (send (send msg :polygon) :points
                  (mapcar #'(lambda (p)
                              (instance geometry_msgs::Point32 :init
                                        :x (* 0.001 (elt p 0))
                                        :y (* 0.001 (elt p 1))
                                        :z (* 0.001 (elt p 2))))
                          world-pos-lst))
            (ros::publish "~debug_target_poly" msg)
            ))
        (when *publish-debug*
          (send cam-model :reset-coords)
          (let ((msg (instance geometry_msgs::PolygonStamped :init
                               :header (instance std_msgs::header :init
                                                 :frame_id (send cam-model :get :frame_id)
                                                 :stamp (ros::time-now))
                               ))
                (pt-lst
                 (mapcar #'(lambda (uv)
                             (let ((pt (send cam-model :ray (elt uv 0) (elt uv 1)))
                                   scl)
                               (setq scl (/ 1500.0 (elt pt 2)))
                               (scale scl pt pt)
                               pt)) pix-lst))
                )
            (send (send msg :polygon) :points
                  (mapcar #'(lambda (p)
                              (instance geometry_msgs::Point32 :init
                                        :x (* 0.001 (elt p 0))
                                        :y (* 0.001 (elt p 1))
                                        :z (* 0.001 (elt p 2)))) pt-lst))
            (ros::publish "~debug_clicked_poly" msg)
            ))
        )
      )
    )
  )

;;
;; using eus_nlopt
;;
(load "package://eus_nlopt/euslisp/nlopt.l")

(defun param-to-coords (param)
  (make-coords :pos (subseq param 0 3)
               :rpy (subseq param 3 6)))

(defun evaluation-func (param)
  ;; global: *cam-model* *default-cam-coords* *wp-list* *uv-list*
  (let ((cds (param-to-coords param)))
    (send *cam-model* :newcoords cds)
    (send *cam-model* :transform *default-cam-coords* :world)
    (apply #'+
           (mapcar #'(lambda (wp uv)
                       (distance
                        (send *cam-model* :screen-point wp)
                        uv))
                   *wp-list* *uv-list*))
    ))
(defun-c-callable func-f ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) (evaluation-func v1))
  0)

(defun-c-callable func-df ((v1 :string) (v2 :string)) :integer
  (let ((org (evaluation-func v1))
        (diff-mm 1) (diff-rad 0.01))
    (setf (aref v2 0)
          (/ (- (evaluation-func (v+ (float-vector diff-mm 0 0 0 0 0) v1))
                org) diff-mm))
    (setf (aref v2 1)
          (/ (- (evaluation-func (v+ (float-vector 0 diff-mm 0 0 0 0) v1))
                org) diff-mm))
    (setf (aref v2 2)
          (/ (- (evaluation-func (v+ (float-vector 0 0 diff-mm 0 0 0) v1))
                org) diff-mm))
    (setf (aref v2 3)
          (/ (- (evaluation-func (v+ (float-vector 0 0 0 diff-rad 0 0) v1))
                org) diff-rad))
    (setf (aref v2 4)
          (/ (- (evaluation-func (v+ (float-vector 0 0 0 0 diff-rad 0) v1))
                org) diff-rad))
    (setf (aref v2 5)
          (/ (- (evaluation-func (v+ (float-vector 0 0 0 0 0 diff-rad) v1))
                org) diff-rad))
    0))
(defun-c-callable dummy-g ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) 0)
  0)
(defun optimization-projected-points-nlopt (wp-lst uv-lst)
  (setq *default-cam-coords*
        (make-coords :pos (float-vector (* 1000 *default-x*)
                                        (* 1000 *default-y*)
                                        (* 1000 *default-z*))
                     :rpy (float-vector *default-yaw* *default-pitch* *default-roll*)))
  (setq *wp-list* wp-lst)
  (setq *uv-list* uv-lst)
  ;;
  (let ((min-eval 1e100)
        (min-param (float-vector 0 0 0 0 0 0))
        curr-param
        curr-eval
        (init-state (float-vector 0 0 0 0 0 0))
        (min-state (float-vector -1400 -2000  -1000 -1.0 -1.0 -1.0))
        (max-state (float-vector  1400  2000   1500  1.0  1.0  1.0))
        (ftol 0.0005)
        )
    (dotimes (i 2)
      (dolist
          (alg
           (reverse (list
                     ;;DIRECT
                     ;;G_DIRECT
                     ;;DIRECT_L
                     ;;G_DIRECT_L
                     CRS
                     ;;STOGO ;un-supported
                     ;;ISRES
                     ;;
                     CCSA
                     ;;SLSQP
                     ;;L_BFGS
                     ;;TN
                     ;;SL_VM
                     ;;
                     COBYLA ;beta
                     BOBYQA
                     NEWUOA
                     ;;PRAXIS
                     NelderMeadSimplex
                     Sbplx
                     )))
        (setq curr-param
              (nlopt-optimize
               :initial-state    init-state
               :state-min-vector min-state
               :state-max-vector max-state
               :evaluation-function 'func-f
               :evaluation-function-gradient 'func-df
               :equality-function   'dummy-g ;; g = 0
               :equality-dimension 1
               :inequality-dimension 0
               :xtol -2
               :ftol ftol
               :max-time 3
               :alg alg
               ))
        (when (< (setq curr-eval (evaluation-func curr-param)) min-eval)
          (setq min-eval curr-eval
                min-param curr-param))
        )
      (pprint (list 'min min-eval min-param))
      ;; for 2nd round
      (setq ftol (* ftol 0.01))
      (setq init-state min-param)
      (setq max-state (v+ init-state (float-vector 500 500 500 0.5 0.5 0.5)))
      (setq min-state (v- init-state (float-vector 500 500 500 0.5 0.5 0.5)))
      )
    (setq cds (param-to-coords min-param))
    (send cds :transform *default-cam-coords* :world)
    cds
    ))

;;
;; ROSEUS node
;;

(ros::roseus "calc_camera_pose")

;; parameters
(setq *origin-frame* "map")
(when (ros::has-param "~origin_frame")
  (setq *origin-frame* (ros::get-param "~origin_frame")))
(ros::ros-info "origin_frame: ~A" *origin-frame*)

(setq *robot-frame* "BODY")
(when (ros::has-param "~robot_frame")
  (setq *robot-frame* (ros::get-param "~robot_frame")))
(ros::ros-info "robot_frame: ~A" *robot-frame*)

(setq *target-list* nil)
(when (ros::has-param "~target_list")
  (setq *target-list* (ros::get-param "~target_list")))
(ros::ros-info "target_list: ~A" *target-list*)

(setq *publish-debug* nil)
(when (ros::has-param "~debug")
  (setq *publish-debug* (ros::get-param "~debug")))
(ros::ros-info "debug: ~A" *publish-debug*)

;;; default coordinates
(setq *default-x* 3.0)
(when (ros::has-param "~DEFAULT_x")
  (setq *default-x* (ros::get-param "~DEFAULT_x")))
(ros::ros-info "default-x: ~A" *default-x*)
(setq *default-y* -1.2)
(when (ros::has-param "~DEFAULT_y")
  (setq *default-y* (ros::get-param "~DEFAULT_y")))
(ros::ros-info "default-y: ~A" *default-y*)
(setq *default-z* 1.0)
(when (ros::has-param "~DEFAULT_z")
  (setq *default-z* (ros::get-param "~DEFAULT_z")))
(ros::ros-info "default-z: ~A" *default-z*)

(setq *default-roll* -pi/2)
(when (ros::has-param "~DEFAULT_roll")
  (setq *default-roll* (ros::get-param "~DEFAULT_roll")))
(ros::ros-info "default-roll: ~A" *default-roll*)
(setq *default-pitch* 0.0)
(when (ros::has-param "~DEFAULT_pitch")
  (setq *default-pitch* (ros::get-param "~DEFAULT_pitch")))
(ros::ros-info "default-pitch: ~A" *default-pitch*)
(setq *default-yaw* pi/2)
(when (ros::has-param "~DEFAULT_yaw")
  (setq *default-yaw* (ros::get-param "~DEFAULT_yaw")))
(ros::ros-info "default-yaw: ~A" *default-yaw*)


;; services
(ros::wait-for-service "set_dynamic_tf")
(ros::ros-info "find set dynamic tf")

(when *publish-debug*
  ;; debug topic
  (ros::advertise "~debug_target_poly"  geometry_msgs::PolygonStamped)
  (ros::advertise "~debug_clicked_poly" geometry_msgs::PolygonStamped)
  )

;; subscribe
(ros::subscribe "camera_info"
                sensor_msgs::camerainfo
                #'callback-camerainfo)

;; subscribe
(ros::subscribe "poly"
                geometry_msgs::polygonstamped
                #'callback-polygon)


(defvar *tfl* (instance ros::transform-listener :init))

(ros::rate 10)

(while (ros::ok)
  (ros::spin-once)
  (when *polygon*
    ;; optimization
    (if *cam-model*
        (calc-polygon-to-coords *polygon* *cam-model*)
      (ros::ros-error "camera_info have not been received yet")
      )
    ;; prepare for next input
    (setq *cam-model* nil)
    (setq *polygon*   nil)
    )
  (ros::sleep)
  )
