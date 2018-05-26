#!/usr/bin/env roseus
;; check-force.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "wrench_publisher")

(require :kalmanlib "irteus/kalmanlib.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; suppress print violate max/min-angle
(unless (assoc :joint-angle-org (send rotational-joint :methods))
  (rplaca (assoc :joint-angle (send rotational-joint :methods)) :joint-angle-org))
(defmethod rotational-joint
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
              (setq v max-angle)))
       (cond ((< v min-angle)
              (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  ) ;; defmethod rotational-joint

(defclass wrench-filter
  :slots (kf kt init-q-var offset arm frame wrench-topic wrench-raw-topic wrench-reset-service)
  :super propertied-object)
(defmethod wrench-filter
    (:init (limb ns &optional (q-var 1e-4))
     (setq arm limb
           offset (float-vector 0 0 0 0 0 0)
           ft (float-vector 0 0 0 0 0 0)
           init-q-var q-var
           kf (instance kalman-filter :init :state-dim 3 :measure-dim 3 :q-variance q-var)
           kt (instance kalman-filter :init :state-dim 3 :measure-dim 3 :q-variance q-var)
           frame (send (send *robot* arm :end-coords :parent) :name)
           wrench-topic (format nil "~A/wrench" ns)
           wrench-raw-topic (format nil "~A/wrench/raw" ns)
           wrench-reset-service (format nil "~A/wrench/reset" ns))
     (ros::advertise wrench-topic geometry_msgs::WrenchStamped 1)
     (ros::advertise wrench-raw-topic geometry_msgs::WrenchStamped 1)
     (ros::advertise-service wrench-reset-service std_srvs::Empty
                             #'send self :reset))
  (:publish (force torque topic stamp)
            (let ((msg (instance geometry_msgs::WrenchStamped :init)))
              (send msg :header :stamp stamp)
              (send msg :header :frame_id frame)
              (send msg :wrench :force :x (elt force 0))
              (send msg :wrench :force :y (elt force 1))
              (send msg :wrench :force :z (elt force 2))
              (send msg :wrench :torque :x (elt torque 0))
              (send msg :wrench :torque :y (elt torque 1))
              (send msg :wrench :torque :z (elt torque 2))
              (ros::publish topic msg)))
  (:proc (stamp)
         (let ((msg (instance geometry_msgs::WrenchStamped :init))
               (ft (send *robot* :calc-force-from-joint-torque arm
                         (send *robot* :torque-vector)))
               force torque)
           (setq force (subseq ft 0 3) torque (subseq ft 3 6))
           (send self :publish force torque wrench-raw-topic stamp)
           (setq ft (v- ft offset))
           (setq force (subseq ft 0 3) torque (subseq ft 3 6))
           (setq force (send kf :proc force))
           (setq torque (send kt :proc torque))
           (send self :publish force torque wrench-topic stamp)))
  (:reset (&optional req)
          (setq offset
                (send *robot* :calc-force-from-joint-torque arm
                      (send *robot* :torque-vector))
                kf (instance kalman-filter :init :state-dim 3 :measure-dim 3 :q-variance init-q-var)
                kt (instance kalman-filter :init :state-dim 3 :measure-dim 3 :q-variance init-q-var))
          (when req (send req :response))))

(setq *robot* (pr2))
(setq *wrench-larm* (instance wrench-filter :init :larm "l_arm_controller"))
(setq *wrench-rarm* (instance wrench-filter :init :rarm "r_arm_controller"))

(defun callback (msg)
  (apply-joint_state msg *pr2*)
  (send *wrench-larm* :proc (send msg :header :stamp))
  (send *wrench-rarm* :proc (send msg :header :stamp)))

(ros::rate 100)
(ros::subscribe "/joint_states" sensor_msgs::JointState #'callback 1)

(dotimes (i 300)
  (ros::spin-once)
  (ros::sleep))

(send *wrench-larm* :reset)
(send *wrench-rarm* :reset)

(ros::spin)
(exit)
