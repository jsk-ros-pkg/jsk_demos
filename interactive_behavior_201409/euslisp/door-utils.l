#!/usr/bin/env roseus
;; door-utils.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "door")

(ros::load-ros-manifest "jsk_recognition_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-force-utils "package://interactive_behavior_201409/euslisp/pr2-force-utils.l")

(defparameter *door-handle* nil)
(defvar *base-frame-id* (ros::get-param "~base_frame_id" "base_footprint"))

(defun handle-bbox-cb (msg)
  (let ((dim (float-vector
              (send msg :dimensions :x)
              (send msg :dimensions :y)
              (send msg :dimensions :z)))
        ps cds)
    (when (<= (norm dim) 0)
      (return-from handle-bbox-cb nil))
    (setq ps (instance geometry_msgs::PoseStamped :init
                       :header (send msg :header)
                       :pose (send msg :pose)))
    (setq cds (send *tfl* :transform-pose *base-frame-id* ps))
    (unless cds
      (ros::ros-error "failed to transform-pose ~A -> ~A"
                      (send msg :header :frame_id) *base-frame-id*)
      (return-from handle-bbox-cb nil))
    (send cds :rotate pi/2 :y)
    (setq *door-handle* cds)))

(warn ";; find-handle ")
(defun find-handle ()
  (pr2-tuckarm-pose :larm :outside)
  (send *pr2* :head :neck-p :joint-angle 40)
  (send *pr2* :head :neck-y :joint-angle 15)
  (send *ri* :head-angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (setq *door-handle* nil)
  (with-subscribe ("/door_detector/door_handle_box"
                   jsk_recognition_msgs::BoundingBox
                   #'handle-bbox-cb)
    (if (wait-until-set 10 *door-handle*)
        *door-handle*
        (progn (ros::ros-error "failed to find door handle") nil))))

(warn ";; go-to-handle ")
(defun go-to-handle (handle)
  (let ((target-cds  (make-cascoords :coords (send handle :copy-worldcoords)))
        (desired-cds (make-cascoords :coords (send *pr2* :copy-worldcoords)))
        (diff-max 0.1)
        diff-pos diff-rot diff)
    (send target-cds :rotate -pi/2 :x)
    (send desired-cds :translate (float-vector 780 300 (elt (send target-cds :worldpos) 2)))
    (setq diff-pos (scale 0.001 (send desired-cds :difference-position target-cds))
          diff-rot (send desired-cds :difference-rotation target-cds))
    (setq diff (norm (float-vector (elt diff-pos 0) (elt diff-pos 1) (elt diff-rot 0))))

    (when (and x::*display* (>= x::*display* 0))
      (objects (list *pr2* target-cds desired-cds)))

    (ros::ros-info "target-cds: ~A" target-cds)
    (ros::ros-info "desired-cds: ~A" desired-cds)
    (ros::ros-info "diff-pos: ~A" diff-pos)
    (ros::ros-info "diff-rot: ~A" diff-rot)
    (ros::ros-info "diff: ~A" diff)
    (if (< diff diff-max)
        (return-from go-to-handle t)
       (send *ri* :go-pos-unsafe (elt diff-pos 0) (elt diff-pos 1) (rad2deg (elt diff-rot 0)))
        )
    nil))

(warn ";; knock-door ")
(defun knock-door (handle arm)
  (let ((door-cds (make-cascoords :coords (send handle :copy-worldcoords)))
        (handle->door (float-vector 50 -300 0))
        preknock-av knock-av)
    (send *pr2* :reset-pose)
    (send door-cds :translate handle->door *pr2*)
    (setq knock-av
          (send *pr2* arm :inverse-kinematics
                door-cds
                :move-target (send *pr2* arm :end-coords)
                :link-list (send *pr2* :link-list (send *pr2* arm :end-coords :parent))
                :rotation-axis nil
                :use-torso t
                :look-at-target t))
    (unless knock-av
      (ros::ros-error "failed to solve ik for door-cds: ~A" door-cds)
      (return-from knock-door nil))
    (send door-cds :translate (float-vector -100 0 100) *pr2*)
    (setq preknock-av
          (send *pr2* arm :inverse-kinematics
                door-cds
                :move-target (send *pr2* arm :end-coords)
                :link-list (send *pr2* :link-list (send *pr2* arm :end-coords :parent))
                :rotation-axis nil
                :use-torso t
                :look-at-target t))
    (unless preknock-av
      (ros::ros-error "failed to solve ik for pre door-cds: ~A" door-cds)
      (return-from knock-door nil))

    (send *ri* :angle-vector (send *pr2* :reset-pose) 3000
          (if (eq arm :rarm) :rarm-controller :larm-controller))
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector-sequence
          (list preknock-av knock-av preknock-av knock-av preknock-av)
          (list 4000 500 1000 500 1000)
          (if (eq arm :rarm) :rarm-controller :larm-controller))
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *pr2* :reset-pose) 3000
          (if (eq arm :rarm) :rarm-controller :larm-controller))
    (send *ri* :wait-interpolation)
    t))

(warn ";; grasp-handle ")
(defun grasp-handle (arm handle &optional (offset (float-vector 30 10 0)))
  (let (pregrasp-av grasp-av target)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq target (make-cascoords :coords (send *pr2* :copy-worldcoords)))
    (send target :translate (send handle :worldpos))
    (send target :translate offset)
    (send target :rotate pi/2 :x)
    (when (and x::*display* (>= x::*display* 0))
      (objects (list *pr2* target)))

    (send target :translate (float-vector -150 0 0) *pr2*)
    (setq pregrasp-av
          (send *pr2* :inverse-kinematics target
                :move-target (send *pr2* arm :end-coords)
                :link-list (send *pr2* :link-list (send *pr2* arm :end-coords :parent))
                :rotation-axis t
                :use-torso t
                :look-at-target t))
    (send target :translate (float-vector 150 0 0) *pr2*)
    (setq grasp-av
          (send *pr2* :inverse-kinematics target
                :move-target (send *pr2* arm :end-coords)
                :link-list (send *pr2* :link-list (send *pr2* arm :end-coords :parent))
                :rotation-axis t
                :use-base nil
                :use-torso t
                :look-at-target t))

    (unless (and pregrasp-av grasp-av)
      (ros::ros-error "ik fail")
      (return-from grasp-handle nil))

    ;; run motion
    (pr2-tuckarm-pose arm :outside)
    (send *ri* :stop-grasp arm :wait t)

    (send *ri* :angle-vector-sequence
          (list pregrasp-av grasp-av)
          (list 5000 2000))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp arm :gain 1.0)
    (ros::ros-info "gripper width: ~A" (send *ri* :start-grasp arm :gain 1.0))

    (send *ri* :reset-wrench arm)
    (send *pr2* :angle-vector grasp-av)
    (send *pr2* arm :move-end-pos (float-vector -5 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (let ((wrench (send *ri* :wrench arm)))
      (ros::ros-info "wrench: ~A" wrench)
      (when (< (abs (elt wrench 0)) 1.0)
        (ros::ros-error "failed to grasp handle")
        (send *ri* :stop-grasp arm :wait t)
        (send *ri* :angle-vector pregrasp-av)
        (send *ri* :wait-interpolation)
        (pr2-tuckarm-pose arm :outside)
        (return-from grasp-handle nil)))

    (send *pr2* arm :move-end-pos (float-vector 5 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    t))


(defun unlatch-door (arm type rot)
  "arm: :larm or :rarm
   type: :pull or :push
   rot: degrees to unlatch"
  (let ((pos 100))
    (if (eq type :push)
        (setq rot (abs rot) pos (abs pos))
        (setq rot (- (abs rot)) pos (- (abs pos))))

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* arm :wrist-r :joint-angle
          (+ (send *pr2* arm :wrist-r :joint-angle) rot))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp arm :gain 1.0)
    (send *pr2* arm :move-end-pos (float-vector pos 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (let ((prev-pos (send *pr2* arm :end-coords :worldpos))
          cur-pos)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq cur-pos (send *pr2* arm :end-coords :worldpos))
      (unless (< (norm (v- cur-pos prev-pos)) 50)
        (send *ri* :go-pos-unsafe (* 0.01 pos) 0 0)))

    (send *ri* :start-grasp arm :gain 0.3)
    (send *pr2* arm :wrist-r :joint-angle
          (- (send *pr2* arm :wrist-r :joint-angle) rot))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)))

(defun pull-door (radius &optional (dth 30))
  "radius: door radius (mm)
   dth: diff theta (degree)
  "
  (let ((th 0) cds arm->base)
    (while (ros::ok)
      (when (>= th 60)
        (return))
      (send *ri* :reset-wrench :larm)
      (send *pr2* :larm :move-end-pos (float-vector -5 0 0))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      (let ((wrench (send *ri* :wrench :larm)))
        (ros::ros-info "wrench: ~A" wrench)
        (when (< (abs (elt (send *ri* :wrench :larm) 0)) 1.0)
          (ros::ros-error "Gripper slipped")
          (return)))
      (send *pr2* :larm :move-end-pos (float-vector 5 0 0))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
      (send *ri* :wait-interpolation)
      ;;
      (setq cds (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
      (send cds :translate (float-vector (- (* radius (sin (deg2rad dth))))
                                         0.0
                                         (- (* radius (- 1.0 (cos (deg2rad dth)))))))
      (send cds :rotate (deg2rad (- dth)) :y)
      (setq arm->base
            (send (send *pr2* :larm :end-coords :copy-worldcoords)
                  :transformation
                  (send *pr2* :copy-worldcoords)))
      (send cds :transform arm->base)

      (when (and x::*display* (>= x::*display* 0))
        (objects (list *pr2* cds)))

      (send *ri* :go-pos-unsafe
            (* 0.001 (elt (send *pr2* :difference-position cds) 0))
            (* 0.001 (elt (send *pr2* :difference-position cds) 1))
            (rad2deg (elt (send *pr2* :difference-rotation cds) 2)))
      (incf th dth)))
  ;;
  ;; rotate base
  (dotimes (i 3)
    (let (cds init-base diff-base)
      (setq init-base (send *pr2* :copy-worldcoords))
      (setq cds (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
      (send cds :assoc *pr2*)
      (send cds :rotate (deg2rad 15) :z)
      (send cds :dissoc *pr2*)
      (setq diff-base (send init-base :transformation (send *pr2* :copy-worldcoords)))
      (ros::ros-info "diff-base: ~A" diff-base)
      (send *ri* :go-pos-unsafe
            (* 0.001 (elt (send diff-base :pos) 0))
            (* 0.001 (elt (send diff-base :pos) 1))
            (rad2deg (caar (send diff-base :rpy-angle))))))

  (send *ri* :go-pos-unsafe -0.1 0.3 0)
  (send *ri* :go-pos-unsafe -0.1 0.3 0)
  (send *ri* :stop-grasp :larm :wait t)
  )

(defun push-door (arm radius &optional (dth 15))
  ;; first stretch arm as long as possible
  (let ((th 0) cds av diff-cds)
    (setq diff-cds
          (send (send *pr2* :copy-worldcoords) :transformation
                (send *pr2* arm :end-coords :copy-worldcoords))
    (while (ros::ok)
      (setq av (send *pr2* arm :move-end-pos (float-vector 100 0 0)))
      (unless av
        (return)))

    (when (>= th 80)
      (incf th dth))
    ;; adjust base position according with handle position

    )))

(defun open-door ()
  (let (handle)
    (while (ros::ok)
      (setq handle (find-handle))
      (when (go-to-handle handle)
        (return)))
    (grasp-handle :larm handle)
    (unlatch-door :larm :pull 50)
    (pull-door 600)))

(provide :door-utils)
(warn "~%")
