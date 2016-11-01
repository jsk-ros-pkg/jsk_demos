;; fetch-and-place-action.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defparameter *detection-topic* "/kinect_head/rgb/ObjectDetection")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
;;(require :pr2-action "package://jsk_demo_common/euslisp/pr2-attention-action.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun setup-env ()
  (let ((eng2 (make-eng2-scene))
        scene)
    ;; adjust offset of 73b2
    (send (send eng2 :object "/eng2/7f")
          :translate (float-vector 0 -6900 -30000))
    (setq scene (send eng2 :object "/eng2/7f/73b2"))
    (send (send scene :spot "table-spot")
          :translate (float-vector -1000 -1000 0) :world)
    scene))

(defun task-compiler/after-init-hook ()
  ;; init context
  (setq *current-context* "fetch_and_place")
  (setq *scene* (setup-env))
  (pr2-init)

  ;; initial location of robot
  (send *pr2* :reset-coords)
  (send *ri* :robot :reset-coords)
  (let ((cds (send (send *scene* :spot "init-spot") :worldcoords)))
    (send *pr2* :transform cds)
    (send *ri* :robot :transform cds))

  ;; visualization
  (objects (list *pr2* *scene*))
  (send *ri* :objects (list *scene*))
  )

(defun find-object (&optional target-sym)
  (let ((target-name (string target-sym))
        (target (send *scene* :object (format nil "room73b2-~A" target-sym))))
    (ros::ros-info "target: ~A ~A ~A" target-sym target-name target)
    (when (send *ri* :simulation-modep)
      (send target :reset-coords)
      (send target :transform *pr2*)
      (send target :translate (float-vector 800 0 700) *pr2*)
      (return-from find-object t))
    (not (null (check-detection :target-object target :speak-name target-name)))))

(defun pick (&optional target-sym arm-sym)
  (let ((target (send *scene* :object (format nil "room73b2-~A" target-sym)))
        (arm (intern (string arm-sym) *keyword-package*)))
    (unless (send target :handle)
      (return-from pick nil))
    (dolist (handle (send target :handle))
      (block continue
        (ros::ros-info "trying pick at ~A" handle)
        (unless (send *pr2* arm :inverse-kinematics handle)
          (ros::ros-error "no ik found")
          (return-from continue))
        (unless (send *pr2* arm :move-end-pos (float-vector -100 0 0))
          (ros::ros-error "no pre ik found")
          (return-from continue))
        ;; pre pick pose
        (send *pr2* :look-at-target (send *pr2* arm :end-coords))
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation)
        ;; open gripper
        (send *ri* :stop-grasp arm :wait t)
        ;; reach to target object
        (send *pr2* arm :move-end-pos (float-vector 100 0 0))
        (send *pr2* :look-at-target (send *pr2* arm :end-coords))
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation)
        (let (end-coords-before-pick diff-pose)
          (unwind-protect
               (progn
                 ;; close gripper
                 (send *ri* :start-grasp arm :objects (list target))
                 (setq end-coords-before-pick
                       (send *pr2* arm :end-coords :copy-worldcoords))
                 ;; pick it up
                 (send *pr2* arm :move-end-pos (float-vector 0 0 200) *pr2*)
                 (send *pr2* :look-at-target (send *pr2* arm :end-coords))
                 (send *ri* :wait-interpolation)
                 ;; save as prop
                 (send *pr2* :put arm t)
                 (return-from pick t))
            (progn
              (when end-coords-before-pick
                (setq diff-pose
                    (send (send *pr2* arm :end-coords :copy-worldcoords)
                          :difference-position end-coords-before-pick))
                (when (> (norm diff-pose) 10)
                  ;; revert arm pose
                  (send *pr2* arm :move-end-pos diff-pose *pr2*)
                  (send *ri* :angle-vector (send *pr2* :angle-vector))
                  (Send *ri* :wait-interpolation)))
              ;; open gripper
              (send *ri* :stop-grasp arm :wait t)
              (send *pr2* arm :move-end-pos (float-vector -100 0 0))
              (send *ri* :angle-vector (send *pr2* :angle-vector))
              (send *ri* :wait-interpolation))))))

    (ros::ros-error "no valid handle")
    (return-from pick nil)))

(defun place (&optional target-sym arm-sym)
  (let ((target (send *scene* :object (format nil "room73b2-~A" target-sym)))
        (arm (intern (string arm-sym) *keyword-package*)))
    ;; FIXME (perceive table and place on
    (send *pr2* arm :move-end-pos (float-vector 0 0 -300) *pr2*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp arm :wait t)
    (send *pr2* arm :move-end-pos (float-vector -100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :put arm nil))
  t)

(defun place-on (&optional target-sym onto-sym arm-sym)
  ;; FIXME (perceive table and place on
  (place target arm))

(defun move-to (&optional destination-sym)
  (let ((dest (send *scene* :spot (format nil "~A-spot" destination-sym))))
    ;; tuckarm pose before navigation
    (cond
      ((or
        (send *pr2* :get :arms)
        (and (send *pr2* :get :rarm) (send *pr2* :get :larm)))
       ;; do not tuck arm
       t)
      ((send *pr2* :get :rarm)
       (pr2-tuckarm-pose :rarm :outside))
      ((send *pr2* :get :larm)
       (pr2-tuckarm-pose :larm :outside))
      (t
       (pr2-tuckarm-pose)))
    (prog1
        (not (null (send *ri* :move-to dest)))
      (send *pr2* :move-to dest :world))))

(provide :fetch-and-place-action) ;; end of fetch-and-place-action.l
