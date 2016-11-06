;; fetch-and-place-action.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :pr2-logging-interface "package://jsk_robot_startup/lifelog/pr2-logging-interface.l")

(defparameter *detection-topic* "/kinect_head_c2/rgb/ObjectDetection")

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

(defun task-compiler/before-run-hook (&optional sm)
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
  (start-task :pr2-fetch-and-place)
  )

(defun task-compiler/after-run-hook (&optional sm)
  (end-task))

(defaction find-object (&optional target-sym)
  (let* ((target-name (format nil "room73b2-~A" target-sym))
         (target (send *scene* :object target-name)))
    (when (send *ri* :simulation-modep)
      (send target :reset-coords)
      (send target :transform *pr2*)
      (send target :translate (float-vector 800 0 700) *pr2*)
      (let ((target-ri (car (send *ri* :find-object target))))
        (send target-ri :reset-coords)
        (send target-ri :transform *pr2*)
        (send target-ri :translate (float-vector 800 0 800) *pr2*))
      (return-from find-object t))
    ;; real robot
    (send *pr2* :head :neck-y :joint-angle 0)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (dotimes (i 3)
      (when
          (check-detection :type target-name
                           :speak nil
                           :target-object target)
        ;; adjust detected object
        (let ((cds (send target :copy-worldcoords)))
          (send target :newcoords (send *pr2* :copy-worldcoords))
          (send target :transform cds))
        (return-from find-object t)))))

(defaction approach (&optional target-sym arm-sym)
  (let ((target (send *scene* :object (format nil "room73b2-~A" target-sym))))
    (when (send *ri* :simulation-modep)
      ;; in eus simulation just rotate target around z axis
      (send target :rotate (deg2rad 30) :z :local)
      (let ((target-ri (car (send *ri* :find-object target))))
        (send target-ri :rotate (deg2rad 30) :z :local))
      t)
    ;; find ar place and move base (not implemented)
    t))


(defaction pick (&optional target-sym arm-sym)
  (let ((target (send *scene* :object (format nil "room73b2-~A" target-sym)))
        (arm (intern (string arm-sym) *keyword-package*)))
    (unless (send target :handle)
      (return-from pick nil))
    (dolist (handle (send target :handle))
      (block continue
        (let ((neck-p (send *pr2* :head :neck-p :joint-angle))
              (neck-y (send *pr2* :head :neck-y :joint-angle)))
          (pr2-pick-tray-pose)
          (send *pr2* :head :neck-p :joint-angle neck-p)
          (send *pr2* :head :neck-y :joint-angle neck-y)
          (send *ri* :angle-vector (send *pr2* :angle-vector))
          (send *ri* :wait-interpolation))
        (ros::ros-info "trying pick at ~A" handle)
        (unless
            (with-assoc-move-target
                (mt :move-target
                    (send (send *pr2* arm :end-coords :copy-worldcoords)
                          :translate (float-vector 100 0 0)) ;; offset
                    :parent-link (send *pr2* arm :end-coords :parent))
              (send *pr2* :inverse-kinematics
                    (make-coords :pos (send target :centroid))
                          :move-target (car mt)
                          :rotation-axis :z
                          :link-list (send *pr2* :link-list (send (car mt) :parent))))
          (ros::ros-error "no ik found")
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
                 (let ((gripper-pos
                         (send *ri* :start-grasp arm
                                    :objects (send *ri* :find-object target))))
                   (when (< gripper-pos 1.0)
                     (ros::ros-warn "failed to grasp ~A < 1.0" gripper-pos)
                     (return-from continue)))
                 (send *pr2* arm :end-coords :assoc target)
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
              (unless (send *pr2* :get arm)
                (ros::ros-info "recover grasp failure")
                (when end-coords-before-pick
                  (setq diff-pose
                        (send (send *pr2* arm :end-coords :copy-worldcoords)
                              :difference-position end-coords-before-pick))
                  (when (> (norm diff-pose) 10)
                    ;; revert arm pose
                    (send *pr2* arm :move-end-pos diff-pose *pr2*)
                    (send *ri* :angle-vector (send *pr2* :angle-vector))
                    (send *ri* :wait-interpolation)))
                ;; open gripper
                (send *ri* :stop-grasp arm :wait t)
                (send *pr2* arm :end-coords :dissoc target)
                (send *pr2* arm :move-end-pos (float-vector -100 0 0))
                (send *ri* :angle-vector (send *pr2* :angle-vector))
                (send *ri* :wait-interpolation)))))))

    (ros::ros-error "no valid handle")
    (return-from pick nil)))

(defaction place (&optional target-sym arm-sym)
  (let ((target (send *scene* :object (format nil "room73b2-~A" target-sym)))
        (place-pos (make-coords :pos (float-vector 700 0 700)))
        (arm (intern (string arm-sym) *keyword-package*)))
    ;; FIXME (perceive table and place on
    (when (send *ri* :simulation-modep)
      (let ((inverse-factor (if (eq arm :larm) 1 -1)) avs tms)
        (send *pr2* :torso_lift_joint :joint-angle 150)
        (send *pr2* arm :collar-y :joint-angle (* inverse-factor 70))
        (send *pr2* arm :elbow-p :joint-angle (* inverse-factor -120))
        (send *pr2* :head :look-at (send *pr2* arm :end-coords :worldpos))
        (push (send *pr2* :angle-vector) avs)
        (push 1500 tms)
        (send *pr2* arm :shoulder-p :joint-angle 0)
        (send *pr2* :head :look-at (send *pr2* arm :end-coords :worldpos))
        (push (send *pr2* :angle-vector) avs)
        (push 3000 tms)
        (send *pr2* arm :inverse-kinematics
              (send (send *pr2* :copy-worldcoords)
                    :translate (float-vector 800 0 1000) *pr2*)
                        :rotation-axis :z
                        :look-at-target t)
        (push (send *pr2* :angle-vector) avs)
        (push 3000 tms)
        (send *pr2* arm :inverse-kinematics
              (send (send *pr2* :copy-worldcoords)
                    :translate (float-vector 900 0 800) *pr2*)
                        :rotation-axis :z
                        :look-at-target t)
        (push (send *pr2* :angle-vector) avs)
        (push 1500 tms)
        (send *ri* :angle-vector-sequence (reverse avs) (reverse tms))
        (send *ri* :wait-interpolation))
      (send *ri* :stop-grasp arm :wait t)
      (send *pr2* arm :end-coords :dissoc target)
      (send *pr2* arm :move-end-pos (float-vector -100 0 0))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      (send *pr2* :put arm nil)))
  t)

(defaction place-on (&optional target-sym onto-sym arm-sym)
  ;; FIXME (perceive table and place on
  (place target arm))

(defaction move-to (&optional destination-sym)
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
        (not (null (send *ri* :move-to dest :frame-id "/eng2/7f/73B2")))
      (send *pr2* :move-to dest :world))))

(defun test-pick-bottle ()
  (task-compiler/before-run-hook)
  (move-to 'coe)
;;  (find-object 'bottle)
;;  (pick 'bottle 'rarm)
  (task-compiler/after-run-hook)
)

(defun test ()
  (task-compiler/before-run-hook)
  (move-to 'coe)
  (find-object 'bottle)
  (pick 'bottle 'rarm)
  (move-to 'cook)
  (find-object 'mug-cup)
  (approach 'mug-cup 'larm)
  (approach 'mug-cup 'larm)
  (pick 'mug-cup 'larm)
  (move-to 'table)
  (task-compiler/after-run-hook))


(provide :fetch-and-place-action) ;; end of fetch-and-place-action.l
