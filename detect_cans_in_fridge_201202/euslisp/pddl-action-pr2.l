;; pddl-action-pr2.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "detect_cans_in_fridge_201607_demo_pddl")

(require :fridge-main "package://detect_cans_in_fridge_201202/euslisp/main.l")
(require :find-object "package://detect_cans_in_fridge_201202/euslisp/find-object.l")
(require :wander-pr2 "package://detect_cans_in_fridge_201202/euslisp/wander-pr2.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :place-object "package://detect_cans_in_fridge_201202/euslisp/place-object.l")

(setq *use-arm* :larm)
(setq *detected-object* nil)

(defun find-object (target)
  (let (ret)
    (cond
     ((eql target 'can)
      (setq ret t))
     ((eql target 'fridge)
      (setq *detected-object* (find-fcn-object "refrigerator")))
     (t
      (setq *detected-object* (find-fcn-object (string-downcase target)))))
    (or ret (not (null *detected-object*)))))

(defun approach-to (target)
  (cond
    ((eql target 'can)
     (grasp-can :move-only t :pre-move t :post-move nil :use-arm *use-arm*))
    ((eql target 'table)
     (approach-to-target *detected-object* :move-back 1000.0)
     t)
    (t
     (pr2-tuckarm-pose)
     (approach-to-target *detected-object*))))

(defun open-door (&optional args)
  (open-fridge-door :use-arm *use-arm* :max-retry 3))

(defun look-around (&optional args)
  (send *pr2* :head :neck-p :joint-angle 15)
  (send *pr2* :head :neck-y :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :go-pos-unsafe 0 0 30)
  t)

#|
(setq *look-around-factor* 0.15)
(defun look-around (&optional args)
  (let* ((angle-range (- (send *pr2* :head :neck-y :max-angle)
                         (send *pr2* :head :neck-y :min-angle)))
         (angle (send *pr2* :head :neck-y :joint-angle))
         (angle-diff (* *look-around-factor* angle-range)))
    (ros::ros-info "angle: ~A range: ~A diff: ~A" angle angle-range angle-diff)
    (cond
     ((< (send *pr2* :head :neck-y :max-angle) (+ angle angle-diff))
      (send *ri* :go-pos-unsafe 0 0 180)
      (send *pr2* :head :neck-y :joint-angle (+ (send *pr2* :head :neck-y :min-angle)
                                                (random angle-diff))))
     (t
      (send *pr2* :head :neck-y :joint-angle (+ angle angle-diff))))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    t))
|#
#|
(defun look-around (&optional args)
  (let ((angle-range (- (send *pr2* :head :neck-y :max-angle)
                        (send *pr2* :head :neck-y :min-angle))))
    (send *pr2* :head :neck-y :joint-angle
          (+ (send *pr2* :head :neck-y :min-angle)
             (abs (random angle-range))))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation :timeout 1.0))
  t)
|#
(defun close-door (&optional args)
  (grasp-can :move-only t :pre-move nil :post-move t :use-arm *use-arm*)
  (close-fridge :use-arm *use-arm*))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can :use-arm *use-arm* :move nil)))

(defun place-object (&optional args)
  (place-object-to-table))

(defun task-compiler/after-init-hook ()
  (init))

(defun task-compiler/before-run-hook (sm)
  (setq *detected-object* nil)
  (disable-behavior-server)
  (initialize-demo "georgia"))

(defun task-compiler/after-run-hook (sm)
  (enable-behavior-server))

(defun debug-init ()
  (task-compiler/after-init-hook)
  (task-compiler/before-run-hook nil))

(provide :pddl-action-pr2) ;; end of pddl-action-pr2.l
