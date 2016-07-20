;; pddl-action-pr2.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "detect_cans_in_fridge_201607_demo_pddl")

(require :fridge-main "package://detect_cans_in_fridge_201202/euslisp/main.l")
(require :find-object "package://detect_cans_in_fridge_201202/euslisp/find-object.l")
(require :wander-pr2 "package://detect_cans_in_fridge_201202/euslisp/wander-pr2.l")

(setq *use-arm* :larm)
(setq *detected-object* nil)

(defun find-object (target)
  (cond
    ((eql target 'can)
     t)
    ((eql target 'fridge)
     (setq *detected-object* (find-fcn-object "refrigerator")))
    (t
     (setq *detected-object* (find-fcn-object (string-down-case target))))))

(defun approach-to (target)
  (cond
    ((eql target 'can)
     (grasp-can :move-only t :pre-move t :post-move nil :use-arm *use-arm*))
    (t
     (pr2-tuckarm-pose)
     (approach-to-target *detected-object*))))

(defun open-door (&optional args)
  (open-fridge-door :use-arm *use-arm*))

(defun look-around (&optional args)
  (pr2-tuckarm-pose)
  (wander))

(defun close-door (&optional args)
  (close-fridge :use-arm *use-arm*))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can :use-arm *use-arm* :move nil)))

(defun place-object (&optional args)
  (place2 (float-vector 600 0 700) :use-torso nil :move-forward nil :arm *use-arm*))

(defun task-compiler/after-init-hook ()
  (init))

(defun task-compiler/before-run-hook (sm)
  (disable-behavior-server)
  (initialize-demo "georgia"))

(defun task-compiler/after-run-hook (sm)
  (enable-behavior-server))

(provide :pddl-action-pr2) ;; end of pddl-action-pr2.l
