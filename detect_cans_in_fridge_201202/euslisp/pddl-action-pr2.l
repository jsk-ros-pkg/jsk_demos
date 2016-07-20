;; pddl-action-pr2.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "detect_cans_in_fridge_201607_demo_pddl")

(require :fridge-main "package://detect_cans_in_fridge_201202/euslisp/main.l")

(setq *use-arm* :larm)

(defun find-object (target)
  (cond
    ((eql target 'can)
     t)
    (t
     ;; TODO: find target by deep learning
     )))

(defun approach-to (target)
  (cond
    ((eql target 'can)
     (grasp-can :move-only t :pre-move t :post-move nil :use-arm *use-arm*))
    (t
     ;; TODO: move to target detected by deep learning
     t)))

(defun open-door (&optional args)
  (open-fridge-door :use-arm *use-arm*))

(defun look-around (&optional args)
  (warn "not implemented~%"))

(defun close-door (&optional args)
  (close-fridge :use-arm *use-arm*))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can :use-arm *use-arm* :move nil)))

(defun place-object (&optional args)
  (warn "to be implemented~%"))


(provide :pddl-action-pr2) ;; end of pddl-action-pr2.l
