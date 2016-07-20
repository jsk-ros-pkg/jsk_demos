(ros::roseus "detect_cans_in_fridge_201202_demo_pddl")

(require :fridge-main "package://detect_cans_in_fridge_201202/euslisp/main.l")

(setq *use-arm* :larm)
(setq *finish-type* :pass-to-human)

(defun move-to (from to)
  (cond
   ((and (eql from 'start) (eql to 'frontfridge))
    (goto-front-of-fridge))
   ((and (eql from 'frontfridge) (eql to 'pregrasp))
    (grasp-can :move-only t :pre-move t :post-move nil :use-arm *use-arm*))
   ((and (eql from 'pregrasp) (eql to 'preclose))
    (grasp-can :move-only t :pre-move nil :post-move t :use-arm *use-arm*))
   ((and (eql from 'preclose) (eql to 'start))
    (goto-initial-position :finish-type *finish-type* :use-arm *use-arm*))
   (t (error "invalid spot from: ~A to: ~A" from to))))

(defun open-door ()
  (open-fridge-door :use-arm *use-arm*))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can :use-arm *use-arm* :move nil)))

(defun close-door ()
  (close-fridge :use-arm *use-arm*))

(defun move-recoverly ()
  (goto-front-of-fridge))

(defun try-close ()
  (close-fridge :use-arm *use-arm*))

(defun task-compiler/after-init-hook ()
  (init))

(defun task-compiler/before-run-hook (sm)
  (disable-behavior-server)
  (initialize-demo "georgia"))

(defun task-compiler/after-run-hook (sm)
  (enable-behavior-server))
