(ros::roseus "detect_cans_in_fridge_201202_demo_pddl")

(load "main.l")

(setq *use-arm* :larm)
(setq *finish-type* :pass-to-human)

(defun move-to (from to)
  (when (eql from 'start)
    (when (eql to 'frontfridge)
      (goto-frint-of-fridge)))
  (when (eql from 'frontfridge)
    (when (eql to 'pregrasp)
      (grasp-can :move-only t :pre-move t)))
  (when (eql from 'pregrasp)
    (when (eql to 'preclose)
      (grasp-can :move-only t :post-move t)))
  (when (eql from 'preclose)
    (when (eql to 'start)
      (goto-initial-position :finish-type *finish-type* :use-arm *use-arm*))))

(defun open-door ()
  (open-fridge-door :use-arm *use-arm*))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can :use-arm *use-arm*)))

(defun close-door ()
  (close-fridge :use-arm *use-arm*))

(defun move-recoverly ()
  (goto-front-of-fridge))

(defun try-close ()
  (close-fridge :use-arm *use-arm*))
