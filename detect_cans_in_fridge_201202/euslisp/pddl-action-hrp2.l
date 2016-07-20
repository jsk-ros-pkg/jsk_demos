(ros::roseus "detect_cans_in_fridge_201202_demo_pddl")

(require :pr2-action "package://jsk_demo_common/euslisp/hrp2-action.l")

(defvar *use-voicetext* t)
(defvar *use-english* nil)
(defvar *logging* (ros::get-param "~logging" nil))
(defvar *debug-view* (ros::get-param "~debug_view" t))

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(hrp2jsknts-init)
(setq *robot* *hrp2jsknts*)

(objects (list *robot*))

(defun find-object (target)
  (cond
    ((eql target 'can)
     t)
    (t
     ;; TODO: find target by deep learning
     (warn "not yet implemented~%")
     )))

(defun approach-to (target)
  (cond
    ((eql target 'can)
     (move-from-frontfridge-to-pregrasp))
    (t
     (warn "not yet implemented~%"))))

(defun open-door (&optional args)
  (open-fridge-door))

(defun look-around (&optional args)
  (warn "not yet implemented~%"))

(defun close-door (&optional args)
  (move-from-pregrasp-to-preclose)
  (close-fridge))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can)))

(defun place-object (obj)
  (warn "to be implemented~%"))

(provide :pddl-action-hrp2) ;; end of pddl-action-hrp2.l
