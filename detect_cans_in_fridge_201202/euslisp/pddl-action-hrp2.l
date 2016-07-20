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

(defun open-door ()
  (open-fridge-door))

(defun move-to (from to)
  (cond
   ((and (eql from 'start) (eql to 'frontfridge))
    ) ;; not yet
   ((and (eql from 'frontfridge) (eql to 'pregrasp))
    (move-from-frontfridge-to-pregrasp))
   ((and (eql from 'pregrasp) (eql to 'preclose))
    (move-from-pregrasp-to-preclose))
   ((and (eql from 'preclose) (eql to 'start))
    ) ;; not yet
   (t (error "invalid spot from: ~A to: ~A" from to))))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can)))

(defun close-door ()
  (close-fridge))
