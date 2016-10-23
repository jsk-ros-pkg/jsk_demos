(ros::roseus "detect_cans_in_fridge_201202_demo_pddl")

(require :pr2-action "package://jsk_demo_common/euslisp/hrp2-action.l")
(require :find-object "package://detect_cans_in_fridge_201202/euslisp/find-object.l")

(defvar *use-voicetext* t)
(defvar *use-english* nil)
(defvar *logging* (ros::get-param "~logging" nil))
(defvar *debug-view* (ros::get-param "~debug_view" t))

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(hrp2jsknts-init)
(setq *robot* *hrp2jsknts*)

(objects (list *robot*))
(setq *detected-object* nil)

(defun find-object (target)
  (let (ret)
    (cond
     ((eql target 'can)
      (setq ret t))
     ((eql target 'fridge)
      (setq *base-frame-id* "/ground")
      (setq *detected-object* (find-fcn-object "refrigerator"))
      (setq *base-frame-id* "/BODY")
      )
     (t
      (setq *base-frame-id* "/ground")
      (setq *detected-object* (find-fcn-object (string-downcase target)))
      (setq *base-frame-id* "/BODY")
      ))
    (or ret (not (null *detected-object*)))
    ))

(defun approach-to (target)
  (cond
   ((eql target 'can)
    (move-from-frontfridge-to-pregrasp))
   (t
    (approach-to-target *detected-object*))
   ))

(defun open-door (&optional args)
  (open-fridge-door))

(defun look-around (&optional args)
  (send *ri* :go-pos 0 0 30)
  t)

(defun close-door (&optional args)
  (move-from-pregrasp-to-preclose)
  (close-fridge))

(defun grasp-object (obj)
  (when (eql obj 'can)
    (grasp-can)))

(defun place-object (&optional args)
  ()
  (place-can)
  )



(provide :pddl-action-hrp2) ;; end of pddl-action-hrp2.l
