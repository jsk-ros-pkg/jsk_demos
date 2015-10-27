
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "move_forward_larm_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "move-forward-larm callback func called")
  (cond
   (*dryrun*
    )
   (t
    (move-forward-larm)
    ))
  (send server :set-succeeded (send server :result))
  )

(setq s (instance ros::simple-action-server :init
                  "move_forward_larm" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

