
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "close_fridge_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-action.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "close-fridge callback func called")
  (cond
   (*dryrun*
    )
   (t
    (ros::ros-info "hoge")
    (close-fridge :use-arm :larm)
    ))
  (send server :set-succeeded (send server :result))
  )

(setq s (instance ros::simple-action-server :init
                  "close_fridge" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

