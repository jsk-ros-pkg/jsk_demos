
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "base_go_to_fridge_init_pose_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-move.l")
  (pr2-init)
  )

(defun cb (server goal)
  (ros::ros-info "base-go-to-fridge-init-pose callback func called")
  (cond
   (*dryrun*
    )
   (t
    (base-go-to-fridge-init-pose) 
     ))
  (send server :set-succeeded (send server :result :value 1))
  )

(setq s (instance ros::simple-action-server :init
                  "base_go_to_fridge_init_pose" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)


