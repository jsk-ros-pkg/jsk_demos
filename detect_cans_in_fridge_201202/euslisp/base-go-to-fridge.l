
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "base_go_to_fridge_server")

(defvar *dryrun* (ros::get-param "/dryrun"))
(when (null *dryrun*)
  (load "package://pr2eus/pr2-interface.l")
  (load "package://jsk_demo_common/euslisp/pr2-move.l")
  (pr2-init)
  (if (send *ri* :simulation-modep)
    (progn
      (load "package://jsk_maps/src/eng2-scene.l")
      (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
      (load "models/room73b2-scene.l")
      (room73b2)
      (send *ri* :objects (send *room73b2* :objects))
      ))
  )

(defun cb (server goal)
  (ros::ros-info "base-go-to-fridge callback func called" )
  (cond
   (*dryrun*
    )
   (t
    (base-go-to-fridge)
    ))
  (send server :set-succeeded (send server :result))
  )

(setq s (instance ros::simple-action-server :init
                  "base_go_to_fridge" roseus_smach::Sub5Action
                  :execute-cb 'cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)


