(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://roseus_smach/src/state-machine-actionlib.l")

(ros::roseus-add-msgs "jsk_demo_common")

(ros::roseus "pick_server")

(warn "pick server is working and press any key to stop.")
(setq pick-action-server (instance ros::simple-action-server :init "/pick"
				   jsk_demo_common::democommonaction
				   :execute-cb `(lambda (s g)
						  (let ((result-msg (send s :result)))
						    (pr2-init)
						    (pick (ros::tf-pose-stamped->coords  (send g :goal :pos))
							  :arm (intern (string-upcase (send g :goal :arm)) *keyword-package*)
							  :debug-view (send g :goal :debug-view))
						    (send result-msg :result :joint_state)
						    (send s :set-succeeded result-msg)
						    ))))


(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send pick-action-server :worker)
 (ros::sleep))

