(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")

(ros::load-ros-manifest "jsk_demo_common")

(ros::roseus "pick2_server")

(warn "pick2 server is working and press any key to stop.")
(setq pick2-action-server
      (instance ros::simple-action-server :init "/pick2"
                jsk_demo_common::PR2CommonAction
                :execute-cb `(lambda (s g)
                               (let ((result-msg (send s :result)))
                                 (pick2 (ros::tf-pose-stamped->coords  (send g :goal :position))
                                        :arm (intern (string-upcase (send g :goal :arm)) *keyword-package*)
                                        :rotation-axis (intern (string-upcase (send g :goal :rotation_axis))  *keyword-package*)
                                        :debug-view (send g :goal :debug_view))
                                 (send *ri* :wait-interpolation)
                                 (send result-msg :result :joint_state (send *pr2* :angle-vector))
                                 (send s :set-succeeded result-msg)
                                 ))))

(ros::rate 10)
(pr2-init)
(do-until-key
 (ros::spin-once)
 (send pick2-action-server :worker)
 (ros::sleep))

