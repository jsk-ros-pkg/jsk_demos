(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")

(ros::load-ros-manifest "jsk_demo_common")

(ros::roseus "place_server")

(warn "place server is working and press any key to stop.")
(setq place-action-server
      (instance ros::simple-action-server :init "/place"
                jsk_demo_common::PR2CommonAction
                :execute-cb `(lambda (s g)
                               (let ((result-msg (send s :result)))
                                 (place (ros::tf-pose-stamped->coords  (send g :goal :position))
                                        :arm (intern (string-upcase (send g :goal :arm)) *keyword-package*)
                                        :height (send g :goal :move_above_height)
                                        :debug-view (send g :goal :debug_view))
                                 (send *ri* :wait-interpolation)
                                 (send result-msg :result :joint_state (send *pr2* :angle-vector))
                                 (send s :set-succeeded result-msg)
                                 ))))

(ros::rate 10)
(pr2-init)
(do-until-key
 (ros::spin-once)
 (send place-action-server :worker)
 (ros::sleep))
