;; test-pr2-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(ros::load-ros-manifest "jsk_demo_common")
(ros::roseus "test_pr2_action_server")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(pr2-init)

(init-unit-test)

(setq *threshold* 6.0)

(deftest test-pick-action-server
    (let ((client (instance ros::simple-action-client :init
                            "pick" jsk_demo_common::PR2CommonAction))
          (goal (instance jsk_demo_common::PR2CommonActionGoal :init))
          err)

      (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
      (send client :wait-for-server)
      (pr2-reset-pose)
      (send goal :goal :position (ros::coords->tf-pose-stamped
                                  (make-coords :pos #f(600 0 800)) "base_footprint"))
      (send goal :goal :arm "rarm")
      (send goal :goal :rotation_axis ":z")
      (send goal :goal :debug_view nil)
      (send client :send-goal goal)

      (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
      (send client :wait-for-result)
      (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send client :get-result) :joint_state))
      (send *pr2* :angle-vector (send (send client :get-result) :joint_state))
      (setq err (norm (v- (send *pr2* :rarm :end-coords :worldpos)
                          #f(600 0 800))))
      (warning-message 2 ";; err: ~A (~A -> ~A) ~%" err (send *pr2* :rarm :end-coords :worldpos) #f(600 0 800))
      (assert (< err *threshold*)
              "result: ~A, err: ~A" (send (send client :get-result) :joint_state) err)))

(deftest test-pick2-action-server
    (let ((client (instance ros::simple-action-client :init
                            "pick2" jsk_demo_common::PR2CommonAction))
          (goal (instance jsk_demo_common::PR2CommonActionGoal :init))
          err)

      (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
      (send client :wait-for-server)
      (pr2-reset-pose)
      (send goal :goal :position (ros::coords->tf-pose-stamped
                                  (make-coords :pos #f(700 0 800)) "base_footprint"))
      (send goal :goal :arm "rarm")
      (send goal :goal :debug_view nil)
      (send client :send-goal goal)

      (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
      (send client :wait-for-result)
      (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send client :get-result) :joint_state))
      (send *pr2* :angle-vector (send (send client :get-result) :joint_state))
      (setq err (norm (v- (send *pr2* :rarm :end-coords :worldpos)
                          #f(750 0 800)
                          )))
      (warning-message 2 ";; err: ~A (~A -> ~A) ~%" err (send *pr2* :rarm :end-coords :worldpos) #f(750 0 800))
      (assert (< err *threshold*)
              "result: ~A, err: ~A" (send (send client :get-result) :joint_state) err)))

(deftest test-place-action-server
    (let ((client (instance ros::simple-action-client :init
                            "place" jsk_demo_common::PR2CommonAction))
          (goal (instance jsk_demo_common::PR2CommonActionGoal :init))
          err)

      (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
      (send client :wait-for-server)
      (pr2-reset-pose)
      (send goal :goal :position (ros::coords->tf-pose-stamped
                                  (make-coords :pos #f(600 0 800)) "base_footprint"))
      (send goal :goal :arm "rarm")
      (send goal :goal :rotation_axis ":z")
      (send goal :goal :move_above_height -50)
      (send goal :goal :debug_view nil)
      (send client :send-goal goal)

      (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
      (send client :wait-for-result)
      (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send client :get-result) :joint_state))
      (send *pr2* :angle-vector (send (send client :get-result) :joint_state))
      (setq err (norm (v- (send *pr2* :rarm :end-coords :worldpos)
                          #f(600 0 870)
                          )))
      (warning-message 2 ";; err: ~A (~A -> ~A) ~%" err (send *pr2* :rarm :end-coords :worldpos) #f(600 0 870))
      (assert (< err *threshold*)
              "result: ~A, err: ~A" (send (send client :get-result) :joint_state) err)))

(deftest test-place2-action-server
    (let ((client (instance ros::simple-action-client :init
                            "place2" jsk_demo_common::PR2CommonAction))
          (goal (instance jsk_demo_common::PR2CommonActionGoal :init))
          err)

      (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
      (send client :wait-for-server)
      (pr2-reset-pose)
      (send goal :goal :position (ros::coords->tf-pose-stamped
                                  (make-coords :pos #f(600 0 800)) "base_footprint"))
      (send goal :goal :arm "rarm")
      (send goal :goal :rotation_axis ":z")
      (send goal :goal :move_above_height -50)
      (send goal :goal :debug_view nil)
      (send client :send-goal goal)

      (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
      (send client :wait-for-result)
      (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send client :get-result) :joint_state))
      (send *pr2* :angle-vector (send (send client :get-result) :joint_state))
      (setq err (norm (v- (send *pr2* :rarm :end-coords :worldpos)
                          #f(600 0 800)
                          )))
      (warning-message 2 ";; err: ~A (~A -> ~A) ~%" err (send *pr2* :rarm :end-coords :worldpos) #f(600 0 800))
      (assert (< err *threshold*)
              "result: ~A, err: ~A" (send (send client :get-result) :joint_state) err)))


(run-all-tests)
(exit)
