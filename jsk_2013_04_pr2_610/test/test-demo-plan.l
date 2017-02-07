;; test-demo-plan.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(defparameter *msg* nil)
(defmacro subscribe-and-count (name)
  `(ros::subscribe ,name std_msgs::String
                   #'(lambda (m)
                       (ros::ros-info "Received ~A" (send m :data))
                       (push m *msg*))))

(init-unit-test)

(deftest test-demo-plan ()
  (setq *msg* nil)

  (subscribe-and-count "move_to")
  (subscribe-and-count "pick")
  (subscribe-and-count "place")
  (subscribe-and-count "put")
  (subscribe-and-count "open_door")
  (subscribe-and-count "close_door")
  (subscribe-and-count "push_button")
  (subscribe-and-count "sweep")
  (subscribe-and-count "sweep_under")
  (subscribe-and-count "pull_chair")
  (subscribe-and-count "push_chair")

  (ros::rate 1)
  (dotimes (i 100)
    (when (eq 20 (length *msg*))
      (return))
    (ros::ros-info "waiting for actions.. ~A" (length *msg*))
    (ros::spin-once)
    (ros::sleep))

  (ros::ros-warn "Received *msg* are ~A" (length *msg*))
  (assert (eq (length *msg*) 20))
)

(ros::roseus "test_demo_plan")
(run-all-tests)
(exit)
